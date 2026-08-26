const std = @import("std");
const spi = @import("spi.zig");
const gpio = @import("gpio.zig");
const radio = @import("radio.zig");

pub const Nrf905 = struct {
    io: std.Io,
    gpa: std.mem.Allocator,
    spi_dev: *spi.Spi,
    gpio_ctrl: *gpio.GpioCtrl,
    pins: Pins,
    settings: Settings,
    peer_addr: u32,

    pub const Pins = struct {
        tx_en_pin: u32,
        trx_ce_pin: u32,
        dr_pin: u32,
    };

    pub const Settings = struct {
        channel_num: u9 = 108,
        auto_retran: u1 = 0,
        rx_red_power: u1 = 0,
        pa_power: u2 = 0,
        hfreq_pll: u1 = 0,
        rx_addr_width: u3 = 0b100,
        tx_addr_width: u3 = 0b100,
        rx_payload_width: u6 = 0b100000,
        tx_payload_width: u6 = 0b100000,
        rx_addr: u32 = 0xE7E7E7E7,
        crc_mode: u1 = 1,
        crc_en: u1 = 1,
        cryst_freq: u3 = 0b100,
        up_clk_en: u1 = 1,
        up_clk_freq: u2 = 0b11,
    };

    const Logger = std.log.scoped(.nrf905);

    pub fn init(
        io: std.Io,
        gpa: std.mem.Allocator,
        spi_dev: *spi.Spi,
        gpio_ctrl: *gpio.GpioCtrl,
        pins: Pins,
        settings: Settings,
        peer_addr: u32,
    ) !Nrf905 {
        var new_self = Nrf905{
            .io = io,
            .gpa = gpa,
            .spi_dev = spi_dev,
            .gpio_ctrl = gpio_ctrl,
            .pins = pins,
            .settings = settings,
            .peer_addr = peer_addr,
        };

        try new_self.setMode(.standby);

        try std.Io.sleep(io, .fromMilliseconds(3), .real);

        const configs = [10]u8{
            @truncate(settings.channel_num),
            (@as(u8, settings.auto_retran) << 5) | (@as(u8, settings.rx_red_power) << 4) | (@as(u8, settings.pa_power) << 2) | (@as(u8, settings.hfreq_pll) << 1) | @as(u8, @intCast(settings.channel_num >> 8)),
            (@as(u8, settings.tx_addr_width) << 4) | @as(u8, settings.rx_addr_width),
            settings.rx_payload_width,
            settings.tx_payload_width,
            @truncate(settings.rx_addr),
            @truncate(settings.rx_addr >> 8),
            @truncate(settings.rx_addr >> 16),
            @truncate(settings.rx_addr >> 24),
            (@as(u8, settings.crc_mode) << 7) | (@as(u8, settings.crc_en) << 6) | (@as(u8, settings.cryst_freq) << 3) |
                (@as(u8, settings.up_clk_en) << 2) | @as(u8, settings.up_clk_freq),
        };
        try new_self.spiWriteConfigs(0, &configs);

        try std.Io.sleep(io, .fromMilliseconds(3), .real);

        Logger.info("Radio nRF905 ready", .{});

        return new_self;
    }

    pub fn deinit(self: *const Nrf905) void {
        _ = self;
    }

    pub fn checkDevice(self: *Nrf905) !bool {
        var buf: [1]u8 = undefined;
        try self.spiReadConfigs(3, &buf);
        const result = @popCount(buf[0]) == 1;
        if (result) {
            Logger.info("Device check passed! Reg[3] = 0x{x}", .{buf[0]});
        } else {
            Logger.warn("Device check failed! Reg[3] = 0x{x}", .{buf[0]});
        }
        return result;
    }

    pub fn configDump(self: *Nrf905) !void {
        var buf: [10]u8 = undefined;
        try self.spiReadConfigs(0, &buf);
        Logger.debug("Config: {any}", .{buf});
    }

    pub fn transmit(self: *Nrf905, dest_addr: u32, payload: []const u8) !void {
        if (payload.len > 32) {
            return error.IllegalLength;
        }

        // Logger.debug("Transmitting a packet with length {d}", .{
        //     payload.len,
        // });

        var msg_buf = std.mem.zeroes([33]u8);
        var trash_buf: [33]u8 = undefined;

        // Set TX address
        msg_buf[0] = 0b00100010;
        msg_buf[1] = @truncate(dest_addr);
        msg_buf[2] = @truncate(dest_addr >> 8);
        msg_buf[3] = @truncate(dest_addr >> 16);
        msg_buf[4] = @truncate(dest_addr >> 24);
        try self.spi_dev.transact(msg_buf[0..5], trash_buf[0..5]);

        // Send payload
        msg_buf[0] = 0b00100000;
        @memcpy(msg_buf[1..(payload.len + 1)], payload);
        try self.spi_dev.transact(&msg_buf, &trash_buf);

        // try std.Io.sleep(self.io, .fromMicroseconds(500), .real);
        try self.setMode(.tx);
        // try std.Io.sleep(self.io, .fromMilliseconds(1), .real);
        // try self.setMode(.standby);

        const gpio_mask: u64 = @as(u64, 1) << @intCast(self.pins.dr_pin);
        var gpio_val: u64 = undefined;
        var retry_count: u32 = 0;
        const max_retries: u32 = 200;
        while (retry_count < max_retries) {
            try std.Io.sleep(self.io, .fromMicroseconds(500), .real);
            retry_count += 1;

            gpio_val = try self.gpio_ctrl.get(gpio_mask);
            // _ = try self.checkDevice();
            // Logger.debug("gpio_val = 0x{x}", .{gpio_val});
            if ((gpio_val & (@as(u64, 1) << @intCast(self.pins.dr_pin))) != 0) {
                break;
            }
        }

        try self.setMode(.standby);
        // try std.Io.sleep(self.io, .fromMicroseconds(500), .real);

        if (retry_count >= max_retries) {
            Logger.warn("Max retry count ({d}) exceeded!", .{max_retries});
            return error.TooManyRetries;
        } else {
            // Logger.debug("Transmit took {d} retries", .{retry_count});
        }

        // Logger.debug("Transmission complete", .{});
    }

    pub fn receive(self: *Nrf905) !void {
        try self.setMode(.rx);

        // try std.Io.sleep(self.io, .fromMilliseconds(1), .real);
    }

    pub fn getReceived(self: *Nrf905) !?[32]u8 {
        const gpio_mask: u64 = (@as(u64, 1) << @intCast(self.pins.dr_pin));
        const gpio_val = try self.gpio_ctrl.get(gpio_mask);
        // _ = try self.checkDevice();
        // Logger.debug("gpio_val = 0x{x}", .{gpio_val});

        if (gpio_val & (@as(u64, 1) << @intCast(self.pins.dr_pin)) != 0) {
            try self.setMode(.standby);

            var tx_msg_buf = std.mem.zeroes([33]u8);
            var rx_msg_buf: [33]u8 = undefined;
            tx_msg_buf[0] = 0b00100100;
            try self.spi_dev.transact(&tx_msg_buf, &rx_msg_buf);

            // Logger.debug("Received a packet", .{});

            var result: [32]u8 = undefined;
            @memcpy(&result, rx_msg_buf[1..33]);
            return result;
        }
        return null;
    }

    pub fn getVtable(self: *const Nrf905) radio.Radio.VTable {
        _ = self;
        return .{
            .transmit_fn = interfaceTransmit,
            .receive_fn = interfaceReceive,
            .get_received_fn = interfaceGetReceived,
            .wait_fn = interfaceWait,
        };
    }

    fn spiReadConfigs(self: *const Nrf905, reg_start: u4, values: []u8) !void {
        var msg_tx_buf = std.mem.zeroes([17]u8);
        var msg_rx_buf: [17]u8 = undefined;
        msg_tx_buf[0] = 0x10 | @as(u8, reg_start);
        try self.spi_dev.transact(
            msg_tx_buf[0..(values.len + 1)],
            msg_rx_buf[0..(values.len + 1)],
        );
        @memcpy(values, msg_rx_buf[1..(values.len + 1)]);
    }

    fn spiWriteConfigs(self: *const Nrf905, reg_start: u4, values: []const u8) !void {
        var msg_tx_buf: [17]u8 = undefined;
        var msg_rx_buf: [17]u8 = undefined;
        msg_tx_buf[0] = 0x00 | reg_start;
        @memcpy(msg_tx_buf[1..(values.len + 1)], values);
        try self.spi_dev.transact(
            msg_tx_buf[0..(values.len + 1)],
            msg_rx_buf[0..(values.len + 1)],
        );
    }

    fn setMode(self: *Nrf905, mode: enum { standby, tx, rx }) !void {
        const gpio_mask: u64 = (@as(u64, 1) << @intCast(self.pins.trx_ce_pin)) | (@as(u64, 1) << @intCast(self.pins.tx_en_pin));
        const gpio_val: u64 = switch (mode) {
            .standby => (@as(u64, 0) << @intCast(self.pins.trx_ce_pin)) | (@as(u64, 0) << @intCast(self.pins.tx_en_pin)),
            .tx => (@as(u64, 1) << @intCast(self.pins.trx_ce_pin)) | (@as(u64, 1) << @intCast(self.pins.tx_en_pin)),
            .rx => (@as(u64, 1) << @intCast(self.pins.trx_ce_pin)) | (@as(u64, 0) << @intCast(self.pins.tx_en_pin)),
        };
        try self.gpio_ctrl.set(gpio_val, gpio_mask);
    }

    fn interfaceTransmit(self: *anyopaque, data: []const u8) error{TransmitError}!void {
        const self_typed: *Nrf905 = @ptrCast(@alignCast(self));
        self_typed.transmit(self_typed.peer_addr, data) catch return error.TransmitError;
    }

    fn interfaceReceive(self: *anyopaque) error{ReceiveError}!void {
        const self_typed: *Nrf905 = @ptrCast(@alignCast(self));
        self_typed.receive() catch return error.ReceiveError;
    }

    fn interfaceGetReceived(self: *anyopaque) error{ReceiveError}!?[]const u8 {
        const self_typed: *Nrf905 = @ptrCast(@alignCast(self));
        const result = self_typed.getReceived() catch return error.ReceiveError;
        if (result) |result_nn| {
            const msg = self_typed.gpa.alloc(u8, 32) catch return error.ReceiveError;
            @memcpy(msg, &result_nn);
            return msg;
        }
        return null;
    }

    fn interfaceWait(self: *anyopaque) error{WaitError}!void {
        const self_typed: *Nrf905 = @ptrCast(@alignCast(self));
        self_typed.gpio_ctrl.wait() catch return error.WaitError;
    }
};
