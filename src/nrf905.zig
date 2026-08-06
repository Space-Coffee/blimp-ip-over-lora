const std = @import("std");
const spi = @import("spi.zig");
const gpio = @import("gpio.zig");

pub const Nrf905 = struct {
    io: std.Io,
    spi_dev: *spi.Spi,
    gpio_ctrl: *gpio.GpioCtrl,
    pins: Pins,
    settings: Settings,

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
        spi_dev: *spi.Spi,
        gpio_ctrl: *gpio.GpioCtrl,
        pins: Pins,
        settings: Settings,
    ) !Nrf905 {
        const new_self = Nrf905{
            .io = io,
            .spi_dev = spi_dev,
            .gpio_ctrl = gpio_ctrl,
            .pins = pins,
            .settings = settings,
        };

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
        try new_self.spi_write_configs(0, &configs);

        Logger.info("Radio nRF905 ready", .{});

        return new_self;
    }

    pub fn deinit(self: *const Nrf905) void {
        _ = self;
    }

    pub fn transmit(self: *Nrf905, dest_addr: u32, payload: []const u8) !void {
        if (payload.len > 32) {
            return error.IllegalLength;
        }

        Logger.debug("Transmitting a packet with length {d}", .{
            payload.len,
        });

        var msg_buf = std.mem.zeroes([33]u8);
        var trash_buf: [33]u8 = undefined;

        // Set TX address
        msg_buf[0] = 0b00100010;
        msg_buf[1] = @truncate(dest_addr);
        msg_buf[1] = @truncate(dest_addr >> 8);
        msg_buf[1] = @truncate(dest_addr >> 16);
        msg_buf[1] = @truncate(dest_addr >> 24);
        try self.spi_dev.transact(msg_buf[0..5], trash_buf[0..5]);

        // Send payload
        msg_buf[0] = 0b0010000;
        @memcpy(msg_buf[1..(payload.len + 1)], payload);
        try self.spi_dev.transact(&msg_buf, &trash_buf);

        // Set TRX_CE and TX_EN
        var gpio_val: u64 = (1 << self.pins.trx_ce_pin) | (1 << self.pins.tx_en_pin);
        var gpio_mask: u64 = (1 << self.pins.trx_ce_pin) | (1 << self.pins.tx_en_pin);
        try self.gpio_ctrl.set(gpio_val, gpio_mask);

        gpio_mask = 1 << self.pins.dr_pin;
        while (true) {
            try std.Io.sleep(self.io, .fromMilliseconds(50), .real);

            gpio_val = self.gpio_ctrl.get(gpio_mask);
            if ((gpio_val & (1 << self.pins.dr_pin)) != 0) {
                break;
            }
        }

        gpio_val = (0 << self.pins.trx_ce_pin) | (0 << self.pins.tx_en_pin);
        gpio_mask = (1 << self.pins.trx_ce_pin) | (1 << self.pins.tx_en_pin);
        try self.gpio_ctrl.set(gpio_val, gpio_mask);
    }

    pub fn receive(self: *Nrf905) !void {
        const gpio_val: u64 = (1 << self.pins.trx_ce_pin) | (0 << self.pins.tx_en_pin);
        const gpio_mask: u64 = (1 << self.pins.trx_ce_pin) | (1 << self.pins.tx_en_pin);
        try self.gpio_ctrl.set(gpio_val, gpio_mask);
    }

    pub fn getReceived(self: *Nrf905) !?[32]u8 {
        var gpio_mask: u64 = (1 << self.pins.dr_pin);
        var gpio_val = try self.gpio_ctrl.get(gpio_mask);

        if (gpio_val & (1 << self.pins.dr_pin)) {
            gpio_val = (0 << self.pins.trx_ce_pin) | (0 << self.pins.tx_en_pin);
            gpio_mask = (1 << self.pins.trx_ce_pin) | (1 << self.pins.tx_en_pin);
            try self.gpio_ctrl.set(gpio_val, gpio_mask);

            var tx_msg_buf = std.mem.zeroes([33]u8);
            var rx_msg_buf: [33]u8 = undefined;
            tx_msg_buf[0] = 0b00100100;
            self.spi_dev.transact(tx_msg_buf, rx_msg_buf);

            Logger.debug("Received a packet", .{});

            var result: [32]u8 = undefined;
            @memcpy(&result, rx_msg_buf[1..33]);
            return result;
        }
        return null;
    }

    fn spi_read_configs(self: *const Nrf905, reg_start: u4, values: []u8) !void {
        var msg_tx_buf = std.mem.zeroes([17]u8);
        var msg_rx_buf: [17]u8 = undefined;
        msg_tx_buf[0] = 0x10 | reg_start;
        try self.spi_dev.transact(
            msg_tx_buf[0..(values.len + 1)],
            msg_rx_buf[0..(values.len + 1)],
        );
        @memcpy(values, msg_rx_buf[1..(values.len + 1)]);
    }

    fn spi_write_configs(self: *const Nrf905, reg_start: u4, values: []const u8) !void {
        var msg_tx_buf: [17]u8 = undefined;
        var msg_rx_buf: [17]u8 = undefined;
        msg_tx_buf[0] = 0x00 | reg_start;
        @memcpy(msg_tx_buf[1..(values.len + 1)], values);
        try self.spi_dev.transact(
            msg_tx_buf[0..(values.len + 1)],
            msg_rx_buf[0..(values.len + 1)],
        );
    }
};
