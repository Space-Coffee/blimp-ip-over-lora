const std = @import("std");
const spi = @import("spi.zig");
const gpio = @import("gpio.zig");
const tun = @import("tun.zig");
const radio = @import("radio.zig");
const nrf905 = @import("nrf905.zig");
const config = @import("config.zig");

pub fn main(init: std.process.Init) !void {
    // Read config file
    const cwd = std.Io.Dir.cwd();
    var conf_file = try cwd.openFile(init.io, "config.zon", .{});
    var conf_file_reader_buf: [1024]u8 = undefined;
    var conf_file_reader = conf_file.reader(init.io, &conf_file_reader_buf);
    var conf_content_writer_alloc = std.Io.Writer.Allocating.init(init.gpa);
    _ = try conf_file_reader.interface.streamRemaining(&conf_content_writer_alloc.writer);
    const conf_content_sentinel = try conf_content_writer_alloc.toOwnedSliceSentinel(0);
    defer init.gpa.free(conf_content_sentinel);
    const conf = try std.zon.parse.fromSliceAlloc(config.ConfigRoot, init.gpa, conf_content_sentinel, null, .{});

    std.log.scoped(.config).info("Configuration file read successfully!", .{});

    const spi_dev_path: [:0]const u8 = conf.spi_dev_path;
    var spi_dev = try spi.Spi.init(init.io, spi_dev_path);
    defer spi_dev.deinit(init.io);
    // try spi_dev.write_reg8(0b10110001, 0x00);

    const gpio_dev_path: [:0]const u8 = "/dev/gpiochip0";
    var gpio_ctrl = try gpio.GpioCtrl.init(
        init.io,
        gpio_dev_path,
        &.{ 23, 24 },
        &.{25},
    );
    defer gpio_ctrl.deinit(init.io);

    const tun_dev = try tun.Tun.init(init.io, init.gpa, "ip-over-lora");
    defer tun_dev.deinit(init.io, init.gpa);

    var nrf905_inst: ?nrf905.Nrf905 = nrf905_inst_blk: {
        if (conf.nrf905) |conf_nrf905_nn| {
            var nrf905_inst =
                try nrf905.Nrf905.init(
                    init.io,
                    init.gpa,
                    &spi_dev,
                    &gpio_ctrl,
                    .{
                        .tx_en_pin = 0,
                        .trx_ce_pin = 1,
                        .dr_pin = 0,
                    },
                    conf_nrf905_nn.settings,
                );
            _ = try nrf905_inst.checkDevice();
            try nrf905_inst.configDump();
            break :nrf905_inst_blk nrf905_inst;
        } else {
            break :nrf905_inst_blk null;
        }
    };
    defer {
        if (nrf905_inst) |nrf905_inst_nn| {
            nrf905_inst_nn.deinit();
        }
    }

    const radio_iface_vt: radio.Radio.VTable, const radio_iface_impl: *anyopaque =
        switch (conf.radio.impl) {
            .failing => .{ radio.Radio.VTable.failing, undefined },
            .dummy => .{ radio.Radio.VTable.dummy, undefined },
            .nrf905 => .{ nrf905_inst.?.getVtable(), @ptrCast(&nrf905_inst.?) },
        };
    var radio_iface = radio.Radio{
        .gpa = init.gpa,
        .vt = radio_iface_vt,
        .impl = radio_iface_impl,
        .packet_len_min = conf.radio.packet_len_min,
        .packet_len_max = conf.radio.packet_len_max,
    };
    defer radio_iface.deinit();
    // try radio_iface.transmit("Hello world!");
    try radio_iface.sendMessage(try init.gpa.dupe(u8, "Hello world!"));

    while (true) {
        _ = try radio_iface.update(init.io);
        try std.Io.sleep(init.io, .fromMilliseconds(25), .real);
    }
}
