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
    try conf_content_writer_alloc.writer.flush();
    const conf_content_sentinel = try conf_content_writer_alloc.toOwnedSliceSentinel(0);
    defer init.gpa.free(conf_content_sentinel);
    const conf = try std.zon.parse.fromSliceAlloc(config.ConfigRoot, init.gpa, conf_content_sentinel, null, .{});
    defer std.zon.parse.free(init.gpa, conf);

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

    var tun2radio_queue_buf: [256][]const u8 = undefined;
    var tun2radio_queue = std.Io.Queue([]const u8).init(&tun2radio_queue_buf);
    var radio2tun_queue_buf: [256][]const u8 = undefined;
    var radio2tun_queue = std.Io.Queue([]const u8).init(&radio2tun_queue_buf);

    var tun_dev = try tun.Tun.init(
        init.io,
        init.gpa,
        "ip-over-lora",
        conf.tun.local_addr,
        conf.tun.netmask,
        conf.tun.mtu,
    );
    defer tun_dev.deinit(init.io, init.gpa);

    _ = try init.io.concurrent(tun.Tun.worker, .{
        &tun_dev,
        init.io,
        init.gpa,
        &tun2radio_queue,
        &radio2tun_queue,
    });

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
                    conf_nrf905_nn.peer_addr,
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
        .empty_turns = conf.radio.max_empty_turns,
        .max_empty_turns = conf.radio.max_empty_turns,
        .turn_duration_ms = conf.radio.turn_duration_ms,
    };
    defer radio_iface.deinit();
    // try radio_iface.transmit("Hello world!");
    // try radio_iface.sendMessage(try init.gpa.dupe(u8, "Hello world!"));

    const SelectU = union(enum) {
        sleep: std.Io.Cancelable!void,
        tun_queue_read: error{ Canceled, Closed }![]const u8,
    };
    var select_buf: [2]SelectU = undefined;
    var select = std.Io.Select(SelectU).init(init.io, &select_buf);

    try select.concurrent(
        .sleep,
        std.Io.sleep,
        .{
            init.io,
            std.Io.Duration.fromMilliseconds(1),
            std.Io.Clock.real,
        },
    );
    try select.concurrent(
        .tun_queue_read,
        std.Io.Queue([]const u8).getOne,
        .{
            &tun2radio_queue, init.io,
        },
    );

    while (true) {
        const select_result = try select.await();
        switch (select_result) {
            .sleep => {
                while (true) {
                    const update_result = try radio_iface.update(init.io);
                    if (update_result.recv_msg) |recv_msg_nn| {
                        // defer init.gpa.free(recv_msg_nn);
                        try radio2tun_queue.putOne(init.io, recv_msg_nn);
                    }

                    if (!update_result.quick_update) {
                        break;
                    }
                }

                try select.concurrent(
                    .sleep,
                    std.Io.sleep,
                    .{
                        init.io,
                        std.Io.Duration.fromMicroseconds(200),
                        std.Io.Clock.real,
                    },
                );
            },
            .tun_queue_read => |tun_queue_read| {
                const msg = try tun_queue_read;
                try radio_iface.sendMessage(msg);

                try select.concurrent(
                    .tun_queue_read,
                    std.Io.Queue([]const u8).getOne,
                    .{
                        &tun2radio_queue, init.io,
                    },
                );
            },
        }
    }
}
