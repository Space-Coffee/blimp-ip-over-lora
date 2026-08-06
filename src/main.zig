const std = @import("std");
const spi = @import("spi.zig");
const gpio = @import("gpio.zig");
const tun = @import("tun.zig");
const radio = @import("radio.zig");
const nrf905 = @import("nrf905.zig");

pub fn main(init: std.process.Init) !void {
    const spi_dev_path: [:0]const u8 = "/dev/spidev0.0";
    var spi_dev = try spi.Spi.init(init.io, spi_dev_path);
    defer spi_dev.deinit(init.io);
    try spi_dev.write_reg8(0b10110001, 0x00);

    const gpio_dev_path: [:0]const u8 = "/dev/gpiochip0";
    var gpio_ctrl = try gpio.GpioCtrl.init(init.io, gpio_dev_path, &.{}, &.{});
    defer gpio_ctrl.deinit(init.io);

    const tun_dev = try tun.Tun.init(init.io, init.gpa, "ip-over-lora");
    defer tun_dev.deinit(init.io, init.gpa);

    var nrf905_inst = try nrf905.Nrf905.init(
        init.io,
        &spi_dev,
        &gpio_ctrl,
        .{ // TODO: Determine pin numbers
            .tx_en_pin = 1,
            .trx_ce_pin = 2,
            .dr_pin = 3,
        },
        .{},
    );
    defer nrf905_inst.deinit();

    var radio_iface = radio.Radio{
        .gpa = init.gpa,
        .vt = .{
            .transmit_fn = radio.Radio.VTable.failingTransmit,
            .receive_fn = radio.Radio.VTable.failingReceive,
            .get_received_fn = radio.Radio.VTable.failingGetReceived,
        },
        .impl = undefined,
        .packet_len_min = 32,
        .packet_len_max = 32,
    };
    defer radio_iface.deinit();
    // try radio_iface.transmit("Hello world!");
    try radio_iface.sendMessage(try init.gpa.dupe(u8, "Hello world!"));
    _ = try radio_iface.update(init.io);
}
