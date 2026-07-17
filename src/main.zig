const std = @import("std");
const spi = @import("spi.zig");
const gpio = @import("gpio.zig");

pub fn main(init: std.process.Init) !void {
    const spi_dev_path: [:0]const u8 = "/dev/spidev0.0";
    var spi_dev = try spi.Spi.init(init.io, spi_dev_path);
    defer spi_dev.deinit(init.io);
    try spi_dev.write_reg8(0b10110001, 0x00);

    const gpio_dev_path: [:0]const u8 = "/dev/gpiochip0";
    const gpio_ctrl = try gpio.GpioCtrl.init(init.io, gpio_dev_path, &.{}, &.{});
    _ = gpio_ctrl;
}
