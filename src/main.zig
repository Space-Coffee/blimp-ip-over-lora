const std = @import("std");
const spi = @import("spi.zig");
const gpio = @import("gpio.zig");

pub fn main(init: std.process.Init) !void {
    const spi_dev_path: [:0]const u8 = "/dev/spidev0.0";
    const spi_dev = try spi.Spi.init(init.io, spi_dev_path);
    _ = spi_dev;

    const gpio_dev_path: [:0]const u8 = "/dev/gpiochip0";
    const gpio_ctrl = try gpio.GpioCtrl.init(init.io, gpio_dev_path);
    _ = gpio_ctrl;
}
