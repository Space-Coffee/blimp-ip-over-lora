const std = @import("std");

const spi = @import("spi.zig");

pub fn main(init: std.process.Init) !void {
    _ = init;

    const spi_dev_path = "/dev/spidev0.0";
    const spi_dev = try spi.Spi.init(spi_dev_path);
    _ = spi_dev;
}
