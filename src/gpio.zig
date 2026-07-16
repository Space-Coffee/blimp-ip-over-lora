const std = @import("std");
const c = @import("c");

const spi = @import("spi.zig");
const gpio_consumer_name = "blimp-ip-over-lora";

const GpioCtrl = struct {
    fd: i32,

    fn init(dev_path: [:0]u8) !@This() {
        const fd: i32 = std.os.linux.open(dev_path);
        if (fd < 0) {
            std.log.err("couldn't open GPIO device {s}, errno = {d}", .{
                dev_path,
                std.posix.errno,
            });
        }
    }
};
