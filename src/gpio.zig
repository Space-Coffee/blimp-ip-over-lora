const std = @import("std");
const c = @import("c");
const misc = @import("misc.zig");

const gpio_consumer_name = "blimp-ip-over-lora";

pub const GpioCtrl = struct {
    dev_file: std.Io.File,

    pub fn init(io: std.Io, dev_path: []const u8) !@This() {
        // const fd: i32 = @intCast(std.os.linux.open(dev_path, .{}, c.O_RDWR));
        const dev_file = try std.Io.Dir.openFileAbsolute(io, dev_path, .{ .mode = .read_write });
        const fd = dev_file.handle;
        _ = fd;

        return .{
            .dev_file = dev_file,
        };
    }
};
