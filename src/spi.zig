const std = @import("std");
const c = @import("c");
const misc = @import("misc.zig");

pub const Spi = struct {
    dev_file: std.Io.File,

    pub fn init(io: std.Io, dev_path: []const u8) !@This() {
        // const fd: i32 = @intCast(std.os.linux.open(dev_path, .{}, c.O_RDWR));
        const dev_file = try std.Io.Dir.openFileAbsolute(io, dev_path, .{ .mode = .read_write });
        const fd = dev_file.handle;

        var buf8: u8 = 0; // CPOL=0, CPHA=0
        _ = try misc.ioctl_checked(
            fd,
            c.SPI_IOC_WR_MODE,
            @intFromPtr(&buf8),
            "couldn't set SPI mode",
        );
        buf8 = 0; // MSF first
        _ = try misc.ioctl_checked(
            fd,
            c.SPI_IOC_WR_LSB_FIRST,
            @intFromPtr(&buf8),
            "couldn't set SPI MSB/LSB first",
        );
        buf8 = 8;
        _ = try misc.ioctl_checked(
            fd,
            c.SPI_IOC_WR_BITS_PER_WORD,
            @intFromPtr(&buf8),
            "couldn't set bit count per word",
        );
        var buf32: u32 = 100000;
        _ = try misc.ioctl_checked(
            fd,
            c.SPI_IOC_WR_MAX_SPEED_HZ,
            @intFromPtr(&buf32),
            "couldn't set max speed",
        );

        return .{
            .dev_file = dev_file,
        };
    }
};
