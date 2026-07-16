const std = @import("std");
const c = @cImport({
    @cInclude("fcntl.h");
    @cInclude("linux/spi/spi.h");
    @cInclude("linux/spi/spidev.h");
});

fn ioctl_checked(fd: std.os.linux.fd_t, request: u32, arg: usize, err_msg: []const u8) !usize {
    const result = std.os.linux.ioctl(fd, request, arg);
    if (result < 0) {
        std.log.info("{s}\nerrno = {d}", .{ err_msg, result });
        return error.IoctlFailed;
    } else {
        return result;
    }
}

pub const Spi = struct {
    fd: i32,

    pub fn init(dev_path: [:0]const u8) !@This() {
        const fd: i32 = @intCast(std.os.linux.open(dev_path, .{}, c.O_RDWR));
        if (fd < 0) {
            return error.SpiOpenError;
        }

        var buf8: u8 = 0; // CPOL=0, CPHA=0
        _ = try ioctl_checked(
            fd,
            c.SPI_IOC_WR_MODE,
            @intFromPtr(&buf8),
            "couldn't set SPI mode",
        );
        buf8 = 0; // MSF first
        _ = try ioctl_checked(
            fd,
            c.SPI_IOC_WR_LSB_FIRST,
            @intFromPtr(&buf8),
            "couldn't set SPI MSB/LSB first",
        );
        buf8 = 8;
        _ = try ioctl_checked(
            fd,
            c.SPI_IOC_WR_BITS_PER_WORD,
            @intFromPtr(&buf8),
            "couldn't set bit count per word",
        );
        var buf32: u32 = 100000;
        _ = try ioctl_checked(
            fd,
            c.SPI_IOC_WR_MAX_SPEED_HZ,
            @intFromPtr(&buf32),
            "couldn't set max speed",
        );

        return .{ .fd = fd };
    }
};
