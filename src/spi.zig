const std = @import("std");
const c = @import("c");
const misc = @import("misc.zig");

pub const Spi = struct {
    dev_file: std.Io.File,

    const Logger = std.log.scoped(.spi);

    pub fn init(io: std.Io, dev_path: []const u8) !Spi {
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
        buf8 = 0; // MSB first
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
        // var buf32: u32 = 1000;
        _ = try misc.ioctl_checked(
            fd,
            c.SPI_IOC_WR_MAX_SPEED_HZ,
            @intFromPtr(&buf32),
            "couldn't set max speed",
        );

        Logger.info("SPI ready", .{});

        return .{
            .dev_file = dev_file,
        };
    }

    pub fn deinit(self: *const Spi, io: std.Io) void {
        self.dev_file.close(io);
    }

    pub fn write_reg8(self: *Spi, addr: u8, val: u8) !void {
        const fd = self.dev_file.handle;
        var xfer = std.mem.zeroes(c.spi_ioc_transfer);
        var buf = [2]u8{ addr, val };
        buf[0] |= 0x80; // Indicate write
        xfer.tx_buf = @intFromPtr(&buf);
        xfer.rx_buf = 0;
        xfer.len = 2;

        _ = try misc.ioctl_checked(
            fd,
            c.SPI_IOC_MESSAGE_1,
            @intFromPtr(&xfer),
            "couldn't perform SPI register write",
        );
    }

    pub fn write_bulk(self: *Spi, addr: u8, data: []const u8, gpa: std.mem.Allocator) !void {
        const fd = self.dev_file.handle;
        var xfer = std.mem.zeroes(c.spi_ioc_transfer);
        const buf = try gpa.alloc(u8, data.len + 1);
        defer gpa.free(buf);
        buf[0] = addr | 0x80; // Indicate write
        @memcpy(buf[1..], data);
        xfer.tx_buf = @intFromPtr(buf.ptr);
        xfer.rx_buf = 0;
        xfer.len = data.len + 1;

        _ = try misc.ioctl_checked(
            fd,
            c.SPI_IOC_MESSAGE_1,
            @intFromPtr(&xfer),
            "couldn't perform SPI register write",
        );
    }

    pub fn read_reg8(self: *Spi, addr: u8) !u8 {
        const fd = self.dev_file.handle;
        var xfer = std.mem.zeroes([2]c.spi_ioc_trasnfer);
        var val: u8 = undefined;
        xfer[0].tx_buf = @intFromPtr(&addr);
        xfer[0].rx_buf = 0;
        xfer[0].len = 1;
        xfer[1].tx_buf = 0;
        xfer[1].rx_buf = @intFromPtr(&val);
        xfer[1].len = 1;

        _ = try misc.ioctl_checked(
            fd,
            c.SPI_IOC_MESSAGE_2,
            @intFromPtr(&xfer),
            "coulnd't perform SPI register read",
        );
        return val;
    }

    pub fn read_bulk(self: *Spi, addr: u8, result: []u8) !void {
        const fd = self.dev_file.handle;
        var xfer = std.mem.zeroes([2]c.spi_ioc_trasnfer);
        xfer[0].tx_buf = @intFromPtr(&addr);
        xfer[0].rx_buf = 0;
        xfer[0].len = 1;
        xfer[1].tx_buf = 0;
        xfer[1].rx_buf = @intFromPtr(result.ptr);
        xfer[1].len = result.len;

        _ = try misc.ioctl_checked(
            fd,
            c.SPI_IOC_MESSAGE_2,
            @intFromPtr(&xfer),
            "coulnd't perform SPI register read",
        );
    }

    pub fn transact(self: *Spi, data_tx: []const u8, data_rx: []u8) !void {
        if (data_tx.len != data_rx.len) {
            return error.LengthMismatch;
        }

        const fd = self.dev_file.handle;
        var xfer = std.mem.zeroes(c.spi_ioc_transfer);
        xfer.tx_buf = @intFromPtr(data_tx.ptr);
        xfer.rx_buf = @intFromPtr(data_rx.ptr);
        xfer.len = @intCast(data_tx.len);

        _ = try misc.ioctl_checked(
            fd,
            c.SPI_IOC_MESSAGE_1,
            @intFromPtr(&xfer),
            "coulnd't perform SPI transaction",
        );

        Logger.debug("SPI transaction: tx={any}, rx={any}", .{
            data_tx,
            data_rx,
        });
    }
};
