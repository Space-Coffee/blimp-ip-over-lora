const std = @import("std");
const c = @import("c");
const misc = @import("misc.zig");

const gpio_consumer_name: []const u8 = "blimp-ip-over-lora";

pub const GpioCtrl = struct {
    dev_file: std.Io.File,
    out_lines_buf: [64]u32,
    out_lines: []u32,
    out_lines_fd: i32,
    in_lines_buf: [64]u32,
    in_lines: []u32,
    in_lines_fd: i32,

    pub fn init(io: std.Io, dev_path: []const u8, out_lines: []const u32, in_lines: []const u32) !GpioCtrl {
        if (out_lines.len > 64 or in_lines.len > 64) {
            return error.LengthTooLarge;
        }

        const dev_file = try std.Io.Dir.openFileAbsolute(io, dev_path, .{ .mode = .read_write });
        const fd = dev_file.handle;

        var self = GpioCtrl{
            .dev_file = dev_file,
            .out_lines_buf = std.mem.zeroes([64]u32),
            .out_lines = undefined,
            .out_lines_fd = undefined,
            .in_lines_buf = std.mem.zeroes([64]u32),
            .in_lines = undefined,
            .in_lines_fd = undefined,
        };
        @memcpy(self.out_lines[0..out_lines.len], out_lines);
        @memcpy(self.in_lines[0..in_lines.len], in_lines);

        if (out_lines.len > 0) {
            var out_req = std.mem.zeroes(c.gpio_v2_line_request);
            out_req.config.flags = c.GPIO_V2_LINE_FLAG_OUTPUT;
            out_req.config.num_attrs = 0; // do we need attributes?
            @memcpy(out_req.offsets[0..out_lines.len], out_lines);
            out_req.num_lines = @intCast(out_lines.len);
            @memcpy(out_req.consumer[0..gpio_consumer_name.len], gpio_consumer_name);

            _ = try misc.ioctl_checked(
                fd,
                c.GPIO_V2_GET_LINE_IOCTL,
                @intFromPtr(&out_req),
                "couldn't open GPIO output",
            );
            if (out_req.fd < 0) {
                std.log.err(
                    "Failed to open gpio device {s} output lines with errno = {d}",
                    .{ dev_path, std.c._errno().* },
                );
                return error.GpioLineOpenFailed;
            }
            self.out_lines_fd = out_req.fd;
        } else {
            self.out_lines_fd = -1;
        }

        if (in_lines.len > 0) {
            var in_req = std.mem.zeroes(c.gpio_v2_line_request);
            in_req.config.flags = c.GPIO_V2_LINE_FLAG_INPUT;
            in_req.config.num_attrs = 0; // do we need attributes?
            @memcpy(in_req.offsets[0..in_lines.len], in_lines);
            in_req.num_lines = @intCast(in_lines.len);
            @memcpy(in_req.consumer[0..gpio_consumer_name.len], gpio_consumer_name);

            _ = try misc.ioctl_checked(
                fd,
                c.GPIO_V2_GET_LINE_IOCTL,
                @intFromPtr(&in_req),
                "couldn't open GPIO input",
            );
            if (in_req.fd < 0) {
                std.log.err(
                    "Failed to open gpio device {s} input lines with errno = {d}",
                    .{ dev_path, std.c._errno().* },
                );
                return error.GpioLineOpenFailed;
            }
            self.in_lines_fd = in_req.fd;
        } else {
            self.in_lines_fd = -1;
        }

        return self;
    }

    pub fn deinit(self: *const GpioCtrl, io: std.Io) void {
        if (self.out_lines_fd >= 0) {
            _ = std.c.close(self.out_lines_fd);
        }
        if (self.in_lines_fd >= 0) {
            _ = std.c.close(self.in_lines_fd);
        }
        self.dev_file.close(io);
    }

    pub fn set(self: *GpioCtrl, val: u64, mask: u64) !void {
        var vals = std.mem.zeroes(c.gpio_v2_line_value);
        vals.bits = val;
        vals.mask = mask;
        _ = try misc.ioctl_checked(
            self.dev_file.handle,
            c.GPIO_V2_LINE_SET_VALUES_IOCTL,
            @intFromPtr(&vals),
            "couldn't set GPIO values",
        );
    }

    pub fn get(self: *GpioCtrl, mask: u64) !u64 {
        var vals = std.mem.zeroes(c.gpio_v2_line_value);
        vals.mask = mask;
        _ = try misc.ioctl_checked(
            self.dev_file.handle,
            c.GPIO_V2_LINE_GET_VALUES_IOCTL,
            @intFromPtr(&vals),
            "couldn't get GPIO values",
        );
        return vals.bits;
    }
};
