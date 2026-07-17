const std = @import("std");
const c = @import("c");

pub fn ioctl_checked(fd: std.os.linux.fd_t, request: u32, arg: usize, err_msg: []const u8) !usize {
    const result = std.os.linux.ioctl(fd, request, arg);
    if (result < 0) {
        std.log.info("{s}\nerrno = {d}", .{ err_msg, result });
        return error.IoctlFailed;
    } else {
        return result;
    }
}
