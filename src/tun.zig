const std = @import("std");
const c = @import("c");
const misc = @import("misc.zig");

pub const Tun = struct {
    tun_dev_file: std.Io.File,
    assigned_name: []const u8,

    pub fn init(io: std.Io, gpa: std.mem.Allocator, tun_name: []const u8) !Tun {
        const tun_dev_file = try std.Io.Dir.openFileAbsolute(
            io,
            "/dev/net/tun",
            .{ .mode = .read_write },
        );

        var ifr = std.mem.zeroes(c.ifreq);
        if (tun_name.len > c.IFNAMSIZ) {
            return error.LengthTooLarge;
        }
        @memcpy(ifr.ifr_ifrn.ifrn_name[0..tun_name.len], tun_name);
        ifr.ifr_ifru.ifru_flags = c.IFF_TUN;
        _ = try misc.ioctl_checked(
            tun_dev_file.handle,
            c.TUNSETIFF,
            @intFromPtr(&ifr),
            "couldn't create tun device",
        );

        const name_has_null = name_has_null_blk: {
            for (ifr.ifr_ifrn.ifrn_name[0..c.IFNAMSIZ]) |ch| {
                if (ch == 0) {
                    break :name_has_null_blk true;
                }
            }
            break :name_has_null_blk false;
        };
        var assigned_name: []const u8 = undefined;
        errdefer gpa.free(assigned_name);
        if (name_has_null) {
            assigned_name = try gpa.dupe(u8, ifr.ifr_ifrn.ifrn_name[0..std.mem.findSentinel(u8, 0, @ptrCast(&ifr.ifr_ifrn.ifrn_name))]);
        } else {
            assigned_name = try gpa.dupe(u8, &ifr.ifr_ifrn.ifrn_name);
        }

        // Open a dummy socket
        const socket = try (std.Io.net.IpAddress{
            .ip4 = .loopback(1234),
        }).bind(
            io,
            .{ .mode = .dgram, .protocol = .udp },
        );

        // Address
        ifr = std.mem.zeroes(c.ifreq);
        @memcpy(ifr.ifr_ifrn.ifrn_name[0..assigned_name.len], assigned_name);
        var addr: *c.sockaddr_in = @ptrCast(&ifr.ifr_ifru.ifru_addr);
        addr.sin_family = c.AF_INET;
        if (c.inet_pton(c.AF_INET, "10.12.34.1", &addr.sin_addr) != 1) {
            return error.InvalidAddress;
        }
        _ = try misc.ioctl_checked(
            socket.handle,
            c.SIOCSIFADDR,
            @intFromPtr(&ifr),
            "couldn't set TUN address",
        );

        // Netmask
        ifr = std.mem.zeroes(c.ifreq);
        @memcpy(ifr.ifr_ifrn.ifrn_name[0..assigned_name.len], assigned_name);
        addr = @ptrCast(&ifr.ifr_ifru.ifru_netmask);
        addr.sin_family = c.AF_INET;
        if (c.inet_pton(c.AF_INET, "255.255.255.0", &addr.sin_addr) != 1) {
            return error.InvalidAddress;
        }
        _ = try misc.ioctl_checked(
            socket.handle,
            c.SIOCSIFNETMASK,
            @intFromPtr(&ifr),
            "couldn't set TUN netmask",
        );

        // Netmask
        ifr = std.mem.zeroes(c.ifreq);
        @memcpy(ifr.ifr_ifrn.ifrn_name[0..assigned_name.len], assigned_name);
        _ = try misc.ioctl_checked(
            socket.handle,
            c.SIOCGIFFLAGS,
            @intFromPtr(&ifr),
            "couldn't get TUN flags",
        );
        // Apparently some of those flags don't fit in 16 bits, hence @truncate.
        // I'm not sure what if their use then.
        ifr.ifr_ifru.ifru_flags |= @as(
            c_short,
            @truncate(c.IFF_UP | c.IFF_LOWER_UP | c.IFF_NOARP | c.IFF_MULTICAST | c.IFF_POINTOPOINT | c.IFF_RUNNING),
        );
        _ = try misc.ioctl_checked(
            socket.handle,
            c.SIOCSIFFLAGS,
            @intFromPtr(&ifr),
            "couldn't set TUN flags",
        );

        socket.close(io);

        return .{
            .tun_dev_file = tun_dev_file,
            .assigned_name = assigned_name,
        };
    }

    pub fn deinit(self: *const Tun, io: std.Io, gpa: std.mem.Allocator) void {
        gpa.free(self.assigned_name);
        self.tun_dev_file.close(io);
    }
};
