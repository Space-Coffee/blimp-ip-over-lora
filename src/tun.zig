const std = @import("std");
const c = @import("c");
const misc = @import("misc.zig");

pub const Tun = struct {
    tun_dev_file: std.Io.File,
    assigned_name: []const u8,

    const Logger = std.log.scoped(.tun);

    pub fn init(
        io: std.Io,
        gpa: std.mem.Allocator,
        tun_name: []const u8,
        local_addr: [:0]const u8,
        netmask: [:0]const u8,
        mtu: u32,
    ) !Tun {
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
        if (c.inet_pton(c.AF_INET, local_addr, &addr.sin_addr) != 1) {
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
        if (c.inet_pton(c.AF_INET, netmask, &addr.sin_addr) != 1) {
            return error.InvalidAddress;
        }
        _ = try misc.ioctl_checked(
            socket.handle,
            c.SIOCSIFNETMASK,
            @intFromPtr(&ifr),
            "couldn't set TUN netmask",
        );

        // Flags
        ifr = std.mem.zeroes(c.ifreq);
        @memcpy(ifr.ifr_ifrn.ifrn_name[0..assigned_name.len], assigned_name);
        _ = try misc.ioctl_checked(
            socket.handle,
            c.SIOCGIFFLAGS,
            @intFromPtr(&ifr),
            "couldn't get TUN flags",
        );
        // Apparently some of those flags don't fit in 16 bits, hence @truncate.
        // I'm not sure what is their use then.
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

        // MTU
        ifr = std.mem.zeroes(c.ifreq);
        ifr.ifr_ifru.ifru_mtu = @intCast(mtu);
        _ = try misc.ioctl_checked(
            socket.handle,
            c.SIOCSIFMTU,
            @intFromPtr(&ifr),
            "couldn't set TUN MTU",
        );

        socket.close(io);

        Logger.info("Tun ready (name \"{s}\")", .{assigned_name});

        return .{
            .tun_dev_file = tun_dev_file,
            .assigned_name = assigned_name,
        };
    }

    pub fn deinit(self: *const Tun, io: std.Io, gpa: std.mem.Allocator) void {
        gpa.free(self.assigned_name);
        self.tun_dev_file.close(io);
    }

    pub fn worker(
        self: *Tun,
        io: std.Io,
        gpa: std.mem.Allocator,
        tun2radio_queue: *std.Io.Queue([]const u8),
        radio2tun_queue: *std.Io.Queue([]const u8),
    ) !void {
        var reader_buf: [4096]u8 = undefined;
        // const reader_buf_vec = [_][]u8{&reader_buf};
        // var tun_reader = self.tun_dev_file.readerStreaming(io, &reader_buf);

        const SelectU = union(enum) {
            // tun_read: std.Io.File.ReadStreamingError!usize,
            tun_read: std.Io.File.Reader.Error!usize,
            radio_queue_read: error{ Canceled, Closed }![]const u8,
        };
        var select_buf: [2]SelectU = undefined;
        var select = std.Io.Select(SelectU).init(io, &select_buf);

        // try select.concurrent(
        //     .tun_read,
        //     std.Io.File.readStreaming,
        //     .{ self.tun_dev_file, io, &reader_buf_vec },
        // );
        try select.concurrent(
            .tun_read,
            std.posix.read,
            .{ self.tun_dev_file.handle, &reader_buf },
        );
        try select.concurrent(
            .radio_queue_read,
            std.Io.Queue([]const u8).getOne,
            .{ radio2tun_queue, io },
        );

        while (true) {
            const select_result = try select.await();
            switch (select_result) {
                .tun_read => |tun_read| {
                    const bytes_read = try tun_read;
                    // Logger.debug("Read {d} bytes through tun", .{bytes_read});
                    const msg_copy = try gpa.dupe(u8, reader_buf[0..bytes_read]);
                    try tun2radio_queue.putOne(io, msg_copy);

                    // try select.concurrent(
                    //     .tun_read,
                    //     std.Io.File.readStreaming,
                    //     .{ self.tun_dev_file, io, &reader_buf_vec },
                    // );
                    try select.concurrent(
                        .tun_read,
                        std.posix.read,
                        .{ self.tun_dev_file.handle, &reader_buf },
                    );
                },
                .radio_queue_read => |radio_queue_read| {
                    const radio_msg = try radio_queue_read;
                    defer gpa.free(radio_msg);
                    // To ensure it's done in exactly one syscall
                    _ = std.c.write(
                        self.tun_dev_file.handle,
                        @ptrCast(radio_msg),
                        radio_msg.len,
                    );

                    try select.concurrent(
                        .radio_queue_read,
                        std.Io.Queue([]const u8).getOne,
                        .{ radio2tun_queue, io },
                    );
                },
            }
        }
    }
};
