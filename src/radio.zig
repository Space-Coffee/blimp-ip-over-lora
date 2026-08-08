const std = @import("std");

// Packet structure:
// [0] packet_id
// [1] packet_chunks_count
// [2] chunk_num
// [3] chunk_len
// [4..] payload

pub const Radio = struct {
    gpa: std.mem.Allocator,
    vt: VTable,
    impl: *anyopaque,
    packet_len_min: u32,
    packet_len_max: u32,
    egress_queue: std.Deque([]const u8) = .empty,
    link_state: LinkState = .unknown,
    next_egress_packet_id: u8 = 0,
    expected_ingress_packet_id: ?u8 = null,
    // expected_ingress_packet_chunks_count: ?u8 = null,
    expected_ingress_chunk_num: ?u8 = null,
    ingress_payload_buf: std.ArrayList(u8) = .empty,

    const turn_duration_ms: i64 = 500;

    pub const VTable = struct {
        transmit_fn: *const fn (self: *anyopaque, data: []const u8) error{TransmitError}!void,
        receive_fn: *const fn (self: *anyopaque) error{ReceiveError}!void,
        get_received_fn: *const fn (self: *anyopaque) error{ReceiveError}!?[]const u8,

        pub const failing = VTable{
            .transmit_fn = failingTransmit,
            .receive_fn = failingReceive,
            .get_received_fn = failingGetReceived,
        };

        pub const dummy = VTable{
            .transmit_fn = dummyTransmit,
            .receive_fn = dummyReceive,
            .get_received_fn = dummyGetReceived,
        };

        pub fn failingTransmit(self: *anyopaque, data: []const u8) error{TransmitError}!void {
            _ = .{ self, data };
            return error.TransmitError;
        }
        pub fn failingReceive(self: *anyopaque) error{ReceiveError}!void {
            _ = self;
            return error.ReceiveError;
        }
        pub fn failingGetReceived(self: *anyopaque) error{ReceiveError}!?[]const u8 {
            _ = self;
            return error.ReceiveError;
        }

        pub fn dummyTransmit(self: *anyopaque, data: []const u8) error{TransmitError}!void {
            _ = .{ self, data };
        }
        pub fn dummyReceive(self: *anyopaque) error{ReceiveError}!void {
            _ = self;
        }
        pub fn dummyGetReceived(self: *anyopaque) error{ReceiveError}!?[]const u8 {
            _ = self;
            return null;
        }
    };

    const LinkState = union(enum) {
        unknown: void,
        their_turn: struct { until: std.Io.Timestamp },
        our_turn: struct { until: std.Io.Timestamp },
    };

    const Logger = std.log.scoped(.radio_link);

    fn transmit(self: *Radio, data: []const u8) !void {
        if (data.len < self.packet_len_min or data.len > self.packet_len_max) {
            return error.IllegalLength;
        }
        try self.vt.transmit_fn(self.impl, data);
    }

    fn receive(self: *Radio) !void {
        try self.vt.receive_fn(self.impl);
    }

    fn getReceived(self: *Radio) !?[]const u8 {
        return try self.vt.get_received_fn(self.impl);
    }

    fn createTurn(io: std.Io, our: bool) LinkState {
        const until = std.Io.Timestamp.now(
            io,
            .real,
        ).addDuration(
            .fromMilliseconds(turn_duration_ms),
        );
        if (our) {
            return .{
                .our_turn = .{
                    .until = until,
                },
            };
        } else {
            return .{
                .their_turn = .{
                    .until = until,
                },
            };
        }
    }

    fn chunkAndSend(self: *Radio, data: []const u8) !void {
        var remaining = data;
        var chunks = std.ArrayList([]const u8).empty;
        defer chunks.deinit(self.gpa);
        while (remaining.len > 0) {
            var curr_chunk: []const u8 = undefined;
            var should_break: bool = false;
            if (remaining.len <= self.packet_len_max - 4) {
                curr_chunk = remaining;
                should_break = true;
            } else {
                curr_chunk = remaining[0..(self.packet_len_max - 4)];
                remaining = remaining[(self.packet_len_max - 4)..];
            }

            try chunks.append(self.gpa, curr_chunk);
            if (should_break) {
                break;
            }
        }

        if (chunks.items.len > 255) {
            return error.MessageTooLarge;
        }

        var msg_buf = try self.gpa.alloc(u8, self.packet_len_max);
        defer self.gpa.free(msg_buf);
        for (chunks.items, 0..) |curr_chunk, i| {
            msg_buf[0] = self.next_egress_packet_id;
            msg_buf[1] = @intCast(chunks.items.len);
            msg_buf[2] = @intCast(i);
            msg_buf[3] = @intCast(curr_chunk.len);
            @memcpy(msg_buf[4..(curr_chunk.len + 4)], curr_chunk);
            var padding: u32 = 0;
            if (curr_chunk.len + 4 < self.packet_len_min) {
                padding = @intCast(self.packet_len_min - (curr_chunk.len + 4));
                @memset(msg_buf[(curr_chunk.len + 4)..self.packet_len_min], 0);
            }

            Logger.debug("Sending message chunk, #{d} out of {d}, length {d} (total {d}), padding {d}", .{
                i,
                chunks.items.len,
                curr_chunk.len,
                data.len,
                padding,
            });

            try self.transmit(msg_buf[0..(@max(curr_chunk.len + 4, self.packet_len_min))]);
        }

        self.next_egress_packet_id += 1;
    }

    pub fn deinit(self: *Radio) void {
        var egress_queue_iter = self.egress_queue.iterator();
        while (egress_queue_iter.next()) |msg| {
            self.gpa.free(msg);
        }
        self.egress_queue.deinit(self.gpa);
        self.ingress_payload_buf.deinit(self.gpa);
    }

    pub fn sendMessage(self: *Radio, data: []const u8) !void {
        try self.egress_queue.pushBack(self.gpa, data);
    }

    pub fn update(self: *Radio, io: std.Io) !?[]const u8 {
        const recv_packet = try self.getReceived();
        var recv_msg: ?[]const u8 = null;
        if (recv_packet) |recv_packet_nn| {
            const packet_id = recv_packet_nn[0];
            const packet_chunks_count = recv_packet_nn[1];
            const chunk_num = recv_packet_nn[2];
            const chunk_len = recv_packet_nn[3];
            const payload = recv_packet_nn[4..(4 + chunk_len)];

            Logger.debug(
                "Received a packet: id {d}, chunks count {d}, chunk #{d}, chunk len {d}",
                .{ packet_id, packet_chunks_count, chunk_num, chunk_len },
            );

            const msg_continued = msg_cont_blk: {
                if (self.expected_ingress_packet_id) |eipi_nn| {
                    if (eipi_nn == packet_id) {
                        break :msg_cont_blk true;
                    } else {
                        break :msg_cont_blk false;
                    }
                } else {
                    break :msg_cont_blk false;
                }
            };
            if (!msg_continued) {
                self.ingress_payload_buf.clearRetainingCapacity();
                self.expected_ingress_packet_id = packet_id;
                // self.expected_ingress_packet_chunks_count = packet_chunks_count;
                self.expected_ingress_chunk_num = 0;
            }

            if (self.expected_ingress_chunk_num == @as(?u8, chunk_num)) {
                try self.ingress_payload_buf.appendSlice(self.gpa, payload);

                if (chunk_num == packet_chunks_count - 1) {
                    recv_msg = try self.ingress_payload_buf.toOwnedSlice(self.gpa);
                    self.expected_ingress_packet_id = null;
                } else {
                    self.expected_ingress_chunk_num = chunk_num + 1;
                }
            } else {
                self.expected_ingress_packet_id = null;
            }
        }

        const now = std.Io.Timestamp.now(io, .real);
        switch (self.link_state) {
            .unknown => {
                if (recv_msg) |_| {
                    Logger.debug("They interrupted the silence", .{});
                    self.link_state = createTurn(io, false);
                    try self.receive();
                } else {
                    const first = self.egress_queue.popFront();
                    if (first) |first_nn| {
                        defer self.gpa.free(first_nn);
                        Logger.debug("We're interrupting the silence", .{});
                        self.link_state = createTurn(io, true);
                        try self.chunkAndSend(first_nn);
                    } else {
                        try self.receive();
                    }
                }
            },
            .our_turn => |our_turn| {
                if (now.durationTo(our_turn.until).nanoseconds > 0) {
                    // Still our turn
                    const first = self.egress_queue.popFront();
                    if (first) |first_nn| {
                        defer self.gpa.free(first_nn);
                        try self.chunkAndSend(first_nn);
                    }
                } else {
                    Logger.debug("We're giving the turn back to them", .{});
                    self.link_state = createTurn(io, false);
                }
            },
            .their_turn => |their_turn| {
                if (now.durationTo(their_turn.until).nanoseconds > 0) {
                    //Still their turn
                    try self.receive();
                } else {
                    if (self.egress_queue.len > 0) {
                        Logger.debug("We're getting the turn back", .{});
                        self.link_state = createTurn(io, true);
                    } else {
                        Logger.debug("Back to silence", .{});
                        self.link_state = .unknown;
                    }
                }
            },
        }

        return recv_msg;
    }
};
