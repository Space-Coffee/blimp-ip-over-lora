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
    payload_len_min: u32,
    payload_len_max: u32,
    egress_queue: std.Deque([]const u8) = .empty,
    link_state: LinkState = .unknown,
    next_egress_packet_id: u8 = 0,
    expected_ingress_packet_id: ?u8 = null,
    ingress_payload_buf: std.ArrayList(u8) = .empty,

    const turn_duration_ms: i64 = 500;

    pub const VTable = struct {
        transmit_fn: *const fn (self: *anyopaque, data: []const u8) error{TransmitError}!void,
        receive_fn: *const fn (self: *anyopaque) error{ReceiveError}!void,
        get_received_fn: *const fn (self: *anyopaque) error{ReceiveError}!?[]const u8,

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
    };

    const LinkState = union(enum) {
        unknown: void,
        their_turn: struct { until: std.Io.Timestamp },
        our_turn: struct { until: std.Io.Timestamp },
    };

    fn transmit(self: *Radio, data: []const u8) !void {
        if (data.len < self.payload_len_min or data.len > self.payload_len_max) {
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

    fn createOurTurn(io: std.Io) LinkState {
        return .{
            .our_turn = .{
                .until = std.Io.Timestamp.now(
                    io,
                    .real,
                ).addDuration(
                    .fromMilliseconds(turn_duration_ms),
                ),
            },
        };
    }

    pub fn sent_packet(self: *Radio, data: []const u8) !void {
        try self.egress_queue.pushBack(self.gpa, data);
    }

    pub fn chunk_and_send(self: *Radio, data: []const u8) !void {
        var remaining = data;
        while (remaining.len > 0) {
            var curr_chunk: []const u8 = undefined;
            if (remaining.len < self.payload_len_max - 4) {
                curr_chunk = remaining;
            } else {
                curr_chunk = remaining[0..(self.payload_len_max - 4)];
                remaining = remaining[(self.payload_len_max - 4)..];
            }

            var msg_buf = try self.gpa.alloc(u8, self.payload_len_max + 4);
            defer self.gpa.free(msg_buf);
            msg_buf[0] = self.next_egress_packet_id;
            // msg_buf[1]=
        }
        self.next_egress_packet_id += 1;
    }

    pub fn update(self: *Radio, io: std.Io) !void {
        const now = std.Io.Timestamp.now(io, .real);
        switch (self.link_state) {
            .unknown => {
                const first = self.egress_queue();
                if (first) |first_nn| {
                    self.link_state = createOurTurn(io);
                    try self.chunk_and_send(first_nn);
                } else {
                    try self.receive();
                }
            },
            .our_turn => |our_turn| {
                if (now.durationTo(our_turn.until).nanoseconds > 0) {
                    // Still our turn

                }
            },
            .their_turn => |their_turn| {
                if (now.durationTo(their_turn.until).nanoseconds > 0) {
                    //Still their turn
                    try self.receive();
                } else {
                    if (self.egress_queue.len > 0) {
                        self.link_state = createOurTurn(io);
                    } else {}
                }
            },
        }
    }
};
