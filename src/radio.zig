const std = @import("std");

pub const Radio = struct {
    vt: VTable,
    impl: *anyopaque,
    payload_len_min: u32,
    payload_len_max: u32,

    pub const VTable = struct {
        transmit_fn: *const fn (self: *anyopaque, data: []const u8) error{TransmitError}!void,
        receive_fn: *const fn (self: *anyopaque) error{ReceiveError}!void,
        get_received_fn: *const fn (self: *anyopaque) error{ReceiveError}!?[]const u8,

        pub fn failing_transmit(self: *anyopaque, data: []const u8) error{TransmitError}!void {
            _ = .{ self, data };
            return error.TransmitError;
        }
        pub fn failing_receive(self: *anyopaque) error{ReceiveError}!void {
            _ = self;
            return error.ReceiveError;
        }
        pub fn failing_get_received(self: *anyopaque) error{ReceiveError}!?[]const u8 {
            _ = self;
            return error.ReceiveError;
        }
    };

    pub fn transmit(self: *Radio, data: []const u8) !void {
        if (data.len < self.payload_len_min or data.len > self.payload_len_max) {
            return error.IllegalLength;
        }
        try self.vt.transmit_fn(self.impl, data);
    }

    pub fn receive(self: *Radio) !void {
        try self.vt.receive_fn(self.impl);
    }

    pub fn get_received(self: *Radio) !?[]const u8 {
        return try self.vt.get_received_fn(self.impl);
    }
};
