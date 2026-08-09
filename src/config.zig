const std = @import("std");
const nrf905 = @import("nrf905.zig");

pub const Nrf905Config = struct {
    settings: nrf905.Nrf905.Settings,
    peer_addr: u32,
};

pub const ConfigRoot = struct {
    spi_dev_path: [:0]const u8,
    nrf905: ?Nrf905Config = null,
    radio: struct {
        impl: union(enum) {
            failing: void,
            dummy: void,
            nrf905: void,
        },
        packet_len_min: u32,
        packet_len_max: u32,
    },
};
