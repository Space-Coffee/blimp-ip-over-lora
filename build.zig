const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optim = b.standardOptimizeOption(.{});

    const mod = b.addModule("main", .{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optim,
        .link_libc = true,
    });
    const exe = b.addExecutable(.{ .name = "blimp-ip-over-lora", .root_module = mod });

    b.installArtifact(exe);
}
