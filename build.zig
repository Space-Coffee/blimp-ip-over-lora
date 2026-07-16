const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optim = b.standardOptimizeOption(.{});

    const c_trans = b.addTranslateC(.{
        .target = target,
        .optimize = optim,
        .root_source_file = b.path("src/c.c"),
    });
    const c_mod = c_trans.createModule();
    const mod = b.addModule("main", .{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optim,
        .link_libc = true,
    });
    mod.addImport("c", c_mod);
    const exe = b.addExecutable(.{ .name = "blimp-ip-over-lora", .root_module = mod });

    b.installArtifact(exe);
}
