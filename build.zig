const std = @import("std");

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{ .preferred_optimize_mode = .ReleaseSafe });
    const target = b.standardTargetOptions(.{});

    const logc = b.dependency("logc", .{});

    const lib = b.addLibrary(.{
        .name = "disciplining_minipod",
        .root_module = b.createModule(.{ .target = target, .optimize = optimize }),
    });
    lib.addCSourceFiles(.{
        .root = b.path("src"),
        .files = &.{
            "checks.c",
            "fine_circular_buffer.c",
            "oscillator-disciplining.c",
            "phase.c",
            "utils.c",
        },
        .flags = &CFLAGS,
    });
    lib.addIncludePath(logc.path("src"));
    lib.addIncludePath(b.path("include"));
    lib.installHeadersDirectory(b.path("include"), "", .{});
    lib.linkLibC();
    b.installArtifact(lib);
}

const CFLAGS = .{
    "-Wall",
    "-Wextra",
    "-Wmissing-prototypes",
    "-Wmissing-declarations",
    "-Wformat=2",
    "-Wold-style-definition",
    "-Wstrict-prototypes",
    "-Wpointer-arith",
    "-Werror",
};
