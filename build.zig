/// Tutorial how to build stm32cubeMX with zig: https://github.com/haydenridd/stm32-zig-porting-guide/tree/main
/// for small .bin size add '._user_heap_stack (NOLOAD) :' in the linker file
const std = @import("std");

// MARK: build
pub fn build(b: *std.Build) void {
    const exe_name = "test_stm32f3discovery";
    const elf_name = exe_name ++ ".elf";
    const bin_name = exe_name ++ ".bin";
    const hex_name = exe_name ++ ".hex";

    const target = b.resolveTargetQuery(.{
        .cpu_arch = .thumb,
        .os_tag = .freestanding,
        .abi = .eabihf,
        .cpu_model = std.zig.CrossTarget.CpuModel{ .explicit = &std.Target.arm.cpu.cortex_m4 },
        .cpu_features_add = std.Target.arm.featureSet(&[_]std.Target.arm.Feature{std.Target.arm.Feature.vfp4d16sp}),
    });
    const optimize = b.standardOptimizeOption(.{});

    const elf = b.addExecutable(.{
        .name = elf_name,
        .target = target,
        .optimize = optimize,
        .link_libc = false,
        .linkage = .static,
        .single_threaded = true,
    });

    var arm_none_eabi = GccArm{ .cortex = "cortex-m4", .fpu = "fpv4-sp-d16", .float_abi = "hard" };
    arm_none_eabi.init(b, elf);

    add_HAL(b, elf);
    add_Core(b, elf);
    add_Freertos(b, elf);
    add_Src(b, elf);

    elf.addIncludePath(b.path("Lib/nanoMODBUS"));
    elf.addCSourceFile(.{ .file = b.path("Lib/nanoMODBUS/nanomodbus.c"), .flags = &.{
        "-O0",
        "-std=gnu17",
    } });

    linker_config(b, elf);
    create_bin(b, elf, bin_name, hex_name);
}

// MARK: arm-none-eabi
const GccArm = struct {
    cortex: []const u8,
    fpu: []const u8,
    float_abi: []const u8,

    pgm: []const u8 = "",
    sysroot_path: []const u8 = "",
    multidir_relative_path: []const u8 = "",
    version: []const u8 = "",
    lib_path1: []const u8 = "",
    lib_path2: []const u8 = "",

    pub fn init(self: *GccArm, b: *std.Build, elf: *std.Build.Step.Compile) void {
        self.init_pgm(b);
        enable_float_formatting(b, elf);

        self.multidir_relative_path =
            std.mem.trim(u8, b.run(&.{
            self.pgm,
            b.fmt("-mcpu={s}", .{self.cortex}), // "-mcpu=cortex-m4",
            b.fmt("-mfpu={s}", .{self.fpu}), // "-mfpu=fpv4-sp-d16",
            b.fmt("-mfloat-abi={s}", .{self.float_abi}), // "-mfloat-abi=hard",
            "-print-multi-directory",
        }), "\r\n");
        self.sysroot_path = std.mem.trim(u8, b.run(&.{ self.pgm, "-print-sysroot" }), "\r\n");
        self.version = std.mem.trim(u8, b.run(&.{ self.pgm, "-dumpversion" }), "\r\n");
        self.lib_path1 = b.fmt("{s}/../lib/gcc/arm-none-eabi/{s}/{s}", .{ self.sysroot_path, self.version, self.multidir_relative_path });
        self.lib_path2 = b.fmt("{s}/lib/{s}", .{ self.sysroot_path, self.multidir_relative_path });

        self.add_libs(b, elf);
    }

    /// Try to find arm-none-eabi-gcc program at a user specified path, or PATH variable if none provided
    fn init_pgm(self: *GccArm, b: *std.Build) void {
        self.pgm =
            if (b.option([]const u8, "armgcc", "Path to arm-none-eabi-gcc compiler")) |arm_gcc_path|
            b.findProgram(&.{"arm-none-eabi-gcc"}, &.{arm_gcc_path}) catch {
                std.log.err("Couldn't find arm-none-eabi-gcc at provided path: {s}\n", .{arm_gcc_path});
                unreachable;
            }
        else
            b.findProgram(&.{"arm-none-eabi-gcc"}, &.{}) catch {
                std.log.err("Couldn't find arm-none-eabi-gcc in PATH, try manually providing the path to this executable with -Darmgcc=[path]\n", .{});
                unreachable;
            };
    }

    /// Allow user to enable float formatting in newlib (printf, sprintf, ...)
    fn enable_float_formatting(b: *std.Build, elf: *std.Build.Step.Compile) void {
        if (b.option(bool, "NEWLIB_PRINTF_FLOAT", "Force newlib to include float support for printf()")) |_| {
            elf.forceUndefinedSymbol("_printf_float"); // GCC equivalent : "-u _printf_float"
        }
    }

    /// add "nano" variant newlib C standard lib from arm-none-eabi-gcc library folders
    /// include C runtime objects bundled with arm-none-eabi-gcc
    fn add_libs(self: *GccArm, b: *std.Build, elf: *std.Build.Step.Compile) void {
        // Manually add "nano" variant newlib C standard lib from arm-none-eabi-gcc library folders
        elf.addLibraryPath(.{ .cwd_relative = self.lib_path1 });
        elf.addLibraryPath(.{ .cwd_relative = self.lib_path2 });
        elf.addSystemIncludePath(.{ .cwd_relative = b.fmt("{s}/include", .{self.sysroot_path}) });
        elf.linkSystemLibrary("c_nano");
        elf.linkSystemLibrary("m");

        // Manually include C runtime objects bundled with arm-none-eabi-gcc
        elf.addObjectFile(.{ .cwd_relative = b.fmt("{s}/crt0.o", .{self.lib_path2}) });
        elf.addObjectFile(.{ .cwd_relative = b.fmt("{s}/crti.o", .{self.lib_path1}) });
        elf.addObjectFile(.{ .cwd_relative = b.fmt("{s}/crtbegin.o", .{self.lib_path1}) });
        elf.addObjectFile(.{ .cwd_relative = b.fmt("{s}/crtend.o", .{self.lib_path1}) });
        elf.addObjectFile(.{ .cwd_relative = b.fmt("{s}/crtn.o", .{self.lib_path1}) });
    }
};

// MARK: HAL
fn add_HAL(b: *std.Build, elf: *std.Build.Step.Compile) void {
    elf.addSystemIncludePath(b.path("Drivers/STM32F3xx_HAL_Driver/Inc"));
    elf.addSystemIncludePath(b.path("Drivers/STM32F3xx_HAL_Driver/Inc/Legacy"));
    elf.addSystemIncludePath(b.path("Drivers/CMSIS/Device/ST/STM32F3xx/Include"));
    elf.addSystemIncludePath(b.path("Drivers/CMSIS/Include"));

    elf.addCSourceFiles(.{
        .files = &.{
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_usb.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_can.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_exti.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_spi.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_spi_ex.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pcd.c",
            "Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pcd_ex.c",
        },
        .flags = &.{
            "-Og",
            "-std=gnu17",
            "-DUSE_HAL_DRIVER",
            "-DSTM32F303xC",
        },
    });
}

// MARK: Core
fn add_Core(b: *std.Build, elf: *std.Build.Step.Compile) void {
    elf.addAssemblyFile(b.path("startup_stm32f303xc.s"));

    elf.addIncludePath(b.path("Core/Inc"));

    elf.addCSourceFile(.{ .file = b.path("Core/Src/syscalls.c"), .flags = &.{ "-Og", "-std=gnu17", "-DUSE_HAL_DRIVER", "-DSTM32F303xC" } });

    // Source files
    elf.addCSourceFiles(.{
        .files = &.{
            "Core/Src/can.c",
            "Core/Src/dma.c",
            "Core/Src/freertos.c",
            "Core/Src/gpio.c",
            "Core/Src/i2c.c",
            "Core/Src/main.c",
            "Core/Src/spi.c",
            "Core/Src/stm32f3xx_hal_msp.c",
            "Core/Src/stm32f3xx_hal_timebase_tim.c",
            "Core/Src/stm32f3xx_it.c",
            "Core/Src/system_stm32f3xx.c",
            "Core/Src/usart.c",
            "Core/Src/usb.c",
        },
        .flags = &.{ "-O0", "-std=gnu17", "-DUSE_HAL_DRIVER", "-DSTM32F303xC", "-Wall", "-Werror", "-Wpedantic", "-DNMBS_DEBUG" },
    });
}

// MARK: Core
// MARK: Freertos
fn add_Freertos(b: *std.Build, elf: *std.Build.Step.Compile) void {
    elf.addSystemIncludePath(b.path("Middlewares/Third_Party/FreeRTOS/Source/include"));
    elf.addSystemIncludePath(b.path("Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"));
    elf.addSystemIncludePath(b.path("Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"));

    elf.addCSourceFiles(.{
        .files = &.{
            "Middlewares/Third_Party/FreeRTOS/Source/croutine.c",
            "Middlewares/Third_Party/FreeRTOS/Source/event_groups.c",
            "Middlewares/Third_Party/FreeRTOS/Source/list.c",
            "Middlewares/Third_Party/FreeRTOS/Source/queue.c",
            "Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c",
            "Middlewares/Third_Party/FreeRTOS/Source/tasks.c",
            "Middlewares/Third_Party/FreeRTOS/Source/timers.c",
            "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c",
            "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c",
            "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c",
        },
        .flags = &.{
            "-Og",
            "-std=gnu17",
            "-DUSE_HAL_DRIVER",
            "-DUSE_FULL_LL_DRIVER",
            "-DSTM32F303xC",
        },
    });
}

fn add_Src(b: *std.Build, elf: *std.Build.Step.Compile) void {
    elf.addIncludePath(b.path("Src/driver"));
    elf.addIncludePath(b.path("Src"));

    elf.addCSourceFiles(.{
        .files = &.{
            "Src/nmbs_client_app.c",
        },
        .flags = &.{ "-O0", "-std=gnu17", "-Wall", "-Werror", "-Wpedantic" },
    });

    // Source files
    elf.addCSourceFiles(.{
        .files = &.{
            "Src/driver/nmbs_cmsis_os2_driver.c",
        },
        .flags = &.{ "-O0", "-std=gnu17", "-DUSE_HAL_DRIVER", "-DSTM32F303xC", "-Wall", "-Werror", "-Wpedantic" },
    });
}

// MARK: Linker
fn linker_config(b: *std.Build, elf: *std.Build.Step.Compile) void {
    elf.entry = .{ .symbol_name = "Reset_Handler" };
    elf.link_gc_sections = true;
    elf.link_data_sections = true;
    elf.link_function_sections = true;
    elf.setLinkerScriptPath(b.path("./STM32F303VCTx_FLASH_ZIG.ld"));
}

// MARK: bin
fn create_bin(b: *std.Build, elf: *std.Build.Step.Compile, bin_name: []const u8, hex_name: []const u8) void {
    // Produce .bin file from .elf
    const bin = b.addObjCopy(elf.getEmittedBin(), .{
        .format = .bin,
    });
    bin.step.dependOn(&elf.step);
    const copy_bin = b.addInstallBinFile(bin.getOutput(), bin_name);
    b.default_step.dependOn(&copy_bin.step);

    // Produce .hex file from .elf
    const hex = b.addObjCopy(elf.getEmittedBin(), .{
        .format = .hex,
    });
    hex.step.dependOn(&elf.step);
    const copy_hex = b.addInstallBinFile(hex.getOutput(), hex_name);
    b.default_step.dependOn(&copy_hex.step);

    b.installArtifact(elf);
}

// MARK: libs
