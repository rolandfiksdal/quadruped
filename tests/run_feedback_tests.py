"""Compile real firmware modules for ARM and run them with a fake HAL in Unicorn.

Test-only dependencies (kept inside the ignored build directory):
  python -m pip install --target build/test-deps unicorn==2.1.4 pyelftools==0.33
Run:
  python tests/run_feedback_tests.py --cc /path/to/arm-none-eabi-gcc
No connection to the robot is made.
"""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "build/test-deps"))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", default="arm-none-eabi-gcc")
    args = parser.parse_args()
    firmware = ROOT / "stm32/quadruped_stm32"
    output = ROOT / "build/feedback-tests.elf"
    output.parent.mkdir(exist_ok=True)
    includes = ["Core/Inc", "Drivers/STM32G4xx_HAL_Driver/Inc",
                "Drivers/STM32G4xx_HAL_Driver/Inc/Legacy",
                "Drivers/BSP/STM32G4xx_Nucleo",
                "Drivers/CMSIS/Device/ST/STM32G4xx/Include", "Drivers/CMSIS/Include"]
    subprocess.run([
        args.cc, "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft", "-std=gnu11",
        "-O0", "-g", "-Wall", "-Wextra", "-Wno-unused-parameter", "-Werror",
        "-DSTM32G474xx", "-DUSE_HAL_DRIVER", "-DUSE_NUCLEO_64",
        "-ffunction-sections", "-fdata-sections", "-nostartfiles",
        "--specs=nosys.specs", "-Wl,--gc-sections,-e,run_tests,-Ttext=0x10000",
        *[f"-I{firmware / inc}" for inc in includes],
        str(ROOT / "tests/feedback_freshness.c"),
        str(firmware / "Core/Src/robstride_can.c"),
        str(firmware / "Core/Src/robot_joints.c"), "-lm", "-o", str(output)
    ], check=True)

    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    cpu.mem_map(0x10000, 0x100000)
    cpu.mem_map(0x20000000, 0x10000)
    with output.open("rb") as stream:
        elf = ELFFile(stream)
        for segment in elf.iter_segments():
            if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
                cpu.mem_write(segment["p_vaddr"], segment.data())
        start = elf.get_section_by_name(".symtab").get_symbol_by_name("run_tests")[0]["st_value"]
    stop = 0x100000
    cpu.reg_write(UC_ARM_REG_SP, 0x20010000)
    cpu.reg_write(UC_ARM_REG_LR, stop | 1)
    cpu.emu_start(start | 1, stop, timeout=5_000_000, count=2_000_000)
    from unicorn.arm_const import UC_ARM_REG_PC
    if cpu.reg_read(UC_ARM_REG_PC) != stop:
        raise SystemExit("FAIL: emulation did not return within its execution limit")
    failure_line = cpu.reg_read(UC_ARM_REG_R0)
    if failure_line:
        raise SystemExit(f"FAIL: tests/feedback_freshness.c:{failure_line}")
    print("PASS: feedback discovery/validity, timeout boundary, recovery, gap telemetry,")
    print("      command gating, malformed frames, reset, tick wrap, fault/offline precedence")


if __name__ == "__main__":
    main()
