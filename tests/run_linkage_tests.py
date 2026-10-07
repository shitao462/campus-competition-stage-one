"""Compile and run the real control code in an ARM emulator, without a board.

Dependencies: unicorn, pyelftools; arm-none-eabi-g++ must be on PATH.
"""

from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "build/test_dependencies"))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

binary = ROOT / "build/linkage_test.elf"
subprocess.run(
    ["arm-none-eabi-g++", "-std=gnu++17", "-mcpu=cortex-m3", "-mthumb",
     "-mfloat-abi=soft", "-O0", "-fno-exceptions", "-fno-rtti", "-nostartfiles",
     "--specs=nosys.specs", "-Wl,-e,run_tests", "-Wl,-Ttext=0x10000",
     "-Iapplications", "-Isp_middleware", "tests/linkage_controller_test.cpp",
     "applications/linkage_controller.cpp", "sp_middleware/tools/pid/pid.cpp",
     "sp_middleware/tools/math_tools/math_tools.cpp",
     "sp_middleware/tools/mahony/mahony.cpp", "-o", str(binary)],
    cwd=ROOT, check=True,
)
emulator = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
emulator.mem_map(0, 0x400000)
emulator.mem_map(0x20000000, 0x20000)
with binary.open("rb") as stream:
    elf = ELFFile(stream)
    for segment in elf.iter_segments():
        if segment["p_type"] == "PT_LOAD":
            emulator.mem_write(segment["p_vaddr"], segment.data())
    entry = elf.header["e_entry"]
emulator.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
emulator.reg_write(UC_ARM_REG_LR, 0x300001)
emulator.emu_start(entry | 1, 0x300000, timeout=20000000, count=180000000)
assert emulator.reg_read(UC_ARM_REG_PC) == 0x300000, "Test did not return before emulator limit"
result = emulator.reg_read(UC_ARM_REG_R0)
assert result == 0, f"Control scenario failed: {result}"
print("PASS: ratios, both manual inputs, retained reference, ratio transition, "
      "disable, current limits/slew/reversal, settled torque release, encoder wrapping, "
      "Mahony yaw, continuous low-speed tracking with 10 ms target updates, "
      "fixed encoder mapping, shortest-path reset, completion, and mode transitions.")
