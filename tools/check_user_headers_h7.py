"""Check every User header as a standalone C++17 translation unit on H723.

Uses real Cube headers; does not link firmware or instantiate every template.
C headers are additionally checked as standalone C11 translation units.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board-root", type=Path, required=True)
    args = parser.parse_args()
    board = args.board_root.resolve()
    includes = [ROOT / "User"] + [board / p for p in (
        "Core/Inc", "Drivers/STM32H7xx_HAL_Driver/Inc", "Drivers/CMSIS/Include",
        "Drivers/CMSIS/Device/ST/STM32H7xx/Include",
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
        "Middlewares/Third_Party/FreeRTOS/Source/include",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F")]
    for path in includes:
        if not path.is_dir():
            parser.error(f"Missing include directory: {path}")
    flags = ["arm-none-eabi-g++", "-x", "c++", "-std=c++17", "-fsyntax-only",
             "-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16", "-mfloat-abi=hard",
             "-DSTM32H723xx", "-DUSE_HAL_DRIVER", "-fno-exceptions", "-fno-rtti",
             "-Wall", "-Wextra", "-Wpedantic", "-pedantic-errors", "-Werror"]
    flags += ["-I" + str(p) for p in includes]
    headers = sorted(p for p in (ROOT / "User").rglob("*") if p.suffix in (".h", ".hpp"))

    def check(path):
        name = path.relative_to(ROOT / "User").as_posix()
        result = subprocess.run(flags + ["-"], input=f'#include "{name}"\n',
                                text=True, capture_output=True, cwd=ROOT)
        if not result.returncode and path.suffix == ".h":
            cflags = ["arm-none-eabi-gcc", "-x", "c", "-std=c11"] + [
                flag for flag in flags[5:] if flag not in ("-fno-exceptions", "-fno-rtti")]
            result = subprocess.run(cflags + ["-fsyntax-only", "-"],
                                    input=f'#include "{name}"\ntypedef int mr_header_check_t;\n', text=True,
                                    capture_output=True, cwd=ROOT)
        return name, result

    failures = 0
    with ThreadPoolExecutor(max_workers=4) as pool:
        for name, result in pool.map(check, headers):
            if result.returncode:
                failures += 1
                print(f"FAIL {name}\n{result.stderr}", flush=True)
    print(f"Headers: {len(headers) - failures}/{len(headers)} passed")
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
