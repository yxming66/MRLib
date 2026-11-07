"""Compile actual User IO sources against a read-only STM32H723 Cube/HAL tree.
This is a compilation check, not a firmware link or hardware test.
"""
import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board-root", type=Path, required=True)
    args = parser.parse_args()
    board = args.board_root.resolve()
    includes = [ROOT / "User", ROOT / "User/device"] + [board / p for p in (
        "Core/Inc", "Drivers/STM32H7xx_HAL_Driver/Inc", "Drivers/CMSIS/Include",
        "Drivers/CMSIS/Device/ST/STM32H7xx/Include",
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
        "Middlewares/Third_Party/FreeRTOS/Source/include",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F")]
    for path in includes:
        if not path.is_dir():
            parser.error(f"Missing include directory: {path}")
    output = ROOT / "build/user-io-h7"
    output.mkdir(parents=True, exist_ok=True)
    flags = ["-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16", "-mfloat-abi=hard",
             "-DSTM32H723xx", "-DUSE_HAL_DRIVER", "-O1", "-Wall", "-Wextra",
             "-Wpedantic", "-pedantic-errors", "-Werror"] + ["-I" + str(p) for p in includes]
    sources = ["User/component/communication/mrlink/mrlink.c", "User/component/crc16.c",
               "User/bsp/dma.c", "User/bsp/uart.c", "User/bsp/spi.c", "User/bsp/gpio.c", "User/bsp/fdcan.c",
               "User/bsp/mrlink/channel.c", "User/bsp/mrlink/channel_uart.c",
               "User/bsp/mrlink/channel_usb.c", "User/bsp/mrlink/channel_fdcan.c",
               "User/device/bmi088.c", "User/device/dr16.c",
               "tests/device/mrlink_embedded_compile_check.cpp"]
    for name in sources:
        cpp = name.endswith(".cpp")
        command = ["arm-none-eabi-g++" if cpp else "arm-none-eabi-gcc", *flags,
                   "-std=c++17" if cpp else "-std=c11"]
        if cpp:
            command += ["-fno-exceptions", "-fno-rtti"]
        subprocess.run(command + ["-c", str(ROOT/name), "-o", str(output/(Path(name).stem+".o"))],
                       check=True, cwd=ROOT)
        print(f"PASS {name}", flush=True)

if __name__ == "__main__":
    main()
