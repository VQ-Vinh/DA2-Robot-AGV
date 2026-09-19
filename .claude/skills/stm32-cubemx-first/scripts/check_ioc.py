#!/usr/bin/env python3
"""Doc STM32.ioc va doi chieu voi code CubeMX da sinh ra.

Dung sau khi nguoi dung bao da config xong trong CubeMX, de xac nhan config
that su da duoc luu VA da duoc generate thanh code -- hai viec khac nhau, va
quen bam GENERATE CODE la loi pho bien nhat.

    python check_ioc.py <duong-dan-thu-muc-STM32>
    python check_ioc.py <duong-dan> --expect TIM3,USART2

Exit code 1 neu co ngoai vi trong --expect chua thay.
"""
import argparse
import re
import sys
from pathlib import Path

# Ngoai vi -> ten file .c ma CubeMX sinh ra (ProjectManager.CoupleFile=true)
FILE_OF = {
    "ADC": "adc", "CAN": "can", "DAC": "dac", "DMA": "dma", "ETH": "eth",
    "GPIO": "gpio", "I2C": "i2c", "I2S": "i2s", "IWDG": "iwdg", "RTC": "rtc",
    "SPI": "spi", "TIM": "tim", "UART": "usart", "USART": "usart",
    "USB": "usb", "WWDG": "wwdg", "SDIO": "sdio", "FSMC": "fsmc",
}


def parse_ioc(path):
    cfg = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        cfg[k.strip()] = v.strip().replace(r"\:", ":").replace(r"\,", ",")
    return cfg


def family_of(periph):
    """TIM3 -> tim, USART2 -> usart, I2C1 -> i2c."""
    base = re.match(r"^([A-Z]+)", periph)
    return FILE_OF.get(base.group(1), None) if base else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("project", help="thu muc chua STM32.ioc")
    ap.add_argument("--expect", default="",
                    help="danh sach ngoai vi bat buoc, vd TIM3,USART2")
    args = ap.parse_args()

    root = Path(args.project)
    ioc = root / "STM32.ioc"
    if not ioc.exists():
        print(f"LOI: khong tim thay {ioc}")
        return 2

    cfg = parse_ioc(ioc)

    print("=" * 62)
    print(f"MCU        : {cfg.get('Mcu.UserName', '?')}  ({cfg.get('Mcu.Package', '?')})")
    print(f"CubeMX     : {cfg.get('MxCube.Version', '?')}")
    print(f"Toolchain  : {cfg.get('ProjectManager.TargetToolchain', '?')}")
    print(f"KeepUserCode: {cfg.get('ProjectManager.KeepUserCode', '?')}")
    print("=" * 62)

    # --- Clock: can de tinh prescaler cho timer, baudrate, ... ---
    clocks = [
        ("SYSCLK", "RCC.SYSCLKFreq_VALUE"),
        ("HCLK", "RCC.HCLKFreq_Value"),
        ("APB1", "RCC.APB1Freq_Value"),
        ("APB1 timer", "RCC.APB1TimFreq_Value"),
        ("APB2", "RCC.APB2Freq_Value"),
        ("APB2 timer", "RCC.APB2TimFreq_Value"),
    ]
    found = [(n, cfg[k]) for n, k in clocks if k in cfg]
    if found:
        print("\nCLOCK (dung de tinh prescaler / period):")
        for name, val in found:
            print(f"  {name:<12} {int(val) / 1e6:g} MHz")

    # --- Ngoai vi da bat trong .ioc ---
    n = int(cfg.get("Mcu.IPNb", 0))
    periphs = [cfg[f"Mcu.IP{i}"] for i in range(n) if f"Mcu.IP{i}" in cfg]
    print("\nNGOAI VI BAT TRONG .ioc:")
    print("  " + ", ".join(periphs) if periphs else "  (khong co)")

    # --- Chan da gan tin hieu ---
    npin = int(cfg.get("Mcu.PinsNb", 0))
    pins = [cfg[f"Mcu.Pin{i}"] for i in range(npin) if f"Mcu.Pin{i}" in cfg]
    print("\nCHAN DA GAN:")
    for pin in pins:
        sig = cfg.get(f"{pin}.Signal", "-")
        label = cfg.get(f"{pin}.GPIO_Label", "")
        mode = cfg.get(f"{pin}.Mode", "")
        extra = f"  [{label}]" if label else ""
        extra += f"  mode={mode}" if mode else ""
        print(f"  {pin:<16} {sig}{extra}")

    # --- Doi chieu voi file da sinh ---
    src = root / "Core" / "Src"
    inc = root / "Core" / "Inc"
    print("\nFILE CubeMX DA SINH (Core/Src):")
    # Ngoai vi suy ra tu .ioc, cong them file thuc te dang co tren dia --
    # GPIO khong nam trong Mcu.IP* nhung van sinh gpio.c, nen phai gop ca hai.
    wanted = {f for f in (family_of(p) for p in periphs) if f}
    wanted |= {p.stem for p in src.glob("*.c")} & set(FILE_OF.values())
    for fam in sorted(wanted):
        c_ok = (src / f"{fam}.c").exists()
        h_ok = (inc / f"{fam}.h").exists()
        mark = "OK  " if (c_ok and h_ok) else "THIEU"
        print(f"  [{mark}] {fam}.c / {fam}.h")

    # --- HAL module da bat ---
    conf = inc / "stm32f4xx_hal_conf.h"
    if conf.exists():
        mods = re.findall(r"^#define\s+HAL_(\w+)_MODULE_ENABLED",
                          conf.read_text(encoding="utf-8", errors="replace"),
                          re.M)
        print("\nHAL MODULE DA BAT:")
        print("  " + ", ".join(sorted(mods)))

    # --- File nguon trong CMake ---
    cml = root / "cmake" / "stm32cubemx" / "CMakeLists.txt"
    if cml.exists():
        listed = set(re.findall(r"Core/Src/(\w+)\.c",
                                cml.read_text(encoding="utf-8", errors="replace")))
        on_disk = {p.stem for p in src.glob("*.c")}
        missing = sorted(on_disk - listed)
        print("\nCMAKE:")
        if missing:
            print(f"  CANH BAO: co trong Core/Src nhung CHUA co trong CMakeLists: {', '.join(missing)}")
            print("            -> generate lai tu CubeMX, hoac them tay vao CMakeLists.txt")
        else:
            print("  OK - moi file trong Core/Src deu co trong danh sach build")

    # --- Kiem tra --expect ---
    rc = 0
    if args.expect:
        print("\nKIEM TRA YEU CAU:")
        for want in [w.strip() for w in args.expect.split(",") if w.strip()]:
            in_ioc = want in periphs
            fam = family_of(want)
            has_file = (src / f"{fam}.c").exists() if fam else False
            if in_ioc and has_file:
                print(f"  [OK   ] {want}: co trong .ioc va da sinh {fam}.c")
            elif in_ioc:
                print(f"  [THIEU] {want}: co trong .ioc nhung CHUA sinh {fam}.c "
                      f"-> chua bam GENERATE CODE?")
                rc = 1
            else:
                print(f"  [THIEU] {want}: CHUA bat trong .ioc")
                rc = 1
    print()
    return rc


if __name__ == "__main__":
    sys.exit(main())
