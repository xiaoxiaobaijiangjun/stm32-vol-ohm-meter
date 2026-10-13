#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电压电阻检测仪 - 两点校准上位机（USART1: PA9/PA10, 9600 8N1）

用法（在仓库根目录）：
  python tools/cal_host.py --port COM7 status        # 读当前校准参数
  python tools/cal_host.py --port COM7 calibrate     # 四量程两点校准向导
  python tools/cal_host.py --port COM7 monitor       # 看 BLE/串口上报行
  python tools/cal_host.py --port COM7 monitor --ble # 顺便发 BLE ON 打开 1Hz 上报

设备侧命令（同一串口，手机经 JDY-23 BLE 也能用）：
  STAT | CAP <0-3> | SET <0-3> <k> <b> | SAVE | DEFAULT | EXIT | BLE ON|OFF

模型：每量程 y = k·x + b，x 为校准前计算值（电压 mV / 电阻 Ω），
本工具负责两点解算 k=(y2-y1)/(x2-x1)、b=y1-k·x1，固件只存和应用。
"""

import argparse
import sys
import time

import serial

RANGES = [
    dict(idx=0, name="电压 0-5V", mode="电压模式（Mode:Vol，量程自动）", unit="mV",
         hint="稳压源给 1.000V 和 4.000V 两点（真值以万用表读数为准）"),
    dict(idx=1, name="电压 5-15V", mode="电压模式（Mode:Vol，量程自动）", unit="mV",
         hint="稳压源给 6.00V 和 12.00V 两点"),
    dict(idx=2, name="电阻 0-1K", mode="电阻模式（Mode:Res）", unit="Ω",
         hint="精密电阻两点，建议 100Ω 和 900Ω 左右（真值以万用表读数为准）"),
    dict(idx=3, name="电阻 1-100K", mode="电阻模式（Mode:Res，量程自动到 1K-100K）", unit="Ω",
         hint="精密电阻两点，建议 10kΩ 和 90kΩ 左右"),
]


def open_port(port: str, baud: int) -> serial.Serial:
    return serial.Serial(port, baud, timeout=0.4)


def cmd(ser: serial.Serial, line: str) -> list:
    ser.reset_input_buffer()
    ser.write((line + "\r\n").encode())
    out = []
    t0 = time.time()
    while time.time() - t0 < 1.5:
        r = ser.readline().decode(errors="replace").strip()
        if r:
            out.append(r)
            t0 = time.time() - 1.1  # one more short quiet window, then stop
    return out


def ask_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt).strip())
        except ValueError:
            print("  数字不合法，重输")


def capture(ser: serial.Serial, rng: dict, point_no: int) -> float:
    while True:
        ans = input(f"\n[{rng['name']}] 第 {point_no} 点：施加已知输入、等读数稳定后回车抓取（q=跳过该量程） ")
        if ans.strip().lower() == "q":
            raise KeyboardInterrupt
        r = cmd(ser, f"CAP {rng['idx']}")
        line = r[-1] if r else ""
        if line.startswith("CAP") and "ERR" not in line:
            x = float(line.split()[2])
            print(f"  抓到 x = {x:.2f} {rng['unit']}")
            return x
        if "ERR" in line:
            print(f"  设备回复 {line} —— 先把表切到 {rng['mode']} 再抓")
        else:
            print("  无有效回复，检查接线 / 串口号 / 波特率 9600")


def calibrate(ser: serial.Serial) -> None:
    done = []
    for rng in RANGES:
        print(f"\n===== 量程 {rng['idx']}: {rng['name']} =====")
        print(f"前置：{rng['mode']}；建议两点：{rng['hint']}")
        if input("校准该量程？(y/N) ").strip().lower() != "y":
            print("  跳过")
            continue
        try:
            x1 = capture(ser, rng, 1)
            y1 = ask_float(f"  真值 y1（{rng['unit']}，万用表读数）= ")
            x2 = capture(ser, rng, 2)
            y2 = ask_float(f"  真值 y2（{rng['unit']}，万用表读数）= ")
        except KeyboardInterrupt:
            print("  跳过该量程")
            continue
        if abs(x2 - x1) < 1e-6:
            print("  两点 x 相同，无法拟合，跳过")
            continue
        k = (y2 - y1) / (x2 - x1)
        b = y1 - k * x1
        print(f"  拟合结果：k = {k:.6f}, b = {b:.4f}")
        if not (0.05 < k < 20.0):
            print("  k 超出设备合理范围 [0.05, 20]，放弃写入（检查真值/接线）")
            continue
        r = cmd(ser, f"SET {rng['idx']} {k:.6f} {b:.4f}")
        print(f"  设备：{r[-1] if r else '无回复'}")
        done.append(rng["idx"])
    if done:
        r = cmd(ser, "SAVE")
        print(f"\n保存到 Flash：{r[-1] if r else '无回复'}")
        r = cmd(ser, "STAT")
        print("回读校验：")
        for line in r:
            print(f"  {line}")
        print("\n收尾：把校准前后读数对比表（3 个量程 × 至少 3 点）记进 02-实验记录")
    else:
        print("\n没有可保存的量程")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="串口号，如 COM7")
    ap.add_argument("--baud", type=int, default=9600, help="默认 9600（JDY-23 原生）")
    ap.add_argument("command", choices=["status", "calibrate", "monitor"])
    ap.add_argument("--ble", action="store_true", help="monitor 前先发 BLE ON")
    args = ap.parse_args()

    ser = open_port(args.port, args.baud)
    try:
        if args.command == "status":
            for line in cmd(ser, "STAT"):
                print(line)
        elif args.command == "calibrate":
            calibrate(ser)
        else:
            if args.ble:
                print(cmd(ser, "BLE ON")[-1] if cmd(ser, "BLE ON") else "无回复")
            print("监听中（Ctrl+C 退出）...")
            try:
                while True:
                    line = ser.readline().decode(errors="replace").strip()
                    if line:
                        print(time.strftime("%H:%M:%S"), line)
            except KeyboardInterrupt:
                pass
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
