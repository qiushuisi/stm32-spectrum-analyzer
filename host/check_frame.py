# -*- coding: utf-8 -*-
"""
串口帧验收脚本（诊断增强版）
- 自动找 CH340（VID 1A86 / PID 7523）
- 打开后主动拉低 DTR/RTS，避免某些板子被这两根线复位
- 每秒打印一次收到多少字节，便于判断“完全没数据”还是“偶尔有”

用法：
    1. XCOM 里点“关闭串口”或直接关掉 XCOM
    2. python check_frame.py
"""

import sys
import time

import serial
import serial.tools.list_ports

BAUD = 115200
SECONDS = 5

FRAME_LEN = 1030      # 帧头2 + 序号2 + 数据1024 + CRC2
PAYLOAD_LEN = 1028    # CRC 覆盖的部分（不含 CRC 自己）


def find_port():
    """优先找 CH340"""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == 0x1A86 and p.pid == 0x7523:
            return p.device, p.description
    return None, None


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, 不反转不异或"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def main():
    port, desc = find_port()
    if port is None:
        print('没找到 CH340（VID 1A86 / PID 7523）。')
        print('当前串口列表：')
        for p in serial.tools.list_ports.comports():
            print('  %-6s %s' % (p.device, p.description))
        print('如果板子确实插着，检查一下 USB 线是不是换了口。')
        return

    print('打开 %s (%s)' % (port, desc))
    s = serial.Serial(port, BAUD, timeout=0.05, dsrdtr=False, rtscts=False)
    time.sleep(0.2)
    s.setDTR(False)
    s.setRTS(False)
    s.reset_input_buffer()

    buf = bytearray()
    t0 = time.time()
    last_sec = 0

    while time.time() - t0 < SECONDS:
        n = s.in_waiting
        if n:
            buf += s.read(n)
        time.sleep(0.01)

        sec = int(time.time() - t0)
        if sec != last_sec:
            last_sec = sec
            print('  %d 秒: 累计 %d 字节' % (sec, len(buf)))

    s.close()
    print('总共收到 %d 字节' % len(buf))

    if len(buf) == 0:
        print('一个字节都没有。可能性：')
        print('  1) 板子没在发（程序停了 / 被复位）—— 用 XCOM 打开同一个口看一眼')
        print('  2) 线松了 / 板子没供电（看 PWR 灯）')
        print('  3) 波特率不是 115200')
        return

    idx = [i for i in range(len(buf) - 1) if buf[i] == 0xAA and buf[i + 1] == 0x55]
    print('帧头 AA 55 出现 %d 次' % len(idx))
    if len(idx) < 2:
        print('帧头太少——数据在流，但不是这个帧格式')
        print('前 32 字节: %s' % ' '.join('%02X' % b for b in buf[:32]))
        return

    gaps = [idx[i + 1] - idx[i] for i in range(len(idx) - 1)]
    uniq = sorted(set(gaps))
    print('相邻帧头间隔（去重）: %s' % uniq)
    if uniq == [FRAME_LEN]:
        print('  [OK] 间隔全部 = %d，帧长正确' % FRAME_LEN)
    else:
        print('  [!!] 期望全部 %d；别的值说明丢帧或长度没改' % FRAME_LEN)

    ok = bad = 0
    for start in idx:
        end = start + FRAME_LEN
        if end > len(buf):
            break
        frame = buf[start:end]
        seq = frame[2] | (frame[3] << 8)

        calc = crc16_ccitt(frame[:PAYLOAD_LEN])
        recv = frame[PAYLOAD_LEN] | (frame[PAYLOAD_LEN + 1] << 8)

        if calc == recv:
            ok += 1
            if ok == 1:
                print('  首帧 OK: seq=%d, crc=0x%04X' % (seq, calc))
        else:
            bad += 1
            if bad <= 3:
                print('  seq=%d 算出来 0x%04X，帧里写的 0x%04X' % (seq, calc, recv))

    print('CRC 校验：通过 %d 帧，失败 %d 帧' % (ok, bad))
    if ok and bad == 0:
        print('  [OK] 全部通过，固件这一环完成')
    elif bad:
        print('  [!!] 有失败帧，说明算法或长度不一致')


if __name__ == '__main__':
    main()
