# -*- coding: utf-8 -*-
"""
接收一帧（1030 字节）并画时域波形 + 幅度谱。

前提：
    XCOM 已关闭串口（同一端口不能被两个程序占）
    板子在上电运行、串口在发帧

运行：
    python plot_wave.py

输出：
    上位机\\wave.png  —— 上半张时域，下半张频谱
    同时打印峰值频率、峰峰值、直流电平
"""

import time

import numpy as np
import serial
import serial.tools.list_ports

BAUD = 115200
FS = 1000.0          # 采样率 1 kHz（TIM3 触发 ADC 的频率）
N = 512              # 一帧点数
FRAME_LEN = 4 + N * 2 + 2    # 1030
PAYLOAD_LEN = 4 + N * 2      # 1028（CRC 覆盖的部分）
VREF_MV = 3300.0

# matplotlib 中文显示（Windows 自带雅黑）
import matplotlib
matplotlib.use('Agg')        # 先存图，不弹窗；要看交互窗口改成 TkAgg 并去掉 savefig 后的注释
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x1A86 and p.pid == 0x7523:
            return p.device
    return None


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def recv_one_frame(s, timeout=5.0):
    """一直读到凑够一帧完整且 CRC 正确的帧"""
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        n = s.in_waiting
        if n:
            buf += s.read(n)

        start = -1
        for i in range(len(buf) - 1):
            if buf[i] == 0xAA and buf[i + 1] == 0x55:
                start = i
                break
        if start < 0:
            time.sleep(0.01)
            continue

        if len(buf) - start < FRAME_LEN:
            time.sleep(0.01)
            continue

        frame = buf[start:start + FRAME_LEN]
        calc = crc16_ccitt(frame[:PAYLOAD_LEN])
        recv = frame[PAYLOAD_LEN] | (frame[PAYLOAD_LEN + 1] << 8)
        if calc == recv:
            seq = frame[2] | (frame[3] << 8)
            return seq, frame
        del buf[:start + 1]      # 这一帧坏了，丢掉帧头继续找下一个

    return None, None


def main():
    port = find_port()
    if port is None:
        print('没找到 CH340（VID 1A86 / PID 7523），检查 USB 线和板子供电')
        return

    print('打开 %s' % port)
    s = serial.Serial(port, BAUD, timeout=0.05, dsrdtr=False, rtscts=False)
    time.sleep(0.2)
    s.setDTR(False)
    s.setRTS(False)
    s.reset_input_buffer()

    print('等一帧（最多 5 秒）...')
    seq, frame = recv_one_frame(s)
    s.close()

    if seq is None:
        print('没收到完整且 CRC 正确的帧。先用 check_frame.py 确认在发数据。')
        return

    print('收到 seq=%d' % seq)

    # ---- 解析数据段：512 个 uint16，小端 ----
    raw = np.frombuffer(frame[4:4 + N * 2], dtype='<u2').astype(np.float64)
    mv = raw * VREF_MV / 4095.0
    t = np.arange(N) / FS * 1000.0        # 0—511 ms

    vmin, vmax = mv.min(), mv.max()
    dc = mv.mean()
    print('直流电平 %.1f mV，峰峰值 %.1f mV（范围 %.1f—%.1f）' % (dc, vmax - vmin, vmin, vmax))

    # ---- FFT ----
    x = mv - dc                            # 去掉直流，否则 0 Hz 那条线压倒一切
    win = np.hanning(N)                    # 加汉宁窗，减少频谱泄漏
    X = np.fft.rfft(x * win)
    freq = np.fft.rfftfreq(N, 1.0 / FS)
    amp = 2.0 * np.abs(X) / (N * win.sum() / N)    # 归一化到 mV

    k = int(np.argmax(amp[1:])) + 1        # 跳过 0 Hz
    print('峰值频率 %.1f Hz（分辨率 %.2f Hz），幅度 %.1f mV' % (freq[k], FS / N, amp[k]))

    # ===== THD：总谐波失真 =====
    k1 = int(np.argmax(amp[1:])) + 1          # 基波所在格（复用上面找出的 k）
    f1 = freq[k1]
    V1 = amp[k1]
    harm = []
    for n in range(2, 11):                    # 2—10 次谐波
        kk = int(round(k1 * n))
        if kk >= len(amp):
            break
        lo, hi = max(1, kk - 1), min(len(amp), kk + 2)
        harm.append(float(amp[lo:hi].max()))  # 格心 ±1 格取最大，防栅栏效应
    thd = float(np.sqrt(np.sum(np.square(harm))) / V1 * 100.0) if V1 > 0 else 0.0

    print('  基波 %.2f Hz，幅度 %.1f mV' % (f1, V1))
    print('  THD = %.3f %%（2—10 次谐波）' % thd)
    if harm:
        print('  谐波幅度：' + ' '.join('%.2f' % h for h in harm) + ' mV')
    
    # ---- 画图 ----
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))

    ax1.plot(t, mv, color='#1f77b4', linewidth=1.2)
    ax1.set_xlabel('时间 / ms')
    ax1.set_ylabel('电压 / mV')
    ax1.set_title('时域波形（512 点，1 kHz，整帧 512 ms）')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(0, 512)
    if vmax - vmin < 5:
        ax1.set_ylim(dc - 50, dc + 50)

    ax2.plot(freq[1:], amp[1:], color='#d62728', linewidth=1.2)
    ax2.set_xlabel('频率 / Hz')
    ax2.set_ylabel('幅度 / mV')
    ax2.set_title('幅度谱（去直流 + 汉宁窗，频率分辨率 %.2f Hz）' % (FS / N))
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(0, FS / 2)

    plt.tight_layout()
    out = 'wave.png'
    plt.savefig(out, dpi=130)
    print('已保存 %s' % out)

    if matplotlib.get_backend().lower() != 'agg':
        plt.show()


if __name__ == '__main__':
    main()
