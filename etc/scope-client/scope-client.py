import math
import socket
import sys
import time
from collections import deque, defaultdict
from threading import Thread
from typing import List, Dict

import numpy as np

import matplotlib.pyplot as plt

duration = 1 / 10
sample_rate = 40000

time_x = np.array(np.linspace(0, 1, num=int(duration * sample_rate))) * duration
win_len = len(time_x)

from matplotlib.widgets import Slider


# def update_offset(val):
#    redraw()


class Channel:
    def __init__(self, chNum: int, name: str, type: str):
        self.chNum = chNum
        self.name = name
        self.sample_buffer = deque([math.nan] * win_len, maxlen=win_len)
        self.waveform: plt.Axes = None
        self.offset = 0

    def add_sample(self, y):
        self.sample_buffer.append(y)

    def plot(self, ax):
        self.waveform, = ax.plot(time_x, np.zeros(time_x.shape), label=self.name)  # channel 1 plot

    def redraw(self, trig_i):
        w = max(trig_i, len(self.sample_buffer) - win_len)
        a = np.array(self.sample_buffer)[w:]
        if trig_i:
            b = np.empty(win_len - len(a))
            b.fill(np.nan)
            a = np.concatenate([a, b])
            assert len(a) == win_len
        self.waveform.set_ydata(a + self.offset)


def redraw_loop(channels: Dict[str, Channel], fig, ax):
    trig_ch = 1
    trig_val = 120

    while True:
        trig_i = 0
        if offset_slider:
            trig_val = offset_slider.val

        if trig_ch and trig_ch in channels:
            buf = channels[trig_ch].sample_buffer
            # for i in range(len(buf) - 1, 1, -1):
            for i in range(1, len(buf), 1):
                if buf[i] >= trig_val > buf[i - 1]:
                    trig_i = i
                    break

        for ch in channels.values():
            ch.redraw(trig_i)

        ax.relim()
        ax.autoscale_view(True, False, True)
        fig.canvas.draw_idle()
        time.sleep(.1)


num_bytes_rx = 0
channelNames = dict()


def receive_loop(decoder):
    global num_bytes_rx

    addr = ("192.168.178.89", 24)
    while True:
        print('connecting', addr, '...')
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(4)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            s.connect(addr)
        except socket.timeout:
            print('connection timeout')
            time.sleep(2)

        t_last = time.time()

        while True:
            try:
                buf = s.recv(1024 * 4, 0)
                num_bytes_rx += len(buf)
                t_last = time.time()
                decoder.decode(buf)
            except Exception as e:
                if time.time() - t_last >= 8:
                    print('timeout!', e)
                    s.close()
                    # channelNames.clear()
                    break
                time.sleep(.1)


class ScopeDecoder:

    def __init__(self, on_channels, on_sample):
        self.on_channels = on_channels
        self.on_sample = on_sample

    def _decode_stream(self, ba: bytes):
        i = 0
        while i < len(ba):
            is_ex = ba[i] & 0b00000001
            ch = (ba[i] & 0b00001110) >> 1

            if not is_ex:
                v = ba[i + 1] << 4 | ((ba[i] & 0b11110000) >> 4)
                i += 1
                self.on_sample(ch, v)
            else:
                raise Exception("not impl")

            i += 1

    def decode(self, ba: bytes):
        if ba[:13] == b'###ScopeHead:' and b'###ENDHEAD\n' in ba[:200]:
            chs = []
            for chRepr in ba[13:ba[:200].index(b'###ENDHEAD\n')].decode('utf-8').strip(' ,').split(','):
                chNum, chRepr = chRepr.split('$')
                chName, chTyp = chRepr.split('=')
                chs.append((int(chNum), chName, chTyp))
            self.on_channels(chs)
            print('Received header', chs)
        else:
            self._decode_stream(ba)


offset_slider:plt.Slider = None


def main():
    global offset_slider
    channels = dict()

    fig, ax = plt.subplots()
    ax.grid()
    ax.set_xlabel("Time (s)")
    # ax.set_ylim([-4, 4])
    ax.set_ylabel("Voltage")

    # Adjust plot position so we can place the slider
    plt.subplots_adjust(left=0.2)
    # Make a vertically oriented slider to control the offset
    ax_offset = plt.axes([0.05, 0.2, 0.0225, 0.63], facecolor='lemonchiffon')
    offset_slider = Slider(ax=ax_offset, label="TrigLv", valmin=0, valmax=2 ** 12, valinit=200, orientation="vertical")

    t0 = 0
    num_samples = 0

    def on_sample(ch, y):
        nonlocal num_samples
        channels[ch].add_sample(y)
        num_samples += 1

    def on_channels(chs):
        nonlocal t0
        for ch_num, ch_name, ch_typ in chs:
            if ch_num not in channelNames:
                channelNames[ch_num] = ch_name
                ch = Channel(ch_num, ch_name, ch_typ)
                ch.plot(ax)
                channels[ch_num] = ch
            else:
                assert channelNames[ch_num] == ch_name

        ax.legend(loc='upper left')
        t0 = time.time()

    def lf_loop():
        while True:
            time.sleep(1)
            t_run = time.time() - t0
            sps = num_samples / t_run
            bps = num_bytes_rx / t_run
            sys.stdout.write('\rsps=%.1fk, bps=%.1fk' % (sps / 1000, bps / 1000))
            sys.stdout.flush()

    dec = ScopeDecoder(on_sample=on_sample, on_channels=on_channels)

    Thread(target=receive_loop, args=(dec,), daemon=True).start()
    Thread(target=redraw_loop, args=(channels, fig, ax), daemon=True).start()
    Thread(target=lf_loop, daemon=True).start()

    plt.show()


main()
# websockets.
