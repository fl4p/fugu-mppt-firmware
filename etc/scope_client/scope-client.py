import collections
import math
import select
import socket
import sys
import time
from collections import deque
from threading import Thread
from typing import Dict

#import fastplotlib as fpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.pyplot import figure
from scipy import signal

duration = 4 / 1
sample_rate = 2000

hide_channels = {'ntc'}

time_x = np.array(np.linspace(0, 1, num=int(duration * sample_rate))) * duration
win_len = len(time_x)

is_connected = False

from matplotlib.widgets import Slider


# def update_offset(val):
#    redraw()


def discover_scope_servers(stop_after=99, timeout=1):
    from zeroconf import ServiceBrowser, ServiceListener, Zeroconf

    addr = []

    class MyListener(ServiceListener):

        def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
            print(f"Service {name} updated")

        def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
            print(f"Service {name} removed")

        def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
            info = zc.get_service_info(type_, name)
            addr.extend((a, info.port) for a in info.parsed_addresses())

    zeroconf = Zeroconf()
    listener = MyListener()
    # browser = ServiceBrowser(zeroconf, "_http._tcp.local.", listener)
    browser = ServiceBrowser(zeroconf, "_scope._tcp.local.", listener)

    t0 = time.time()

    addrs = set()

    try:
        while time.time() - t0 < timeout:
            if addr:
                addrs.update(addr)
                if len(addrs) > stop_after:
                    return addrs
            time.sleep(.1)
    finally:
        zeroconf.close()

    return list(addrs)


class NotchIIR():
    def __init__(self, fs, f0, Q=30.0):
        self.b, self.a = signal.iirnotch(f0, Q, fs)
        self.zi = signal.lfilter_zi(self.b, self.a)

    def __call__(self, x):
        y, self.zi = signal.lfilter(self.b, self.a, x, zi=self.zi)
        return y


class EWMA:
    # Implement Exponential Weighted Moving Average
    def __init__(self, span: int):
        self.alpha = math.nan
        self.y = math.nan
        self.update_span(span)

    def update_span(self, span):
        self.alpha = (2 / (span + 1))

    def add(self, x):
        if not math.isfinite(x):
            return
        if not math.isfinite(self.y):
            self.y = x
        self.y = (1 - self.alpha) * self.y + self.alpha * x

    @property
    def value(self):
        return self.y


class SortedVector:
    """Keeps a sorteed list of all inserted elements"""

    def __init__(self):
        self.data_ = []

    def find_pos_(self, x):
        """Finds where given value is or should be"""

        (a, b) = (0, len(self.data_))

        while a < b:
            m = (a + b) // 2

            if self.data_[m] < x:
                a = m + 1
            else:
                b = m

        return a

    def insert(self, x):
        i = self.find_pos_(x)
        self.data_[i:i] = [x]

    def remove(self, x):
        i = self.find_pos_(x)
        del self.data_[i]

    def __getitem__(self, item):
        return self.data_[item]

    def __len__(self):
        return len(self.data_)


def median_from_sorted(s):
    """Returns the median of the _already_sorted_ list s"""
    l = len(s)
    m = l // 2
    return s[m] if l % 2 else (s[m] + s[m - 1]) / 2


class RunningMedian:

    def __init__(self, window_size):
        self.ring_ = [None] * window_size
        self.head_ = 0
        self.sorted_ = SortedVector()

    def insert(self, x):
        current = self.ring_[self.head_]
        self.ring_[self.head_] = x
        self.head_ = (self.head_ + 1) % len(self.ring_)

        if current != None: self.sorted_.remove(current)
        self.sorted_.insert(x)

    def median(self):
        return median_from_sorted(self.sorted_)


class Channel:
    def __init__(self, chNum: int, name: str, type: str):
        self.chNum = chNum
        self.name = name
        self.sample_buffer = deque([math.nan] * win_len, maxlen=win_len)
        self.capture_buffer = deque([], maxlen=200_000)
        self.waveform: plt.Axes = None
        self.fpl_waveform: 'fpl.LineGraphic' = None
        self.offset = 0
        self.win3 = collections.deque(maxlen=3)
        self.med = RunningMedian(5)
        self.notch50 = NotchIIR(451, 50 * 2, 20)
        self.notch60 = NotchIIR(451, 60 * 2, 20)
        from adaptive_noise_filter import anf
        self.anf = anf.AdaptiveLengthFilter(5, 200, 0.2)
        self.ewma = EWMA(90)
        self.t_first = 0
        self.n_samples = 0
        # self.win2 = collections.deque(maxlen=90)

    def add_sample(self, y):
        if math.isfinite(y):
            if self.t_first == 0:
                self.t_first = time.time()
            self.n_samples += 1

        self.capture_buffer.append(y)

        # y =statistics.mean(self.win3)
        # self.ewma.add(y)
        # y = self.ewma.value
        # TODO https://github.com/what-in-the-nim/real-time-iir-filter/blob/main/iir.py

        if self.chNum < 5 and math.isfinite(y):
            # filter inverter noise
            #y = self.notch50([y])[0]
            #y = self.notch60([y])[0]

            # filter spikes (after notch)
            self.med.insert(y)
            y = self.med.median()
            pass

        #y = self.anf.add(y)
        self.sample_buffer.append(y)

    def plot(self, ax):
        if figure is not None:
            self.fpl_waveform = figure[0, 0].add_line(data=np.zeros(time_x.shape), thickness=1, colors="magenta")
        else:
            self.waveform, = ax.plot(time_x, np.zeros(time_x.shape), label=f'#{self.chNum}: {self.name}')  # channel 1 plot

    def redraw(self, trig_i):
        w = max(trig_i, len(self.sample_buffer) - win_len)
        a = np.array(self.sample_buffer, dtype=np.float32)[w:]
        if trig_i:
            b = np.empty(win_len - len(a))
            b.fill(np.nan)
            a = np.concatenate([a, b])
            assert len(a) == win_len

        # if True:
        #     a -= np.nanmean(a)
        # a = median_filter(a, 5)

        if figure is None:
            self.waveform.set_ydata(a + self.offset)
        else:
            self.fpl_waveform.data[:, 1]  = a + self.offset

    def get_sr(self):
        return self.n_samples / (time.time() - self.t_first)

num_frames = 0

def redraw_loop(channels: Dict[str, Channel], fig, ax):
    global num_frames

    trig_ch = 1
    trig_val = 120
    last_num_bytes_rx = 0

    while True:

        if num_bytes_rx == last_num_bytes_rx:
            # only redraw if new data received
            time.sleep(0.1)
            continue
        last_num_bytes_rx = num_bytes_rx

        trig_i = 0
        if offset_slider:
            trig_val = offset_slider.val

        tcpu_0 = time.time()

        if trig_ch and trig_ch in channels:
            buf = channels[trig_ch].sample_buffer
            dc = np.mean(channels[trig_ch].sample_buffer)
            # for i in range(len(buf) - 1, 1, -1):
            for i in range(1, len(buf), 1):
                if buf[i] >= (trig_val + dc) > buf[i - 1]:
                    trig_i = i
                    break

        tcpu_trig = time.time()

        for ch in list(channels.values()):
            if ch.name not in hide_channels:
                ch.redraw(trig_i)

        tcpu_draw = time.time()

        ax.relim()
        ax.autoscale_view(True, False, True)
        fig.canvas.draw_idle()
        tcpu_figure = time.time()

        #tc = lambda dt: round(dt*1000)
        #print('tcpu',tc( tcpu_trig - tcpu_0), tc(tcpu_draw - tcpu_trig), tc(tcpu_figure - tcpu_draw))

        num_frames += 1
        time.sleep(.01)


num_bytes_rx = 0
channelNames = dict()


def receive_loop(decoder: 'ScopeDecoder'):
    global num_bytes_rx, is_connected

    while True:
        print('discovering hosts...')
        addr = discover_scope_servers()

        if not addr:
            print('no services discovered')
            time.sleep(1)
            continue

        print('Discovered services:', addr)
        addr = addr[0]
        # addr = ('192.168.1.208', 24)
        # addr =  ('192.168.1.231', 24)

        print('connecting', addr, '...')
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(4)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            s.connect(addr)
        except Exception as e:
            print('connection timeout', e)
            time.sleep(2)
            continue

        t_last = time.time()
        is_connected = True

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
                    is_connected = False
                    # channelNames.clear()
                    break
                else:
                    pass
                    # raise
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


offset_slider: plt.Slider = None


figure = None

def main():
    global offset_slider, figure
    channels: Dict[str, 'Channel'] = dict()

    fig, ax = plt.subplots()
    ax.grid()
    ax.set_xlabel("Time (s)")
    ax.set_xlim([0, duration])
    # ax.set_ylim([-4, 4])
    ax.set_ylabel("Voltage")

    # Adjust plot position so we can place the slider
    plt.subplots_adjust(left=0.2)
    # Make a vertically oriented slider to control the offset
    ax_offset = plt.axes([0.05, 0.2, 0.0225, 0.63], facecolor='lemonchiffon')
    offset_slider = Slider(ax=ax_offset, label="TrigLv", valmin=0, valmax=2 ** 12, valinit=200, orientation="vertical")

    #figure = fpl.Figure(size=(700, 560))

    def onpick(event):
        legend_label = event.artist.get_label()

        for x in range(len(ax.get_lines())):
            if legend_label == ax.get_lines()[x].get_label():
                ax.get_lines()[x].set_alpha(0)
                fig.canvas.draw()

    for legend in plt.legend().get_lines():
        legend.set_picker(5)
    fig.canvas.mpl_connect('pick_event', onpick)

    t0 = 0
    num_samples = 0

    def on_sample(ch, y):
        nonlocal num_samples
        channels[ch].add_sample(y)
        num_samples += 1

    def on_channels(chs):
        print('channels', chs)
        nonlocal t0, num_samples
        for ch_num, ch_name, ch_typ in chs:
            if ch_num not in channelNames:
                channelNames[ch_num] = ch_name
                ch = Channel(ch_num, ch_name, ch_typ)
                if ch_name not in hide_channels:
                    ch.plot(ax)
                channels[ch_num] = ch
            else:
                assert channelNames[ch_num] == ch_name

        ax.legend(loc='upper left')
        t0 = time.time()
        num_samples = 0

        if figure is not None:
            figure[0, 0].axes.grids.xy.visible = True
            figure.show()

    def console():
        try:
            if select.select([sys.stdin, ], [], [], 0.1)[0]:
                line = next(sys.stdin)
                print("Got:", repr(line.strip()))
                inp = line.strip()
                if inp == 's':
                    for chn, ch in channels.items():
                        fn = f'{ch.name}_{int(round(ch.get_sr()))}hz.csv'
                        pd.Series(ch.capture_buffer).to_csv(fn, index=False)
                        print('written', fn)

            # else:
            # print ("No data for 2 secs")

        except StopIteration:
            return

    def lf_loop():
        while True:
            time.sleep(1)
            t_run = time.time() - t0
            sps = num_samples / t_run
            bps = num_bytes_rx / t_run
            if is_connected:
                srs = 'SR:'
                for ch in channels.values():
                    if ch.name in {'ucTemp', 'ntc'}:
                        continue
                    srs += (f'{ch.name}={round(ch.get_sr(), 0)},'
                            f'anf(L={ch.anf.span},N={round(ch.anf.noise_level * 1000, 2)},S={round(ch.anf.signal_level * 1000, 2)}| ')
                # TODO line up: "\033[A" or curses module
                sys.stdout.write('\rfps=%.0f|sps=%.1fk|bps=%.1fk|t=%.1fs|%s' % (num_frames/t_run, sps / 1000, bps / 1000, t_run, srs))
                sys.stdout.flush()

            console()

    dec = ScopeDecoder(on_sample=on_sample, on_channels=on_channels)

    Thread(target=receive_loop, args=(dec,), daemon=True).start()
    Thread(target=redraw_loop, args=(channels, fig, ax), daemon=True).start()
    Thread(target=lf_loop, daemon=True).start()

    if 'fpl' in globals():
        fpl.loop.run()
    plt.show()

print('discovering hosts..')
print(discover_scope_servers())
main()
# websockets.
