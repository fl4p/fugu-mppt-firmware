import math
from typing import Sequence

import pandas as pd
from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import lfilter_zi


class NotchIIR:
    def __init__(self, fs, f0, Q=30.0):
        self.b, self.a = signal.iirnotch(f0, Q, fs)
        self.zi = signal.lfilter_zi(self.b, self.a)

    def __call__(self, x):
        y, self.zi = signal.lfilter(self.b, self.a, x, zi=self.zi)
        return y


fs = 450.0  # Sample frequency (Hz)
f0 = 50.0*2  # Frequency to be removed from signal (Hz)

y = np.array([1 + 0.1 * math.sin(2 * math.pi * i / fs * f0) for i in range(1000)])

y = pd.read_csv('vout_50hz_inverter.csv')['0'].values

Q = 30.0  # Quality factor
b, a = signal.iirnotch(f0, Q, fs)
y_filt = signal.filtfilt(b, a, y)

zi = signal.lfilter_zi(b, a)
y_filt2 = signal.lfilter(b, a, y, zi=zi)[0]

nr = NotchIIR(fs, f0, 31)

y_filt = np.array([])
for b in np.split(y, 10):
    y_filt = np.concatenate([y_filt, nr(b)])

# s = signal_generator([(8, 5),], (500, 1), fs)

# fi = iir.IIR(1, int(fs), False)
# fi.add_filter(2, (f0*0.9, f0*6/5), 'bandstop')
# fi.coeffs = [np.array([b,a])]
# y_filt2 = fi.filter(np.array([y]).T).T[0]

plt.plot(y, label='in')
plt.plot(y_filt, label='yf')
plt.plot(NotchIIR(fs, f0=100, Q=20.0)(y), label='yf2')
#plt.plot(y_filt2, label='yf2')
plt.legend()
plt.show()
