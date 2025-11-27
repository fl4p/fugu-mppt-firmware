import math
from typing import Sequence

import pandas as pd
from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import lfilter_zi

fs = 450.0  # Sample frequency (Hz)

y = pd.read_csv('iout_50hz_inverter_noise.csv')['0']

#y = (y-y.mean()) / y.mean()

noise_free = y.ewm(40).mean().ewm(40).mean()
noise = y - noise_free


from ewm import EWM




#plt.plot(y, label='in')
#plt.plot(y.rolling(80).mean(), label='mean')
#plt.plot(y.rolling(40).mean().rolling(40).mean(), label='mean2')
#plt.plot(noise, label='noise')
#plt.plot(noise_free, label='noiseFree')
plt.plot((y/y.mean()).ewm(80).std(), label='std(y)')
nstddev = y.map(EWM(80, 10e-9).nstddev)
plt.plot(nstddev, label='nstddev(y) %.4f' % (nstddev.mean()))

y = y.rolling(10).mean() #ewm(2).mean()#.ewm(2).mean()
plt.plot((y/y.mean()).ewm(80).std(), label='std(y2)')

nstddev = y.map(EWM(80, 10e-9).nstddev)
plt.plot(nstddev, label='nstddev(y) %.4f' % (nstddev.mean()))
#plt.plot(y.map(EWM(80, 1e-9).add_get_std), label='ewstd(y)')
#plt.plot((noise/noise.ewm(40).mean().ewm(40).mean()).ewm(80).std(), label='std(noise)')
#
#f = y.ewm(10).mean().ewm(30).mean()
#plt.plot((f-f.ewm(40).mean().ewm(40).mean()).ewm(200).std(), label='std(f)')
#plt.plot(f, label='f')
plt.legend()
plt.grid()
plt.show()
