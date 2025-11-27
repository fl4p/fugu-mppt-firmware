import math
from ewm import EWMA, EWM
import pandas as pd
from matplotlib import pyplot as plt
from anf import AdaptiveLengthFilter

y = pd.read_csv('iout_50hz_inverter_noise.csv')['0']
#y = pd.read_csv('../../notch-filter/vout_50hz_inverter.csv')['0']
y = pd.read_csv('vin1.csv')['0']
#y2 = pd.read_csv('vin.csv')['0']
#y3 = pd.read_csv('iout_50hz_inverter_noise.csv')['0']
#y3 = pd.read_csv('../vout_filt.csv')['0']
y3 = pd.read_csv('../china-inverter-1kw-load/iout.csv')['0']

y3 = pd.read_csv('../data/pv8A-inv500W/iout.csv')['0']
y3 = pd.read_csv('../data/var-inverter/iout_452hz.csv')['0']
y3 = pd.read_csv('../data/heater500w-off-on/iout_454hz.csv')['0']

y = pd.concat([y3], axis=0)

#import notch_filter.notch
from notch import NotchIIR
y = pd.Series(NotchIIR(452, 100, 20.0)(y))


anf = AdaptiveLengthFilter(target_err=0.2)
p = []
for x in y.values:
    y = anf.add(x)
    p.append(dict(x=x, err=anf.noise_level, sig=anf.signal_level, nsr=anf.noise_level/anf.signal_level, span=anf.span, f=y))

df = pd.DataFrame(p)
a = df.plot(subplots=True, layout=(len(df.columns), 1))

a[1][0].semilogy()
plt.tight_layout()
plt.show()

