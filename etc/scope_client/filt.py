import pandas as pd
from matplotlib import pyplot as plt

s = pd.read_csv('iout.csv')['0']

def plot(s, **kwargs):
    s.plot(marker='.', drawstyle="steps-post", markersize=3, linewidth=1, **kwargs)

def med3(a,b,c):
    return max(min(a,b),min(c,max(a,b)))

def med5(a,b,c,d,e):
    f=max(min(a,b),min(c,d))
    g=min(max(a,b),max(c,d))
    return med3(e,f,g)

def blank_spikes(s):
    s = s.copy()
    y = s.values
    med3 = s.rolling(5).median().values
    in_spike = 0
    for i in range(len(s)):
        if med3[i] and (y[i] - med3[i]) / med3[i] > (0.1 if in_spike else 0.2):
            y[i] = med3[i]
            in_spike = 1
        else:
            y[i] = y[i]
            in_spike = 0
    return s


plot(s)
plot(s.rolling(5).median(), label='med5')
plot(s.rolling(5).apply(lambda w:med5(*w.values)))

import pandas.testing
pd.testing.assert_series_equal(s.rolling(5).apply(lambda w:med5(*w.values)), s.rolling(5).median())
#plot(blank_spikes(s), label='spike')


plt.legend()
plt.show()


""
