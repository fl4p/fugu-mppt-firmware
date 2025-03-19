import math

import pandas as pd
from matplotlib import pyplot as plt

y = pd.read_csv('iout_50hz_inverter_noise.csv')['0']

y = pd.read_csv('../notch-filter/vout_50hz_inverter.csv')['0']

y = pd.read_csv('vin.csv')['0']

y = pd.concat([y, y], axis=0)

n_free = y.rolling(40).mean().rolling(40).mean()
n = y - n_free

from ewm import EWMA, EWM


class AdaptiveLengthFilter:

    def __init__(self, initial_span=5, max_span=200, target_err=0.01):
        self.span = initial_span
        self.target_err = target_err
        self.ewmas = [
            EWMA(self.span),
            EWMA(self.span),
            # EWMA(self.span),
        ]
        self.ewm = EWM(initial_span * 8, 1e-9)
        self.max_span = max_span
        self._span_update = 0

    def add(self, x):
        for ewm in self.ewmas:
            ewm.add(x)
            x = ewm.y

        self.ewm.add(x)

        if self._span_update == 0:
            nstd = self.ewm.nstddev()
            if math.isfinite(nstd):
                if self.span < 10:
                    self.span += 1 if nstd > self.target_err else -1
                else:
                    self.span *= 1.1 if nstd > self.target_err else 0.9
                self.span = min(max(1, self.span), 200)

            for ewm in self.ewmas:
                ewm.update_span(self.span)
            self.ewm.update_span(self.span * 8)
            self._span_update = round(self.span + .5) * 16
        else:
            self._span_update -= 1

        return x


anf = AdaptiveLengthFilter(target_err=0.0015)
p = []
for x in y.values:
    y = anf.add(x)
    p.append(dict(err=anf.ewm.nstddev(), span=anf.span, f=y))

pd.DataFrame(p).plot()

plt.show()
