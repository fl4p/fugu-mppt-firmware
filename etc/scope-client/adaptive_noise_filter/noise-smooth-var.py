import pandas as pd
from matplotlib import pyplot as plt

y = pd.read_csv('iout_50hz_inverter_noise.csv')['0']

n_free = y.rolling(40).mean().rolling(40).mean()
n = y - n_free


def filt(y, span):
    return y.rolling(span).mean().rolling(span).mean().rolling(span).mean()


pd.DataFrame(
    [(span, filt(y, span).std()) for span in [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 24, 32, 48, 64, 128]]).set_index(
    0).plot()

plt.semilogx()
plt.semilogy()
plt.grid()
plt.show()
