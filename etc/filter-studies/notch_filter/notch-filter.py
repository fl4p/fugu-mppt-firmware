"""
Real-time IIR Notch filter.
It remembers the state on subsequent calls so it can filter frames of a signal stream.
Useful to filter e.g. 50 Hz or 60 Hz noise.

"""

from scipy import signal



class NotchIIR:
    def __init__(self, fs, f0, Q=30.0):
        """

        :param fs: Sampling frequency
        :param f0: frequency to remove
        :param Q: Quality factor
        """
        self.b, self.a = signal.iirnotch(f0, Q, fs)
        self.zi = signal.lfilter_zi(self.b, self.a)

    def __call__(self, x):
        y, self.zi = signal.lfilter(self.b, self.a, x, zi=self.zi)
        return y


if __name__ == "__main__":
    import numpy as np
    import math
    import matplotlib.pyplot as plt

    fs = 450.0  # Sample frequency (Hz)
    f0 = 50.0  # Frequency to be removed from signal (Hz)

    y = np.array([1 + 0.1 * math.sin(2 * math.pi * i / fs * f0) for i in range(1000)])

    notch = NotchIIR(fs, f0, Q=30.0)

    # split signal into frames and apply notch filter
    y_filt = np.array([])
    for b in np.split(y, 10):
        y_filt = np.concatenate([y_filt, notch(b)])

    # .. or filter signal in a single frame (same result):
    y_filt2 = NotchIIR(fs, f0, Q=30.0)(y)

    plt.plot(y)
    plt.plot(y_filt)
    plt.plot(y_filt2)
    plt.show()