from scipy import signal


class NotchIIR:
    def __init__(self, fs, f0, Q=30.0):
        self.b, self.a = signal.iirnotch(f0, Q, fs)
        self.zi = signal.lfilter_zi(self.b, self.a)

    def __call__(self, x):
        y, self.zi = signal.lfilter(self.b, self.a, x, zi=self.zi)
        return y
