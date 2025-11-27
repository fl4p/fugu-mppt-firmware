import math

try:
    from .ewm import EWMA, EWM
except ImportError:
    from ewm import EWMA, EWM


class AdaptiveLengthFilter:

    def __init__(self, initial_span=5, max_span=200, target_err=0.95):
        self.span = initial_span
        self.target_err = target_err
        self.ewmas = [
            EWMA(self.span),
            EWMA(self.span),
            # EWMA(self.span),
        ]
        self._noise_ewmas = [
            EWMA(self.span),
            EWMA(self.span),
            # EWMA(self.span),
        ]
        self._dc_ewmas = [
            EWMA(self.span * 20),
            EWMA(self.span * 20),
            # EWMA(self.span),
        ]
        self.ewm_sig = EWM(initial_span * 8, 1e-9, True)
        self.ewm_noise = EWM(initial_span * 8, 1e-9)
        self.max_span = max_span
        self._span_update = 0
        self._last_x = math.nan
        self.noise_level = math.nan
        self.signal_level = math.nan

    def add(self, x):

        def apply_ewmas(x, ewmas):
            for ewm_ in ewmas:
                ewm_.add(x)
                x = ewm_.y
            return x

        #xi = x
        x = apply_ewmas(x, self.ewmas)
        #dc = apply_ewmas(x, self._dc_ewmas)

        self.ewm_sig.add(x)
        # self.ewm_noise.add(x - self.ewmas[0].value + dc)
        self.ewm_noise.add(x)

        if self._span_update == 0:
            self.signal_level = self.ewm_sig.nstddev()
            self.noise_level = self.ewm_noise.nstddev()
            nstd = self.noise_level / (self.signal_level+1e-6)
            if math.isfinite(nstd):
                self.span *= 1.1 if nstd > self.target_err else 0.9
            self.span = round(min(max(1, self.span), 200), 2)

            for ewm in self.ewmas:
                ewm.update_span(self.span)
            #for ewm in self._noise_ewmas:
            #    ewm.update_span(self.span)

            #for ewm in self._dc_ewmas:
            #    ewm.update_span(4000)
            self.ewm_sig.update_span(self.span * 8)
            self.ewm_noise.update_span(self.span * 8)
            self._span_update = round(self.span + .5) * 16
        else:
            self._span_update -= 1

        return x
        # return noise
        # return x
