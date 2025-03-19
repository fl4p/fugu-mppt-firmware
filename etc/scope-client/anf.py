import math

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
        self._noise_ewmas = [
            EWMA(self.span),
            EWMA(self.span),
            # EWMA(self.span),
        ]
        self._dc_ewmas = [
            EWMA(self.span *20),
            EWMA(self.span * 20),
            # EWMA(self.span),
        ]
        self.ewm = EWM(initial_span * 8, 1e-9)
        self.max_span = max_span
        self._span_update = 0
        self._last_x = 0

    def add(self, x):

        def apply_ewmas(x, ewmas):
            for ewm in self.ewmas:
                ewm.add(x)
                x = ewm.y
            return x

        xi = x
        x = apply_ewmas(x, self.ewmas)
        dc = apply_ewmas(x,  self._dc_ewmas)


        # extract noise (subtract useful signal)
        noise = (xi - x) #+ dc (x- self._last_x)
        #self._last_x

        for ewm in self._noise_ewmas:
            ewm.add(noise)
            noise = ewm.y

        self.ewm.add(x)

        if self._span_update == 0:
            nstd = self.ewm.nstddev()
            if math.isfinite(nstd):
                #                if self.span < 10:
                #                   self.span += 1 if nstd > self.target_err else -1
                #               else:
                self.span *= 1.1 if nstd > self.target_err else 0.9
            self.span = round(min(max(1, self.span), 200), 2)

            for ewm in self.ewmas:
                ewm.update_span(self.span)
            for ewm in self._noise_ewmas:
                ewm.update_span(self.span)

            for ewm in self._dc_ewmas:
                ewm.update_span(self.span*20)
            self.ewm.update_span(self.span * 8)
            self._span_update = round(self.span + .5) * 16
        else:
            self._span_update -= 1

        return noise
        return x
