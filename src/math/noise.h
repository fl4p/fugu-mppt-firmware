
#include "statmath.h"

class AdaptiveNoiseFilter {
    static constexpr int PASSES = 3;
    static constexpr uint16_t SPAN_MAX = 200;
    static constexpr float SIG_VAR_REG = 1e-7;


    float targetNSR = NAN;
    EWMA_nPass<PASSES, float> ewma{1};
    //EWMA_nCh_nPass<2, 1,



public:
    EWM<false, float> ewmN{1};
    EWM<true, float> ewmS{1};
    float span = 1;
private:
    uint16_t _span_update = 0;
public:
    void begin(uint16_t initialSpan, float targetNoise_) {
        span = initialSpan;
        targetNSR = targetNoise_ * targetNoise_; // square so we don't need to take sqrt during filter len update
        ewma.updateSpan(span);
        ewmN.updateSpan(span * 8);
        ewmS.updateSpan(span * 8);
    }

    float_t add(float_t x) {
        ewma.add(x);
        const auto y = ewma.get();
        ewmS.add(y);
        ewmN.add(y);

        if (_span_update == 0) {
            auto nsr = ewmN.nvar() / (ewmS.nvar() + SIG_VAR_REG);
            if (std::isfinite(nsr)) {
                bool ex = nsr > targetNSR;
                span *= ex ? 1.1f : 0.9f;
                span = std::min(std::max(1.0f, span), (float)SPAN_MAX);
                ewma.updateSpan(span);
                ewmN.updateSpan(span * 8.f);
                ewmS.updateSpan(span * 8.f);
                _span_update = ceilf(span) * 16.0f;
            }
        } else {
            --_span_update;
        }

        return y;
    }

};