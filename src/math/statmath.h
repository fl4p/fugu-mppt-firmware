#pragma once

#include <cstdlib>
#include <limits>

template<class T>
inline T abs(T x) { return x < 0 ? -x : x; }

template<class float_t=float, class float_t_alpha=float_t>
class EWMA {
/**
 * Implement exponential weighted moving average
 */
    float_t_alpha alpha;
public:

    float_t y = std::numeric_limits<float_t>::quiet_NaN();

    explicit EWMA(uint32_t span) {
        updateSpan(span);
    }

    void updateSpan(uint32_t span) {
        alpha = (2.f / (float_t) (span + 1));
    }

    [[nodiscard]] uint32_t span() const { return uint32_t(2.f / alpha) - 1; }

    inline void add(float_t x) {
        if (unlikely(isnan(x))) return;
        if (unlikely(isnan(y))) y = x;
        y = (1 - alpha) * y + alpha * x;
    }

    inline const float_t &get() const { return y; }

    inline void reset() { y = std::numeric_limits<float_t>::quiet_NaN(); }
};

/**
 * Implement exponential weighted moving average and normalised standard deviation
 * @tparam float_t
 */
template<class float_t=float>
class EWM {
    float_t last_x = std::numeric_limits<float_t>::quiet_NaN();
public:
    static constexpr float regularisation = 0.001f;

    EWMA<float_t> avg, std;

    explicit EWM(uint32_t span) : avg(span), std(span) {}

    void updateSpan(uint32_t span) {
        avg.updateSpan(span);
        std.updateSpan(span);
    }

    auto span() const { return avg.span(); }

    void reset() {
        avg.reset();
        std.reset();
    }

    inline void add(float_t x) {
        avg.add(x);
        if constexpr (regularisation != 0) {
            x = abs(x) + regularisation;
        }
        if (likely(!isnan(last_x))) {
            float_t pct = (x - last_x) / last_x;
            if (likely(std::isfinite(pct)))
                std.add(pct * pct);
        }
        last_x = x;
    }
};

template<typename float_t=float>
class PctChange {
    float_t _last = NAN;

public:
    inline float next(float x) {
        float_t pct = (x - _last) / _last;
        _last = x;
        return pct;
    }
};

template<int span, typename float_t=float>
class EWMA_N {
    float_t y = std::numeric_limits<float_t>::quiet_NaN();
public:
    inline void add(float_t x) {
        constexpr auto alpha = (2.f / (float_t) (span + 1));
        if (unlikely(isnan(x))) return;
        if (unlikely(isnan(y))) y = x;
        y = (1 - alpha) * y + alpha * x;
    }

    inline float_t get() const { return y; }

    inline void reset() { y = std::numeric_limits<float_t>::quiet_NaN(); }
};


template<typename T>
inline bool greaterThan(const T &a, const T &b) { return a > b; }


// inspired by https://github.com/wjakob/tbb/blob/master/include/tbb/parallel_sort.h

template<typename T, bool (*comp)(const T &, const T &)>
inline T median_of_three(const T &a, const T &b, const T &c) {
    return comp(a, b) ? (comp(b, c) ? b : (comp(a, c) ? c : a))
                      : (comp(c, b) ? b : (comp(c, a) ? c : a));
}


// https://stackoverflow.com/a/6984153/2950527
template<class V>
inline V median(const V &a, const V &b, const V &c) {
    return max(min(a, b), min(c, max(a, b)));
}

template<class V>
inline V median(const V &a, const V &b, const V &c, const V &d, const V &e) {
    V f = max(min(a, b), min(c, d)); // discards lowest from first 4
    V g = min(max(a, b), max(c, d)); // discards biggest from first 4
    return median(e, f, g);
}

template<typename T>
class RunningMedian3 {
    T s1, s2;
    T _last;
public:
    RunningMedian3() { reset(); }

    inline T next(const T &s) {
        T m = median_of_three<T, greaterThan<T>>(s, s1, s2);
        s2 = s1;
        s1 = s;
        _last = m;
        return m;
    }

    inline T get() const {
        return _last;
    }

    void reset() {
        s1 = std::numeric_limits<T>::quiet_NaN();
        s2 = std::numeric_limits<T>::quiet_NaN();
        _last = std::numeric_limits<T>::quiet_NaN();
    }
};

template<typename T>
class RunningMedian5 {
    T b[5];
    T _last;
    uint8_t pos;
public:
    RunningMedian5() {
        reset();
        pos = 0;
    }

    inline const T &next(const T &s) {
        b[pos] = s;
        pos = (pos + 1) % 5;
        _last = median(b[0], b[1], b[2], b[3], b[4]);
        return _last;
    }

    inline const T &get() const {
        return _last;
    }

    void reset() {
        for (auto &s: b) {
            s = std::numeric_limits<T>::quiet_NaN();
        }
        _last = std::numeric_limits<T>::quiet_NaN();
    }
};

/*
template<typename RandomAccessIter, typename comp>
inline size_t median_of_three(const RandomAccessIter &array, size_t l, size_t m, size_t r)  {
    return comp(array[l], array[m]) ? ( comp(array[m], array[r]) ? m : ( comp( array[l], array[r]) ? r : l ) )
                                    : ( comp(array[r], array[m]) ? m : ( comp( array[r], array[l] ) ? r : l ) );
}

inline size_t pseudo_median_of_nine( const RandomAccessIterator &array, const quick_sort_range &range ) const {
    size_t offset = range.size/8u;
    return median_of_three(array,
                           median_of_three(array, 0, offset, offset*2),
                           median_of_three(array, offset*3, offset*4, offset*5),
                           median_of_three(array, offset*6, offset*7, range.size - 1) );

}
*/





struct MeanAccumulator {
    // TODO trapezoidal sum?
    float sum;
    //float max;
    uint32_t num;

    float getMean() const { return sum / num; }

    //float getMax() const {return max;}

    void clear() {
        sum = 0.f;
        num = 0;
        //max = std::numeric_limits<float>::lowest();
    }

    void add(float x) {
        assert(num < std::numeric_limits<decltype(num)>::max()); // overflow
        sum += x;
        //if (x > max) max = x;
        ++num;
    }

    float pop() {
        float m = getMean();
        clear();
        return m;
    }

    MeanAccumulator() { clear(); }
};


template<typename F=float, typename T=unsigned long, typename D=double>
class TrapezoidalIntegrator {
    const F timeFactor;
    const T maxDt;
    T lastTime;
    F lastX;
    D value;
public:
    explicit TrapezoidalIntegrator(float timeFactor = (1e-6f / 3600.f), T maxDt = 1e6) :
            timeFactor(timeFactor), maxDt(maxDt), lastTime{0}, lastX{0}, value{0} {}

    D get() const { return value; }

    void add(F x, T nowTime) {
        T dt = nowTime - lastTime;
        if (dt < maxDt)
            value += (D) ((lastX + x) * (F) 0.5 * (F) (dt * timeFactor));
        lastTime = nowTime;
        lastX = x;
    }

    void restore(D v) {
        assert(value == 0);
        if(isfinite(v))
            value = v;
    }
};

#include <deque>

template<typename float_t =float>
class RollingMean {
    std::deque<float_t> q;
    float_t sum_;

public:
    explicit RollingMean(size_t len) : q(len, 0), sum_(0) {}

    void add(float_t x) {
        sum_ += x - q.pop_front();
        q.push_back(x);
    }

    inline void mean() const { return sum_ / q.size(); }

    inline void sum() const { return sum_; }
};