#pragma once

#ifndef ASCII_PLOT_DISABLED
#define ASCII_PLOT_DISABLED 0
#endif

#if !ASCII_PLOT_DISABLED
#include "asciichart/ascii.h"
#endif

#include <algorithm>
#include <atomic>
#include <cmath>
#include <ctime>
#include <new>
#include <string>
#include <utility>
#include <vector>

#include "logging.h"

// Points of one sweep (x, power). Heap block owned by core 0.
struct PlotPoints {
    static constexpr uint16_t Cap = 120;
    uint16_t n[2];
    uint32_t endTimeS; // 0: unknown
    std::pair<float, float> p[2][Cap]; // [0] = (Vin, P), [1] = (duty, P)
};

// Sweep chart. The RT core only appends into a block that core 0 allocated before the sweep and
// keeps after plotting (x-ascending), so the RT side never touches the heap; without a block it
// drops the points.
struct Plot {
    std::atomic<PlotPoints *> collecting{nullptr}; // RT appends here
    std::atomic<PlotPoints *> handed{nullptr};     // stopped collecting, waiting for plot()
    PlotPoints *block = nullptr;                   // core 0

    ~Plot() { delete block; }

    // core 0: provide an empty block if none is collecting
    void begin() {
        if (collecting.load(std::memory_order_acquire) || handed.load(std::memory_order_acquire)) return;
        if (!block) block = new(std::nothrow) PlotPoints;
        if (!block) return;
        block->n[0] = block->n[1] = 0;
        block->endTimeS = 0;
        collecting.store(block, std::memory_order_release);
    }

    // core 0: last plotted sweep, nullptr while a sweep is collected or pending
    [[nodiscard]] const PlotPoints *last() const {
        return collecting.load(std::memory_order_acquire) || handed.load(std::memory_order_acquire) ? nullptr : block;
    }

    // RT: empty the block at a sweep start
    void restart() {
        if (auto b = collecting.load(std::memory_order_acquire)) b->n[0] = b->n[1] = 0;
    }

    static void addPoint(std::pair<float, float> *v, uint16_t &n, float x, float y, float xMax) {
        if (!n or std::fabs(v[n - 1].first - x) > (xMax / (PlotPoints::Cap * 0.9f))) {
            if (n >= PlotPoints::Cap) --n;
            v[n++] = {x, y};
        }
    }

    // RT
    void add(float u, float d, float power, float uMax) {
        auto b = collecting.load(std::memory_order_acquire);
        if (!b) return;
        addPoint(b->p[0], b->n[0], u, power, uMax);
        addPoint(b->p[1], b->n[1], d, power, 1.0f);
    }

    // RT: stop collecting; false when there is nothing for plot() (none collected or one still pending)
    bool handOver() {
        if (handed.load(std::memory_order_acquire)) return false;
        auto b = collecting.exchange(nullptr, std::memory_order_acq_rel);
        if (!b) return false;
        handed.store(b, std::memory_order_release);
        return true;
    }

    __attribute__((cold)) static void _plotSeries(std::pair<float, float> *pts, uint16_t n, const char *label) {
        auto *const end = pts + n;
        std::sort(pts, end);
#if !ASCII_PLOT_DISABLED
        if (n < 3) {
            ESP_LOGI("plot", "Not enough data to plot %s", label);
            return;
        }


        std::vector<float> series;

        int bins = 100;

        auto minX = pts->first, maxX = (end - 1)->first;
        auto binW = (maxX - minX) / bins;

        ESP_LOGI("mppt", "Grouping %u %s points (%.2f,%.2f)~(%.2f,%.2f) into %d bins, binW=%.3f", (unsigned) n,
                 label,
                 minX, pts->second, maxX, (end - 1)->second, bins, binW);

        auto it = pts;
        float y = it->second;
        for (int i = 0; i < bins; ++i) {
            auto x = minX + i * binW;
            int cnt = 0;
            float ya = 0;
            while (it != end && it->first < x + binW * 0.5f) {
                ya += it->second;
                cnt++;
                ++it;
            }

            if (cnt) y = ya / cnt;
            else {
                if (it != end) {
                    //TODO  interpolate
                    //y = y
                }
            }
            ESP_LOGD("plot", "bin %i x=%.2f n=%i y=%.2f,", i, x, cnt, y);
            series.push_back(y);
        }

        printf_mux("\r\n");
        ascii::Asciichart(series).height(16).Plot([](const std::string &line) {
            printf_mux("%s\r\n", line.c_str());
        });

        decltype(series)().swap(series); // clear() & shrink_to_fit

        UART_LOG("  P|%s     %.3g .. %.3g\n\n\n", label, minX, maxX);
#endif
    }

    // core 0: chart the handed-over block and keep it
    void plot() {
        auto b = handed.load(std::memory_order_acquire);
        if (!b) return;
        const auto now = time(nullptr);
        b->endTimeS = now > 1000000000 ? (uint32_t) now : 0;
        try {
            _plotSeries(b->p[0], b->n[0], "V");
            _plotSeries(b->p[1], b->n[1], "D");
        } catch (const std::exception &e) {
            ESP_LOGE("plot", "Error: %s", e.what());
        }
        handed.store(nullptr, std::memory_order_release);
    }
};
