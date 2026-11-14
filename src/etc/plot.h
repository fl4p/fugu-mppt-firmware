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
#include <new>
#include <string>
#include <utility>
#include <vector>

#include "logging.h"

// Points of one sweep (x, power) for the console chart. Heap block owned by core 0.
struct PlotPoints {
    static constexpr uint16_t Cap = 240;
    uint16_t n[2];
    std::pair<float, float> p[2][Cap]; // [0] = (Vin, P), [1] = (duty, P)
};

// Sweep chart. The RT core only appends into a block that core 0 allocated before the sweep and
// frees after plotting, so the RT side never touches the heap; without a block it drops the points.
struct Plot {
    std::atomic<PlotPoints *> collecting{nullptr}; // RT appends here
    std::atomic<PlotPoints *> handed{nullptr};     // stopped collecting, waiting for plot()

    // core 0: provide an empty block if there is none
    void begin() {
        if (collecting.load(std::memory_order_acquire)) return;
        auto b = new(std::nothrow) PlotPoints;
        if (!b) return;
        b->n[0] = b->n[1] = 0;
        PlotPoints *none = nullptr;
        if (!collecting.compare_exchange_strong(none, b, std::memory_order_acq_rel)) delete b;
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

    __attribute__((cold)) static void _plotSeries(std::pair<float, float> *pts, uint16_t &n, const char *label) {
#if !ASCII_PLOT_DISABLED
        auto *const end = pts + n;
        std::sort(pts, end);

        if (n < 3) {
            n = 0;
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

        n = 0;

        printf_mux("\r\n");
        ascii::Asciichart(series).height(16).Plot([](const std::string &line) {
            printf_mux("%s\r\n", line.c_str());
        });

        decltype(series)().swap(series); // clear() & shrink_to_fit

        UART_LOG("  P|%s     %.3g .. %.3g\n\n\n", label, minX, maxX);
#endif
    }

    // core 0: chart and free the handed-over block
    void plot() {
        auto b = handed.load(std::memory_order_acquire);
        if (!b) return;
        try {
            _plotSeries(b->p[0], b->n[0], "V");
            _plotSeries(b->p[1], b->n[1], "D");
        } catch (const std::exception &e) {
            ESP_LOGE("plot", "Error: %s", e.what());
        }
        handed.store(nullptr, std::memory_order_release);
        delete b;
    }
};
