#pragma once

#ifndef ASCII_PLOT_DISABLED
#define ASCII_PLOT_DISABLED 0
#endif

#if !ASCII_PLOT_DISABLED
#include "asciichart/ascii.h"
#endif

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "logging.h"

struct Series {
    std::vector<std::pair<float, float> > vec;

    uint16_t expectedLen;

    Series(uint16_t expectedLen) : expectedLen(expectedLen) {
    }

    void add(float x, float y, float xMax) {
        if (vec.empty() or std::fabs(vec.back().first - x) > (xMax / (expectedLen * 0.9f))) {
            if (vec.size() >= expectedLen) vec.pop_back();
            vec.emplace_back(x, y);
        }
    }

    void reserve() {
        vec.reserve(expectedLen);
    }

    void clear() {
        decltype(vec)().swap(vec);
    }
};

struct Plot {
    //typedef std::vector<std::pair<float, float>> Ser;
    Series pointsU{240};
    Series pointsD{240};

    __attribute__((cold)) static void _plotSeries(Series &ser, const std::string &label) {
#if !ASCII_PLOT_DISABLED
        auto &points(ser.vec);
        std::sort(points.begin(), points.end());

        if (points.size() < 3) {
            ser.clear();
            ESP_LOGI("plot", "Not enough data to plot %s", label.c_str());
            return;
        }


        std::vector<float> series;

        int bins = 100;

        auto minX = points.begin()->first, maxX = points.back().first;
        auto binW = (maxX - minX) / bins;

        ESP_LOGI("mppt", "Grouping %u %s points (%.2f,%.2f)~(%.2f,%.2f) into %d bins, binW=%.3f", points.size(),
                 label.c_str(),
                 minX, points.begin()->second, maxX, points.back().second, bins, binW);

        auto it = points.begin();
        float y = it->second;
        for (int i = 0; i < bins; ++i) {
            auto x = minX + i * binW;
            int n = 0;
            float ya = 0;
            while (it != points.end() && it->first < x + binW * 0.5f) {
                ya += it->second;
                n++;
                ++it;
            }

            if (n) y = ya / n;
            else {
                if (it != points.end()) {
                    //TODO  interpolate
                    //y = y
                }
            }
            ESP_LOGD("plot", "bin %i x=%.2f n=%i y=%.2f,", i, x, n, y);
            series.push_back(y);
        }

        ser.clear();

        printf_mux("\r\n");
        ascii::Asciichart(series).height(16).Plot([](const std::string &line) {
            printf_mux("%s\r\n", line.c_str());
        });

        decltype(series)().swap(series); // clear() & shrink_to_fit

        UART_LOG("  P|%s     %.3g .. %.3g\n\n\n", label.c_str(), minX, maxX);
#endif
    }

    void plot() {
        try {
            _plotSeries(pointsU, "V");
            _plotSeries(pointsD, "D");
        } catch (const std::exception &e) {
            ESP_LOGE("plot", "Error: %s", e.what());
        }
    }

    void reserve() {
        pointsD.clear();
        pointsD.reserve();
        pointsU.clear();
        pointsU.reserve();
    }
};
