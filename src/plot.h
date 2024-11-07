#pragma once

#include "asciichart/ascii.h"
#include "logging.h"

struct Series {
    std::vector<std::pair<float, float>> vec;

    uint16_t expectedLen;

    Series(uint16_t expectedLen) : expectedLen(expectedLen) {

    }

    void add(float x, float y, float xMax) {
        if (vec.empty() or abs(vec.back().first - x) > (xMax / (expectedLen * 0.9f))) {
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

    static void _plotSeries(Series &ser, const std::string &label) {
        auto &points(ser.vec);
        std::sort(points.begin(), points.end());

        if (points.size() < 3) {
            ser.clear();
            ESP_LOGI("plot", "Not enough data to plot %s", label.c_str());
            return;
        }


        std::vector<float> series;

        int bins = 100; // 120 causes mem issue already, maybe reduce plot height
        // nootice that sizeof(Text) = 96, so will alloc: alloc 19*123*96 = 224352

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

        std::vector<std::vector<ascii::Text>> screen{};
        {
            ascii::Asciichart asciichart(std::vector<std::vector<float>>{series});
            decltype(series)().swap(series); // clear() & shrink_to_fit
            screen = asciichart.height(16).Plot();
        }

        /* for(auto row = 0; row < screen.n0; ++row) {
             std::stringstream ss;
             for(auto col = 0; col < screen.n1; ++col) {
                 ss << screen[row][col];
             }
             ss << ascii::Decoration::From(ascii::Decoration::RESET) << "\n";
             UART_LOG(ss.str().c_str());
         } */

        for (auto &line: screen) {
            std::stringstream ss;

            for (auto &item: line) {
                ss << item;
            }
            ss << ascii::Decoration::From(ascii::Decoration::RESET);
            ss << "\n";

            printf_mux(ss.str().c_str());
        }

        std::stringbuf buffer;
        std::ostream os(&buffer);
        os << "  P|" << label << "     " << std::setprecision(3) << minX << " .. " << maxX << "\n\n\n";
        //UART_LOG_ASYNC(buffer.str().c_str());
        UART_LOG(buffer.str().c_str());

        //UART_LOG(sc.c_str());
        //UART_LOG("%.1fV .. %.1fV", minX, maxX);

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