#ifndef INCLUDE_ASCII_ASCII_H_
#define INCLUDE_ASCII_ASCII_H_

#include <cmath>
#include <codecvt>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "color.h"
#include "constants.h"
#include "text.h"

namespace ascii {
class Asciichart {
public:
  enum Type {
    LINE = 0, // line
    CIRCLE    // circle
  };

public:
  explicit Asciichart(const std::vector<float> &series)
      : type_(LINE), height_(kDoubleNotANumber), min_(kDoubleInfinity),
        max_(kDoubleNegInfinity), offset_(3), legend_padding_(10),
        basic_width_of_label_(0), show_legend_(false) {
    InitSeries(series);
    InitStyles();
    InitSymbols();
  }

  explicit Asciichart(const std::vector<std::vector<float>> &series)
      : type_(LINE), height_(kDoubleNotANumber), min_(kDoubleInfinity),
        max_(kDoubleNegInfinity), offset_(3), legend_padding_(10),
        basic_width_of_label_(0), show_legend_(false) {
    InitSeries(series);
    InitStyles();
    InitSymbols();
  }

  // For associating a text label with each series
  explicit Asciichart(
      const std::unordered_map<std::string, std::vector<float>> &series)
      : type_(LINE), height_(kDoubleNotANumber), min_(kDoubleInfinity),
        max_(kDoubleNegInfinity), offset_(3), legend_padding_(10),
        basic_width_of_label_(0), show_legend_(false) {
    InitSeries(series);
    InitStyles();
    InitSymbols();
  }

  /// Set type of chart.
  Asciichart &type(Type type) {
    type_ = type;
    return *this;
  }

  /// Set height of chart.
  Asciichart &height(float height) {
    height_ = height;
    return *this;
  }

  /// Set colors of chart.
  Asciichart &styles(const std::vector<Style> &styles) {
    styles_ = styles;
    return *this;
  }

  /// Set min of chart.
  /// This value will be override if larger than true min value.
  Asciichart &min(float min) {
    min_ = min;
    return *this;
  }

  /// Set max of chart.
  /// This value will be override if less than true max value.
  Asciichart &max(float max) {
    max_ = max;
    return *this;
  }

  /// Set offset of label from axis.
  Asciichart &offset(size_t offset) {
    offset_ = offset;
    return *this;
  }

  /// Set padding between legend and label.
  Asciichart &legend_padding(size_t padding) {
    legend_padding_ = padding;
    return *this;
  }

  /// If show legend.
  Asciichart &show_legend(bool show) {
    show_legend_ = show;
    return *this;
  }

  /// Set symbols used to plot.
  Asciichart &symbols(const std::map<std::string, std::string> &symbols) {
    symbols_ = symbols;
    return *this;
  }

  /// Generate this chart.
  std::vector<std::vector<Text>> Plot() {
      return PlotLineChart();
  }

private:
  std::map<std::string, std::string> symbols_;
  std::unordered_map<std::string, std::vector<float>> series_;
  std::vector<Style> styles_;

  Type type_;
  float height_;
  float min_;
  float max_;
  float offset_;
  size_t legend_padding_;
  size_t basic_width_of_label_;

  bool show_legend_;

  void InitSeries(const std::vector<float> &series) {
    series_["series 0"] = series;
  }

  void InitSeries(const std::vector<std::vector<float>> &series) {
    unsigned n = 0;
    for (const auto &s : series) {
      series_["series " + std::to_string(n++)] = s;
    }
  }

  void InitSeries(
      const std::unordered_map<std::string, std::vector<float>> &series) {
    series_ = series;
  }

  void InitStyles() {
    styles_ = {
        Style().fg(Foreground::From(Color::RED)),
        Style().fg(Foreground::From(Color::CYAN)),
        Style().fg(Foreground::From(Color::MAGENTA)),
        Style().fg(Foreground::From(Color::YELLOW)),
        Style().fg(Foreground::From(Color::WHITE)),
        Style().fg(Foreground::From(Color::BRIGHT_WHITE)),
    };
  }

  void InitSymbols() {
    switch (type_) {
    case LINE:
      symbols_ = {{"empty", " "}, {"center", "┼"},  {"axis", "┤"},
                  {"c1", "╶"},    {"c2", "╴"},      {"parellel", "─"},
                  {"down", "╰"},  {"up", "╭"},      {"ldown", "╮"},
                  {"lup", "╯"},   {"vertical", "│"}};
    case CIRCLE:
      break;
    default:
      break;
    }
  }

  /// Writes each character of a string into its respective cell in the
  /// `screen` array, starting from the upper left, `row` and `col`.
  void PutString(std::vector<std::vector<Text>> &screen, const std::string &str,
                 const Style &style, unsigned row, unsigned col) {
    for (unsigned i = 0; i < str.length(); i++) {
      if (str[i] == '\n') {
        row += 1;
      } else {
        screen[row][col + i] = Text(str.substr(i, 1), style);
      }
    }
  }

  std::string FormatLabel(int x) {
    std::stringstream ss;
    ss << std::setw(show_legend_ ? legend_padding_ + basic_width_of_label_
                                 : basic_width_of_label_)
       << std::setfill(' ') << std::setprecision(2);
    ss << x;
    return ss.str();
  }

  std::string Print(const std::vector<std::vector<Text>> &screen) {
    std::stringstream os;
    for (auto &line : screen) {
      for (auto &item : line) {
        os << item;
      }
      os << "\n";
    }
    return os.str();
  }

    std::vector<std::vector<Text> > PlotLineChart() {
    // 1. calculate min and max
    for (auto &label_trace_pair : series_) {
      for (auto &item : label_trace_pair.second) {
        min_ = std::min(item, min_);
        max_ = std::max(item, max_);
      }
    }

    // 2. calaculate range
    auto range = max_ - min_;
    if (range == 0) range = 1;

    // make basic padding as size of str(max)
    basic_width_of_label_ = std::max(std::to_string((int)max_).length(),
                                     std::to_string((int)min_).length());

    // 3. width and height
    int width = 0;
    for (auto &label_trace_pair : series_) {
      width = std::max(width, (int)label_trace_pair.second.size());
    }

    int legend_cols = 0, legend_rows = 0;
    if (show_legend_) {
      // determine width and height of legend, add to offset.
      for (auto &label_trace_pair : series_) {
        legend_rows++;
        legend_cols =
            std::max(legend_cols, (int)label_trace_pair.first.length());
      }
    }

    auto offset = offset_ + legend_cols;

    width += offset;

    if (std::isnan(height_)) {
      height_ = range;
    }

    // extend the height of the plot if we need more rows to display the
    // legend than what the range of the data requires.
    height_ = std::max((float)legend_rows, height_);

    // calculate ratio using height and range
    auto ratio = height_ / range;

    int min2 = std::round(min_ * ratio);
    int max2 = std::round(max_ * ratio);

    // 4. rows and cols of this chart
    auto rows = max2 - min2;
    auto cols = width;

    if (rows == 0) rows = 1;

    // 5. initialize chart using empty str
    std::vector<std::vector<Text>> screen(
        rows + 1, std::vector<Text>(cols, symbols_["empty"]));

    // 6. axis + labels
    for (float y = min2; y <= max2; y++) {
      auto label = FormatLabel(std::round(min_ + (y - min2) * range / rows));
      // vertical reverse
      screen[rows - (y - min2)][legend_cols] =
          Text(label, Style().fg(Foreground::From(Color::BLUE)));
      screen[rows - (y - min2)][offset - 1] =
          Text((y == 0) ? symbols_["center"] : symbols_["axis"],
               Style().fg(Foreground::From(Color::CYAN)));
    }

    if (show_legend_) {
      // 7. Legend
      {
        unsigned j = 0;
        for (auto &label_trace_pair : series_) {
          auto style = styles_[j % styles_.size()];
          PutString(screen, label_trace_pair.first, style, j++, 0);
        }
      }
    }

    // 8. Content
    {
      unsigned j = 0;
      for (auto &label_trace_pair : series_) {
        auto &trace = label_trace_pair.second;
        auto style = styles_[j++ % styles_.size()];
        auto y0 = std::round(trace[0] * ratio) - min2;
        // vertical reverse
        screen[rows - y0][offset - 1] = Text(symbols_["center"], style);

        for (size_t i = 0; i < trace.size() - 1; i++) {
          auto y0 = std::round(trace[i] * ratio) - min2;
          auto y1 = std::round(trace[i + 1] * ratio) - min2;

          if (y0 == y1) {
            screen[rows - y0][i + offset] = Text(symbols_["parellel"], style);
          } else {
            screen[rows - y1][i + offset] =
                Text(y0 > y1 ? symbols_["down"] : symbols_["up"], style);
            screen[rows - y0][i + offset] =
                Text(y0 > y1 ? symbols_["ldown"] : symbols_["lup"], style);
            auto from = std::min(y0, y1);
            auto to = std::max(y0, y1);
            for (size_t y = from + 1; y < to; y++) {
              screen[rows - y][i + offset] = Text(symbols_["vertical"], style);
            }
          }
        }
      }
    }

    return (screen);
  }

};
} // namespace ascii
#endif
