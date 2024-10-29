#ifndef INCLUDE_ASCII_STYLE_H_
#define INCLUDE_ASCII_STYLE_H_
#include "color.h"

namespace ascii {
class Style {
public:
  Style()
      : fg_(Foreground::From(Color::RESET)),
        bg_(Background::From(Color::RESET)), decorations_({}) {}

  Style &fg(Foreground fg) {
    fg_ = fg;
    return *this;
  }

  Style &bg(Background bg) {
    bg_ = bg;
    return *this;
  }

  Style &AddDecoration(Decoration decoration) {
    decorations_.push_back(decoration);
    return *this;
  }

  friend std::ostream &operator<<(std::ostream &os, const Style &val) {
    for (auto &item : val.decorations_) {
      os << item;
    }
    os << Foreground(val.fg_) << Background(val.bg_);
    return os;
  }

private:
  Foreground fg_;
  Background bg_;
  std::vector<Decoration> decorations_;
};

} // namespace ascii
#endif // !INCLUDE_ASCII_STYLE_H_