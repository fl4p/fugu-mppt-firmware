#ifndef INCLUDE_ASCII_TEXT_H_
#define INCLUDE_ASCII_TEXT_H_
#include "style.h"

inline int get_style_stream_iword() {
    static int i = std::ios_base::xalloc();
    return i;
}

namespace ascii {
class Text {
public:
  Text() {}
  Text(std::string text) : text_(text) {}
  Text(std::string text, Style style) : text_(text), style_(style) {}

  Text &style(Style style) {
    style_ = style;
    return *this;
  }

  Text &text(std::string text) {
    text_ = text;
    return *this;
  }

  std::string text() { return text_; }

  friend std::ostream &operator<<(std::ostream &os, const Text &val) {
      std::stringstream ss;
      ss << val.style_;
      std::hash<std::string> hasher;
      auto textStyleHash = (long)hasher(ss.str());
      auto &streamStyleHash = os.iword(get_style_stream_iword());
      if(textStyleHash != streamStyleHash) {
          os << ss.str();
          streamStyleHash = textStyleHash;
      }
      os << val.text_;
      // reset style: << Decoration::From(Decoration::RESET);
    return os;
  }

private:
  std::string text_;
  Style style_;
};
} // namespace ascii
#endif // !INCLUDE_ASCII_TEXT_H_
