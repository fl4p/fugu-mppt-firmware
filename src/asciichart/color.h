#ifndef INCLUDE_ASCII_COLOR_H_
#define INCLUDE_ASCII_COLOR_H_

#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

/// for detailed info, refered to
/// https://gist.github.com/fnky/458719343aabd01cfb17a3a4f7296797.
/// https://talyian.github.io/ansicolors/
namespace ascii {
enum class Color {
  BLACK = 30,
  RED = 31,
  GREEN = 32,
  YELLOW = 33,
  BLUE = 34,
  MAGENTA = 35,
  CYAN = 36,
  WHITE = 37,

  BRIGHT_BLACK = 90,
  BRIGHT_RED = 91,
  BRIGHT_GREEN = 92,
  BRIGHT_YELLOW = 93,
  BRIGHT_BLUE = 94,
  BRIGHT_MAGENTA = 95,
  BRIGHT_CYAN = 96,
  BRIGHT_WHITE = 97,

  RESET = -1,
};

class Color256 {
public:
  Color256(int id) : id_(id) {}
  int id() { return id_; }

private:
  int id_;
};

class RGB {
public:
  RGB(int r, int g, int b) : r_(r), g_(g), b_(b) {}
  int r() { return r_; }
  int g() { return g_; }
  int b() { return b_; }

private:
  int r_;
  int g_;
  int b_;
};

class Decoration {
public:
  enum Attribute {
    RESET = 0,
    BOLD = 1,
    DIM = 2,
    ITALIC = 3,
    UNDERLINE = 4,
    SLOWBLINK = 5,
    RAPIDBLINK = 6,
    REVERSED = 7,
    CONCEAL = 8,
    CROSSED = 9,

    NO_BOLD = 22,
    NO_DIM = 22,
    NO_ITALIC = 23,
    NO_UNDERLINE = 24,
    NO_BLINK = 25,
    NO_REVERSED = 27,
    NO_CONCEAL = 28,
    NO_CROSSED = 29
  };

  static Decoration From(Attribute attr) { return Decoration(attr); }
  friend std::ostream &operator<<(std::ostream &os, const Decoration &val) {
    os << "\x1b[" << static_cast<int>(val.attr_) << "m";
    return os;
  }

private:
  Decoration(Attribute attr) : attr_(attr) {}
  Attribute attr_;
};

class Foreground {
public:
  template <typename T> static Foreground From(T color) {
    return Foreground(Resolve(color));
  }

  friend std::ostream &operator<<(std::ostream &os, const Foreground &val) {
    os << val.color_;
    return os;
  }

private:
  Foreground(std::string color) : color_(color) {}
  std::string color_;

  static std::string Resolve(Color color) {
    if (color == Color::RESET) {
      return "\x1b[39m";
    }
    std::stringstream os;
    os << "\x1b[" << static_cast<int>(color) << "m";
    return os.str();
  }

  static std::string Resolve(RGB color) {
    std::stringstream os;
    os << "\x1b[38;2;" << color.r() << ";" << color.g() << ";" << color.b()
       << "m";
    return os.str();
  }

  static std::string Resolve(Color256 color) {
    std::stringstream os;
    os << "\x1b[38;5;" << color.id() << "m";
    return os.str();
  }
};

class Background {
public:
  template <typename T> static Background From(T color) {
    return Background(Resolve(color));
  }

  friend std::ostream &operator<<(std::ostream &os, const Background &val) {
    os << val.color_;
    return os;
  }

private:
  Background(std::string color) : color_(color) {}
  std::string color_;

  static std::string Resolve(Color color) {
    if (color == Color::RESET) {
      return "\x1b[49m";
    }
    std::stringstream os;
    os << "\x1b[" << 10 + static_cast<int>(color) << "m";
    return os.str();
  }

  static std::string Resolve(RGB color) {
    std::stringstream os;
    os << "\x1b[48;2;" << color.r() << ";" << color.g() << ";" << color.b()
       << "m";
    return os.str();
  }

  static std::string Resolve(Color256 color) {
    std::stringstream os;
    os << "\x1b[48;5;" << color.id() << "m";
    return os.str();
  }
};
} // namespace ascii
#endif