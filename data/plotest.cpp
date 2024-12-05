// g++ -std=c++17  -fsanitize=address -fsanitize=undefined data/plotest.cpp  && ./a.out
#include <vector>

#include "../src/asciichart/ascii.h"

inline int geti() {
    static int i = std::ios_base::xalloc();
    return i;
}

std::ostream &add_one(std::ostream &os) {
    os.iword(geti()) = 1;
    return os;
}

std::ostream &add_none(std::ostream &os) {
    os.iword(geti()) = 0;
    return os;
}

int main() {

    std::vector<float> series{
            87.78,

            87.78,
            87.78,
            87.78,
            87.78,
            87.78,
            87.78,
            90.09,
            90.09,
            90.09,
            90.09,
            90.09,
            90.09,
            90.09,
            92.70,
            92.70,
            92.70,
            92.70,
            92.70,
            92.70,
            92.70,
            92.70,
            95.26,
            95.26,
            95.26,
            95.26,
            95.26,
            95.26,
            95.26,
            97.39,
            97.39,
            97.39,
            97.39,
            97.39,
            97.39,
            97.39,
            99.62,
            99.62,
            99.62,
            99.62,
            99.62,
            99.62,
            99.62,
            102.24,
            102.24,
            102.24,
            102.24,
            102.24,
            102.24,
            102.24,
            104.28,
            104.28,
            104.28,
            104.28,
            104.28,
            104.28,
            104.28,
            107.06,
            107.06,
            107.06,
            107.06,
            107.06,
            107.06,
            107.06,
            109.38,
            109.38,
            109.38,
            109.38,
            109.38,
            109.38,
            109.38,
            111.71,
            111.71,
            111.71,
            111.71,
            111.71,
            111.71,
            111.71,
            113.83,
            113.83,
            113.83,
            113.83,
            113.83,
            113.83,
            113.83,
            116.21,
            116.21,
            116.21,
            116.21,
            116.21,
            116.21,
            116.21,
            118.90,
            118.90,
            118.90,
            118.90,
            118.90,
            118.90,
            118.90,
            105.12,
            105.12,
            105.12,
            105.12,
            105.12,
            105.12,
            105.12,
            69.69,
            69.69,
            69.69,
            69.69,
            69.69,
            69.69,
            69.69,
            31.16,
            31.16,
            31.16,
            31.16,
            31.16,
            31.16,
            31.16,
    };

    auto a = series[0.0];

    ascii::Asciichart asciichart(std::vector<std::vector<float>>{series});
    auto screen = asciichart.height(18).type(ascii::Asciichart::LINE).Plot();

    /*for(auto row = 0; row < screen.n0; ++row) {
        std::stringstream ss;
        for(auto col = 0; col < screen.n1; ++col) {
            std::cout << screen[row][col];
        }
        std::cout  << ascii::Decoration::From(ascii::Decoration::RESET) << "\n";
    }*/


    for (auto &line: screen) {
        for (auto &item: line) {
            std::cout << item;
        }

        std::cout << ascii::Decoration::From(ascii::Decoration::RESET) << "\n";
    }


}