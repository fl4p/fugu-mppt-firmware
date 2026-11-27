/* Native MTP2 pmini encoder.  See pmini_codec.h for the MIT attribution. */

#include "pmini_codec.h"

#include <algorithm>
#include <array>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

constexpr std::uint32_t M32 = 0xffffffffu;
constexpr std::size_t MAX_ROWS = 128;
constexpr std::size_t MAX_COLS = 64;
constexpr std::size_t SYMS_CAP = 8192;
constexpr std::size_t FTAB_CAP = 2048; // uint16 entries (freq,cumulative pairs)

constexpr std::uint8_t T_CONST = 0;
constexpr std::uint8_t T_RAW = 1;
constexpr std::uint8_t T_D1 = 2;
constexpr std::uint8_t T_DICT = 4;
constexpr std::uint8_t T_SPLIT = 5;
constexpr std::uint8_t T_SAME = 6;

constexpr std::array<std::uint16_t, 16> THR = {
    33486, 34969, 36517, 38133, 39822, 41585, 43426, 45348,
    47356, 49453, 51642, 53929, 56316, 58810, 61413, 64132};

inline unsigned bit_length(std::uint32_t x) {
    if (!x) return 0;
#if defined(__GNUC__) || defined(__clang__)
    return 32u - static_cast<unsigned>(__builtin_clz(x));
#else
    unsigned n = 0;
    while (x) { ++n; x >>= 1; }
    return n;
#endif
}

inline std::uint32_t zigzag(std::uint32_t d) {
    return (d & 0x80000000u) ? ((d << 1) ^ M32) : (d << 1);
}

inline std::pair<unsigned, unsigned> bucket(std::uint32_t u) {
    if (u < 4) return {u, 0};
    const unsigned e = bit_length(u) - 3;
    return {static_cast<unsigned>((e << 2) + (u >> e)), e};
}

inline unsigned l16(std::uint32_t x) {
    const unsigned b = bit_length(x) - 1;
    std::uint32_t m;
    if (b <= 15) m = x << (15 - b);
    else m = x >> (b - 15);
    unsigned k = 0;
    while (k < THR.size() && m >= THR[k]) ++k;
    return (b << 4) + k;
}

struct BitWriter {
    std::vector<std::uint8_t> bytes;
    std::uint32_t acc = 0;
    unsigned nbits = 0;

    void write(std::uint32_t v, unsigned n) {
        while (n > 16) {
            write(v & 0xffffu, 16);
            v >>= 16;
            n -= 16;
        }
        if (!n) return;
        acc |= (v & ((1u << n) - 1u)) << nbits;
        nbits += n;
        while (nbits >= 8) {
            bytes.push_back(static_cast<std::uint8_t>(acc));
            acc >>= 8;
            nbits -= 8;
        }
    }

    void finish() {
        if (nbits) {
            bytes.push_back(static_cast<std::uint8_t>(acc));
            acc = 0;
            nbits = 0;
        }
    }
};

struct Stream {
    std::size_t symStart = 0;
    std::size_t freqStart = 0;
};

struct Plan {
    std::uint32_t cost = std::numeric_limits<std::uint32_t>::max();
    std::uint8_t type = T_RAW;
    std::uint32_t value = 0;
    std::array<std::uint32_t, 32> dict{};
    std::size_t nd = 0;
};

struct GapKey {
    std::vector<std::uint32_t> gaps;
    std::size_t column = 0;
};

struct Block {
    BitWriter bits;
    std::vector<std::uint8_t> symbols;
    std::vector<std::uint16_t> freqs;
    std::vector<Stream> streams;
    std::size_t columns = 0;

    void reset() {
        bits = BitWriter{};
        symbols.clear();
        freqs.clear();
        streams.clear();
        columns = 0;
    }
};

class Encoder {
public:
    Encoder(const std::uint32_t* input, std::size_t nrows, std::size_t ncols)
        : input_(input), rows_(nrows), cols_(ncols), column_(nrows), work_(nrows),
          gaps_(nrows), foreground_(nrows) {
        // All large objects are heap-backed vectors.  Reserve caps make a
        // successful encode allocation-stable while scaling ordinary shards
        // with their actual dimensions.  The vectors grow up to the format
        // bounds only for a genuinely worst-case block.
        const std::size_t cells = nrows * ncols;
        block_.symbols.reserve(std::min(SYMS_CAP, std::max<std::size_t>(nrows, cells * 2)));
        block_.freqs.reserve(std::min(FTAB_CAP, std::max<std::size_t>(272, ncols * 64)));
        block_.streams.reserve(std::min<std::size_t>(256, std::max<std::size_t>(2, ncols * 2)));
        block_.bits.bytes.reserve(std::min<std::size_t>(4 * MAX_ROWS + 2048,
                                                        std::max<std::size_t>(256, cells / 2 + 512)));
        keys_.reserve(ncols);
    }

    void encode(std::string& dst, std::size_t start) {
        dst.append("MTP2", 4);
        put_varint(dst, rows_);
        put_varint(dst, cols_);
        if (!rows_) {
            finish_crc(dst, start);
            return;
        }
        for (std::size_t c = 0; c < cols_; ++c) {
            for (std::size_t r = 0; r < rows_; ++r)
                column_[r] = input_[r * cols_ + c];
            encode_column(c, dst);
        }
        close_block(dst);
        finish_crc(dst, start);
    }

private:
    const std::uint32_t* input_;
    std::size_t rows_;
    std::size_t cols_;
    std::vector<std::uint32_t> column_, work_, gaps_, foreground_;
    std::vector<GapKey> keys_;
    Block block_;

    static void put_varint(std::string& out, std::size_t value) {
        while (value > 0x7f) {
            out.push_back(static_cast<char>(0x80u | (value & 0x7fu)));
            value >>= 7;
        }
        out.push_back(static_cast<char>(value));
    }

    static void put16(std::string& out, std::size_t x) {
        out.push_back(static_cast<char>(x & 0xff));
        out.push_back(static_cast<char>((x >> 8) & 0xff));
    }

    static std::uint32_t crc32(const std::string& s, std::size_t start) {
        std::uint32_t crc = 0xffffffffu;
        for (std::size_t i = start; i < s.size(); ++i) {
            const unsigned char ch = static_cast<unsigned char>(s[i]);
            crc ^= ch;
            for (unsigned i = 0; i < 8; ++i)
                crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
        }
        return crc ^ 0xffffffffu;
    }

    static void finish_crc(std::string& out, std::size_t start) {
        const std::uint32_t crc = crc32(out, start);
        out.push_back(static_cast<char>(crc));
        out.push_back(static_cast<char>(crc >> 8));
        out.push_back(static_cast<char>(crc >> 16));
        out.push_back(static_cast<char>(crc >> 24));
    }

    static std::uint32_t cost(const std::vector<std::uint32_t>& src, std::size_t n) {
        std::array<std::uint32_t, 136> h{};
        std::uint32_t offsetBits = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const auto [s, e] = bucket(src[i]);
            ++h[s];
            offsetBits += e;
        }
        std::uint32_t c = (offsetBits + 3) << 4;
        const unsigned ln = l16(static_cast<std::uint32_t>(n));
        int prev = -1;
        for (unsigned s = 0; s < h.size(); ++s) {
            if (!h[s]) continue;
            c += h[s] * (ln - l16(h[s]));
            c += ((bit_length(static_cast<std::uint32_t>(s - prev)) << 1) - 1) << 4;
            c += ((bit_length(h[s]) << 1) - 1) << 4;
            prev = static_cast<int>(s);
        }
        return c;
    }

    static void scan(const std::vector<std::uint32_t>& src, std::size_t n,
                     std::size_t& nd, std::uint32_t& mn,
                     std::uint32_t& majority, std::size_t& majorityCount,
                     std::array<std::uint32_t, 32>& distinct) {
        nd = 0;
        mn = src[0];
        majority = src[0];
        int balance = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const std::uint32_t x = src[i];
            mn = std::min(mn, x);
            if (balance == 0) { majority = x; balance = 1; }
            else if (x == majority) ++balance;
            else --balance;
            if (nd <= 32) {
                std::size_t j = 0;
                while (j < nd && distinct[j] != x) ++j;
                if (j == nd) {
                    if (nd < 32) distinct[nd] = x;
                    ++nd;
                }
            }
        }
        majorityCount = 0;
        for (std::size_t i = 0; i < n; ++i)
            if (src[i] == majority) ++majorityCount;
    }

    static void subtract(const std::vector<std::uint32_t>& src, std::size_t n,
                         std::uint32_t mn, std::vector<std::uint32_t>& dst) {
        for (std::size_t i = 0; i < n; ++i) dst[i] = src[i] - mn;
    }

    static void delta(const std::vector<std::uint32_t>& src, std::size_t n,
                      std::vector<std::uint32_t>& dst) {
        std::uint32_t prev = src[0];
        for (std::size_t i = 1; i < n; ++i) {
            const std::uint32_t d = src[i] - prev;
            prev = src[i];
            dst[i - 1] = zigzag(d);
        }
    }

    static void sort_distinct(std::array<std::uint32_t, 32>& d, std::size_t n) {
        std::sort(d.begin(), d.begin() + static_cast<std::ptrdiff_t>(n));
    }

    static void dictionary(const std::vector<std::uint32_t>& src, std::size_t n,
                           const std::array<std::uint32_t, 32>& d, std::size_t nd,
                           std::vector<std::uint32_t>& dst) {
        for (std::size_t i = 0; i < n; ++i) {
            std::size_t j = 0;
            while (j < nd && d[j] != src[i]) ++j;
            dst[i] = static_cast<std::uint32_t>(j);
        }
    }

    Plan make_plan(const std::vector<std::uint32_t>& src, std::size_t n,
                   std::size_t nd, std::uint32_t mn) {
        Plan p;
        subtract(src, n, mn, work_);
        p.cost = cost(work_, n) + (9u + bit_length(mn)) * 16u;
        p.type = T_RAW;
        p.value = mn;
        if (n > 1) {
            const std::uint32_t first = src[0];
            delta(src, n, work_);
            const std::uint32_t c = cost(work_, n - 1) +
                                    (9u + bit_length(first)) * 16u;
            if (c < p.cost) {
                p.cost = c;
                p.type = T_D1;
                p.value = first;
            }
        }
        if (nd <= 32) {
            std::array<std::uint32_t, 32> d{};
            // `scan`'s distinct values are supplied by the caller through
            // the first nd entries of the scratch array below.
            for (std::size_t i = 0; i < nd; ++i) d[i] = distinct_[i];
            sort_distinct(d, nd);
            dictionary(src, n, d, nd, work_);
            std::uint32_t c = cost(work_, n) +
                (3u + 2u * bit_length(static_cast<std::uint32_t>(nd)) - 1u +
                 6u + bit_length(d[0])) * 16u;
            for (std::size_t i = 1; i < nd; ++i)
                c += (2u * bit_length(d[i] - d[i - 1]) - 1u) * 16u;
            if (c < p.cost) {
                p.cost = c;
                p.type = T_DICT;
                p.value = static_cast<std::uint32_t>(nd);
                p.nd = nd;
                for (std::size_t i = 0; i < nd; ++i) p.dict[i] = d[i];
            }
        }
        return p;
    }

    std::array<std::uint32_t, 32> distinct_{};

    void write_bits(BitWriter& bw, std::uint32_t x, unsigned n) { bw.write(x, n); }

    void gamma(BitWriter& bw, std::uint32_t x) {
        x += 1;
        const unsigned b = bit_length(x);
        write_bits(bw, 0, b - 1);
        write_bits(bw, 1, 1);
        write_bits(bw, x & ((1u << (b - 1)) - 1u), b - 1);
    }

    void big(BitWriter& bw, std::uint32_t x) {
        const unsigned b = bit_length(x);
        write_bits(bw, b, 6);
        if (b) write_bits(bw, x, b);
    }

    void latents(const std::vector<std::uint32_t>& src, std::size_t n) {
        BitWriter& bw = block_.bits;
        std::array<std::uint32_t, 136> hist{};
        for (std::size_t i = 0; i < n; ++i) ++hist[bucket(src[i]).first];
        write_bits(bw, 2, 2);
        const std::size_t ft0 = block_.freqs.size();
        std::array<std::uint8_t, 136> dense{};
        unsigned nu = 0;
        std::uint32_t total = 0;
        unsigned maxIndex = 0;
        std::uint32_t maxFreq = 0;
        int last = -1;
        for (unsigned s = 0; s < hist.size(); ++s) {
            if (!hist[s]) continue;
            std::uint32_t f = (hist[s] << 12) / static_cast<std::uint32_t>(n);
            if (!f) f = 1;
            block_.freqs.push_back(static_cast<std::uint16_t>(f));
            block_.freqs.push_back(0);
            total += f;
            if (f > maxFreq) { maxFreq = f; maxIndex = nu; }
            dense[s] = static_cast<std::uint8_t>(nu++);
            last = static_cast<int>(s);
        }
        if (nu == 0) throw std::bad_alloc();
        const std::int64_t adjusted = static_cast<std::int64_t>(block_.freqs[ft0 + 2 * maxIndex]) +
                                      4096 - total;
        if (adjusted < 1 || adjusted > 65535) throw std::runtime_error("pmini frequency underflow");
        block_.freqs[ft0 + 2 * maxIndex] = static_cast<std::uint16_t>(adjusted);
        gamma(bw, nu - 1);
        int prev = -1;
        for (int s = 0; s <= last; ++s) {
            if (hist[static_cast<unsigned>(s)]) {
                gamma(bw, static_cast<std::uint32_t>(s - prev - 1));
                prev = s;
            }
        }
        for (int s = 0; s < last; ++s)
            if (hist[static_cast<unsigned>(s)]) gamma(bw, hist[static_cast<unsigned>(s)] - 1);
        std::uint32_t cum = 0;
        for (unsigned i = 0; i < nu; ++i) {
            block_.freqs[ft0 + 2 * i + 1] = static_cast<std::uint16_t>(cum);
            cum += block_.freqs[ft0 + 2 * i];
        }
        block_.streams.push_back({block_.symbols.size(), ft0});
        for (std::size_t i = 0; i < n; ++i) {
            const auto [s, e] = bucket(src[i]);
            block_.symbols.push_back(dense[s]);
            if (e) write_bits(bw, src[i] & ((1u << e) - 1u), e);
        }
    }

    void emit_plan(const Plan& plan, const std::vector<std::uint32_t>& src, std::size_t n) {
        BitWriter& bw = block_.bits;
        write_bits(bw, plan.type, 3);
        if (plan.type == T_DICT) {
            gamma(bw, static_cast<std::uint32_t>(plan.nd - 1));
            big(bw, plan.dict[0]);
            distinct_ = plan.dict;
            for (std::size_t i = 1; i < plan.nd; ++i)
                gamma(bw, plan.dict[i] - plan.dict[i - 1] - 1);
            dictionary(src, n, plan.dict, plan.nd, work_);
            latents(work_, n);
        } else if (plan.type == T_RAW) {
            big(bw, plan.value);
            subtract(src, n, plan.value, work_);
            latents(work_, n);
        } else {
            big(bw, plan.value);
            delta(src, n, work_);
            latents(work_, n - 1);
        }
    }

    std::ptrdiff_t find_key(const std::vector<std::uint32_t>& gaps, std::size_t m) const {
        for (std::size_t i = 0; i < keys_.size(); ++i)
            if (keys_[i].gaps.size() == m && keys_[i].gaps ==
                std::vector<std::uint32_t>(gaps.begin(), gaps.begin() + static_cast<std::ptrdiff_t>(m)))
                return static_cast<std::ptrdiff_t>(i);
        return -1;
    }

    void put_key(const std::vector<std::uint32_t>& gaps, std::size_t m, std::size_t column,
                 std::ptrdiff_t found) {
        if (found >= 0) { keys_[static_cast<std::size_t>(found)].column = column; return; }
        if (keys_.size() >= MAX_COLS) return;
        keys_.push_back({std::vector<std::uint32_t>(gaps.begin(), gaps.begin() + static_cast<std::ptrdiff_t>(m)), column});
    }

    void split(const std::vector<std::uint32_t>& src, std::size_t n, std::uint32_t bg,
               std::size_t& m) {
        m = 0;
        std::ptrdiff_t prev = -1;
        for (std::size_t i = 0; i < n; ++i) {
            if (src[i] != bg) {
                foreground_[m] = src[i];
                gaps_[m] = static_cast<std::uint32_t>(static_cast<std::ptrdiff_t>(i) - prev - 1);
                prev = static_cast<std::ptrdiff_t>(i);
                ++m;
            }
        }
    }

    void encode_column(std::size_t ci, std::string& dst) {
        // A conservative preflight lets us close a block before emitting a
        // column.  It covers SPLIT's gap and foreground streams and its two
        // frequency tables, so no rollback buffer is needed.
        if (block_.columns && (block_.symbols.size() + 3 * rows_ > SYMS_CAP ||
                               block_.freqs.size() + 2 * 136 > FTAB_CAP ||
                               block_.bits.bytes.size() > 0xffffu))
            close_block(dst);

        std::size_t nd = 0, majorityCount = 0;
        std::uint32_t mn = 0, majority = 0;
        std::array<std::uint32_t, 32> d{};
        scan(column_, rows_, nd, mn, majority, majorityCount, d);
        distinct_ = d;
        if (nd == 1) {
            write_bits(block_.bits, T_CONST, 3);
            big(block_.bits, column_[0]);
            ++block_.columns;
            return;
        }
        Plan whole = make_plan(column_, rows_, nd, mn);
        Plan fgPlan;
        std::size_t m = 0;
        std::ptrdiff_t found = -1;
        std::size_t ref = 0;
        std::uint8_t splitType = 0;
        if (majorityCount * 2 > rows_ && rows_ - majorityCount >= 2) {
            split(column_, rows_, majority, m);
            std::size_t fnd = 0, fcnt = 0;
            std::uint32_t fmn = 0, fmaj = 0;
            std::array<std::uint32_t, 32> fd{};
            scan(foreground_, m, fnd, fmn, fmaj, fcnt, fd);
            distinct_ = fd;
            fgPlan = make_plan(foreground_, m, fnd, fmn);
            distinct_ = d;
            const std::uint32_t base = (9u + bit_length(majority)) * 16u + fgPlan.cost;
            found = find_key(gaps_, m);
            if (found >= 0) {
                ref = keys_[static_cast<std::size_t>(found)].column;
                if (base + (2u * bit_length(static_cast<std::uint32_t>(ci - ref)) - 1u) * 16u < whole.cost)
                    splitType = T_SAME;
            }
            if (!splitType) {
                const std::uint32_t c = base + (2u * bit_length(static_cast<std::uint32_t>(m)) - 1u) * 16u + cost(gaps_, m);
                if (c < whole.cost) splitType = T_SPLIT;
            }
        }
        if (!splitType) {
            emit_plan(whole, column_, rows_);
        } else {
            write_bits(block_.bits, splitType, 3);
            big(block_.bits, majority);
            if (splitType == T_SPLIT) {
                gamma(block_.bits, static_cast<std::uint32_t>(m - 1));
                latents(gaps_, m);
                put_key(gaps_, m, ci, found);
            } else {
                gamma(block_.bits, static_cast<std::uint32_t>(ci - ref - 1));
            }
            emit_plan(fgPlan, foreground_, m);
        }
        ++block_.columns;
    }

    void close_block(std::string& out) {
        if (!block_.columns) return;
        block_.bits.finish();
        std::vector<std::uint8_t> rans;
        rans.reserve(block_.symbols.size() + 3);
        std::uint32_t x = 1u << 16;
        for (std::ptrdiff_t si = static_cast<std::ptrdiff_t>(block_.streams.size()) - 1; si >= 0; --si) {
            const std::size_t streamIndex = static_cast<std::size_t>(si);
            const Stream& st = block_.streams[streamIndex];
            const std::size_t start = st.symStart;
            const std::size_t end = (streamIndex + 1 < block_.streams.size())
                                        ? block_.streams[streamIndex + 1].symStart
                                        : block_.symbols.size();
            for (std::size_t p = end; p > start; ) {
                --p;
                const std::size_t q = st.freqStart + 2 * block_.symbols[p];
                const std::uint32_t f = block_.freqs[q];
                while (x >= (f << 12)) {
                    rans.push_back(static_cast<std::uint8_t>(x));
                    x >>= 8;
                }
                x = (x / f << 12) + (x % f) + block_.freqs[q + 1];
            }
        }
        if (!block_.symbols.empty()) {
            rans.push_back(static_cast<std::uint8_t>(x));
            rans.push_back(static_cast<std::uint8_t>(x >> 8));
            rans.push_back(static_cast<std::uint8_t>(x >> 16));
        }
        if (block_.bits.bytes.size() > 0xffffu || rans.size() > 0xffffu || block_.columns > 0xffffu)
            throw std::runtime_error("pmini block overflow");
        out.append(reinterpret_cast<const char*>(block_.bits.bytes.data()), block_.bits.bytes.size());
        out.append(reinterpret_cast<const char*>(rans.data()), rans.size());
        put16(out, block_.bits.bytes.size());
        put16(out, rans.size());
        put16(out, block_.columns);
        block_.reset();
    }
};

} // namespace

bool pminiEncode(const std::uint32_t* rowMajorWords, std::size_t rows,
                 std::size_t cols, std::string& out) {
    if (!rows || !cols || (rows && !rowMajorWords) || rows > MAX_ROWS || cols > MAX_COLS)
        return false;
    if (rows && cols > std::numeric_limits<std::size_t>::max() / rows)
        return false;
    const std::size_t oldSize = out.size();
    const std::size_t reserveExtra = 16 + rows * cols + 1024;
    if (oldSize > std::numeric_limits<std::size_t>::max() - reserveExtra)
        return false;
    try {
        out.reserve(oldSize + reserveExtra);
        Encoder enc(rowMajorWords, rows, cols);
        enc.encode(out, oldSize);
        return true;
    } catch (...) {
        out.resize(oldSize);
        return false;
    }
}
