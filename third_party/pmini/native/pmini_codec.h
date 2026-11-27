/*
 * Native MTP2 encoder, compatible with Fabian Schlieper's pmini codec.
 *
 * MIT License
 * Copyright (c) 2026 Fabian Schlieper
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// Encode row-major unsigned 32-bit words as an MTP2 pmini shard.  The words
// are interpreted as the exact field representation: signed integers and
// floating point values must be supplied as their 32-bit raw words.  The
// decoder obtains the schema separately, as it does for Python pmini.
//
// The function appends one complete shard to out on success.  On bad input
// (including zero rows or columns), allocation failure, or an internal encoding
// failure it returns false and out is unchanged.  Bounds are deliberately
// smaller than the Python format's MAX_ROWS so the firmware workspace remains
// bounded.
bool pminiEncode(const std::uint32_t* rowMajorWords,
                 std::size_t rows,
                 std::size_t cols,
                 std::string& out);
