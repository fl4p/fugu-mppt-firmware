# Cutting OTA flash work by making the image layout stable across builds

Measured 2026-09-08 on `fugu-firmware`, esp32s3, IDF 5.5.1.

## The problem

Delta OTA already shrinks what crosses the BLE link (1.75 MB -> 27 KB patch, transfer
36.6 s -> 11.3 s). After that the push is **device-bound**: ~14.7 s of the ~19.5 s wall
clock is flash erase + program, because the receiver rewrites the whole 1.77 MB image.

The delta encoder sees through a shift; **flash cannot** — a shifted sector is a changed
sector. And a trivial edit shifts almost everything:

| consecutive builds, one added log line | value |
|---|---|
| image size change | **+48 B** |
| 4 KB sectors that must be erased+written | **420 / 433 (97 %)** |
| bytes actually differing | 89.4 % |

Two causes, both layout, neither related to image size:

1. **`libmain.a` is scattered through `.flash.text`.** ldgen emits the catch-all
   `*(... .text ...)` first and every *explicitly mapped* archive after it
   (`tools/ldgen/ldgen/generation.py`: `get_node_output_commands` runs before
   `sorted(self.children)`). With no mapping of its own, this firmware's code lands inside
   the catch-all in link order — spread from `0x420015f4` to `0x4211eff4` with **937 KB of
   stable IDF code interleaved**. Growing any app function shifts every IDF function after it.

2. **One 187 KB merged string pool near the start of `.flash.rodata`.** GCC emits string
   literals into `SHF_MERGE|SHF_STRINGS` sections (`.rodata.<fn>.str1.4`); ld coalesces
   *all* of them in an output section into a single blob placed at its first contributor —
   here `libesp_app_format.a(esp_app_desc.c.obj)` at `0x3c140120`, size `187,842`. The blob
   is immune to `EXCLUDE_FILE`, so a linker-fragment placement cannot reach it. Adding one
   string literal to app code grew that early blob by 12 bytes and shifted ~70 % of rodata.

## The fix

`main/linker.lf` maps the two volatile archives explicitly, which puts them **after** the
catch-all (not "at the end": mapped archives are sorted lexically, so `libesp-ota-ble.a`
lands *before* `libmain.a`, and `libnewlib.a`/`libspi_flash.a`/`libxtensa.a` a few KB
after it. A future explicitly-mapped archive sorting after `libmain.a` would sit after it
too. The ordering is deterministic in ldgen but is not a documented compatibility
guarantee), and `-fno-merge-constants` on the app component keeps its literals out of the
shared pool so the fragment can place them last too.

```
[mapping:fugu_main_last]
archive: libmain.a
entries:
    * (default);
        text -> flash_text ALIGN(4),
        rodata -> flash_rodata ALIGN(4)
```

`ALIGN(4)` is load-bearing but free: an entry whose target equals its basis' target is
**not significant** and emits nothing (`generation.py:86`), so without a flag the mapping
silently does nothing. The image is already 4-aligned there.

`.flash.rodata` growth does not push `.flash.text`: the DROM segment is padded to the next
64 KB MMU page (`.flash_rodata_dummy`, `. = ALIGN(0x10000) + 0x20`).

**But the margin is nearly gone, and `-fno-merge-constants` is what ate it.** All 14,352 B
of its cost lands in rodata, so (esptool `image_info`, DROM len vs the 0x5fff0 ceiling):

| build | DROM len | slack below the page boundary |
|---|---|---|
| `linker.lf` only | 0x5bfd4 | **16,412 B** |
| `linker.lf` + `-fno-merge-constants` | 0x5f7e4 | **2,060 B** |

~2 KB is a few log strings. When it is exhausted, esptool advances IROM by a whole 64 KB
page: one update churns nearly every IROM sector *and* costs ~64 KB of the 83.6 KB
partition headroom, leaving ~20 KB. It recovers on the next build, but it is a cliff, not
a slope. **This is the strongest argument against adopting `-fno-merge-constants` as it
stands** -- the fragment alone buys 53 % at *negative* image cost and keeps the full
16 KB margin.

## Result

Same source edit, same toolchain, builds byte-reproducible:

| build | image | +delta | sectors rewritten | differing bytes |
|---|---|---|---|---|
| as shipped | 1,773,040 | +48 B | **420 / 433 (97.0 %)** | 89.4 % |
| `+ linker.lf` | 1,771,904 | +32 B | 230 / 433 (53.1 %) | 26.0 % |
| `+ linker.lf + -fno-merge-constants` | 1,786,256 | +32 B | **147 / 437 (33.6 %)** | 14.9 % |

### Reading the "differing bytes" column

That column is a **positional** comparison (`A[i] != B[i]`) of a shifted image, not a
measure of how much content changed. On the as-shipped pair:

| comparison | bytes matching |
|---|---|
| positional | 10.6 % |
| one global realignment (+24) | 61.7 % |
| each 4 KB block given its own offset | **97.6 %** |

That last row is a best-of-many-shifts score over a -8..+116 step-4 window, so it also
rewards zero runs, `0xff` padding and coincidental matches. It shows the differences are
**explainable by local displacement**; it is an upper bound on preserved content, not a
measurement of it. The patch size below is the trustworthy figure. bsdiff encodes the entire difference in 69-95 KB
(4-5 % of the image). The shift is a *patchwork* — per-block best offsets are +24 (249
blocks), +4 (53), +12 (45), +16 (36), +40 (31), 0 (18), plus one trailing sector too
short to classify — because the string pool, rodata
and text each grew independently and every downstream region carries a different
accumulated sum. That is why no single realignment fixes it. The 10.6 % that still matches
positionally is zero runs, `0xff` padding and repeated opcode bytes, not preserved content.

For the same pair, `detools` (bsdiff + heatshrink, the on-device codec) expresses the whole
difference in **56,406 B (3.18 %)** -- it can say "the next 900 KB is the old bytes, moved
by 24", which the flash controller cannot. That is the same range as the two benchmarked
delta pushes (69,294 B / 3.9 % and 95,202 B / 5.4 %), so those runs are representative of a
one-line edit. With the layout fix the patch drops further, to **35,434 B (1.98 %)**.

Patch size is nearly irrelevant to the wall clock: a 37 % bigger patch (69,294 -> 95,202 B)
cost 2 % more transfer time (11.27 -> 11.51 s), because the transfer reconstructs and writes
the full 1.77 MB either way.

The **sector** column is the one that predicts OTA cost: flash must erase and rewrite any
sector whose contents moved by even one byte.

Image cost: the fragment alone is **-1,136 B**; `-fno-merge-constants` adds **+14,352 B**,
for a net **+13,216 B (+0.75 %)**. Partition free space after: 83.6 KB (4.6 %). The app partition keeps 85 KB (5 %) free.

The residual 147 sectors are almost entirely `libmain` shifting **itself** — its own
216 KB of text and 128 KB of rodata, which is irreducible without per-function padding
(rejected: align=64 costs +295 KB and does not fit). One unexplained 32 KB `libnet80211`
block at file `0x117000` still churns; not chased.


## Erase granularity: 64 KB blocks, almost certainly

The receiver's `eraseAhead()` erases in 64 KB chunks. A skip-identical check makes 4 KB
sector erase tempting, since the sector *count* (147) is far below the sectors inside the
touched *blocks* (19 x 16 = 304). But per-byte, sector erase is much less efficient:
64 KB block erase measures 302 kB/s, while 4 KB sector erase at a typical 60 ms is
68 kB/s -- **4.4x worse**. That wipes out the advantage.

Setting the two schedules equal gives a crisp threshold:

| variant | 64 KB schedule | sector granularity wins only if 4 KB erase is under |
|---|---|---|
| fragment only (230 sectors, 23 blocks) | 12.53 s | **34 ms** |
| fragment + no-merge (147 sectors, 19 blocks) | 10.35 s | **50 ms** |

Typical NOR 4 KB sector erase is 45-60 ms (spec maximum several hundred), and this part's
64 KB erase measured 217 ms against a 150-300 ms typical band, i.e. mid-band. So sector
granularity is marginal at best for the 147-sector case and clearly worse for 230.

**Provisional, from datasheet-typical figures -- 4 KB erase on this part has NOT been
measured.** That measurement is the one open question, and it is now a pass/fail rather
than an exploration: erase a run of 4 KB sectors in the inactive OTA slot, time it, and
compare against 50 ms. If it is above, keep the 64 KB schedule and the projection below
stands; if it is below, sector granularity is worth building.

Blocked 2026-09-08: `flu` was in use by another session (leg T of
`plans/BENCH-flu-E-separation.md`, heat gun on D9), so the board was not taken.

## What this does NOT do on its own

**Nothing, until the receiver skips identical sectors.** `esp_ota_write` currently writes
all 1.77 MB unconditionally, so a stable layout saves zero seconds by itself — and
skip-if-identical saves almost nothing today (6.9-7.9 % of sectors match the inactive slot).
The two are only useful as a pair. With both, and A/B alternation meaning the destination
holds the image from two pushes ago:

The receiver already owns its erase schedule -- `eraseAhead()` drives
`esp_partition_erase_range` directly and passes only a 64 KB head to `esp_ota_begin`
(`esp-ota-ble/src/ota_ble.cpp:283-298, :429`) -- so a compare-before-erase check is
reachable without fighting `esp_ota_begin`'s bulk erase. What is *not* settled is the cost:

| | now | both |
|---|---|---|
| sectors written | 97 % | 34 % (147 of 437) |
| 64 KB blocks touched | 28 of 28 | **19 of 28** |
| erase + write, 64 KB granularity | 14.7 s | **~10.4 s** (`19 x (0.217 + 65536/200000)`) |
| erase + write, 4 KB granularity | | program 3.0 s + erase **unmeasured** |
| delta push wall | 19.5 s | **~15 s at 64 KB granularity** |

The 147 changed sectors are scattered across 19 of the 28 64 KB erase blocks, so block-
granularity erase recovers only ~4 s, not the ~10 s a naive `147/437` scaling suggests.
4 KB sector erase would program in 3.0 s, but **4 KB erase timing was never measured** --
only the 217 ms/64 KB block figure. Until it is, "~5 s flash / ~10 s wall" is unverified
and this table's 64 KB row is the defensible one.

The A/B destination holds the image from *two* pushes ago, not one; the 147/437 rate is
measured for consecutive builds, and the n-vs-n-2 rate is not demonstrated here.

## Status

Build-level prototype only. **Not flashed, not run on hardware.** Builds are
byte-reproducible (rebuilding `lf1` from the same source produced a byte-identical image).
Reviewed adversarially by codex 2026-09-08; the DROM-margin and erase-cost corrections
above came from that review. The layout change alters
every address in the image, so the first push after adopting it rewrites the full image
regardless, and the OTA base-image cache must be re-seeded.
