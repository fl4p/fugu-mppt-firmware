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

### 4 KB sector erase is ~56 ms on this part, derived from a push already measured

The pass/fail above does not need the bench after all. Two independently measured numbers
on this same part pin it:

* **32.4 s of a 43.4 s push was inside `esp_ota_write`**, for a 1.76 MB image under
  `OTA_WITH_SEQUENTIAL_WRITES` -- which erases each 4 KB sector as the write pointer first
  crosses it, so that time is ~430 sector erases plus the programming
  (`esp-ota-ble/src/ota_ble.cpp:248-251`).
* **Programming runs at 190-205 kB/s**, measured separately and independent of chunk size
  (`esp-ota-ble/doc/2026-09-08-payload-transforms-benchmark.md:210`), so 1.76 MB of
  programming is ~8.5 s.

`(32.4 - 8.5) / 430` = **55.6 ms per 4 KB sector erase**. The programming rate's own 8 %
spread moves that by under 2 ms, so the figure is robust where it matters.

That is *above* the 50 ms threshold, so on the stated pass/fail the answer is "keep the
64 KB schedule". It is still derived rather than timed directly, and the `OTAB SKIP` line
the receiver now emits reports `erases` and `erase_ms` on every push, which measures it
outright the first time one runs.

## The other half: the receiver now skips identical sectors

A stable layout saves **zero seconds** by itself -- `esp_ota_write` wrote all 1.77 MB
regardless of what the slot already held. The two halves are only useful as a pair, and
before the layout fix the pairing was worthless in the other direction too: only 6.9-7.9 %
of sectors matched the inactive slot, so there was nothing to skip.

The receiver half now exists (`esp-ota-ble/src/ota_ble.cpp`, `OTA_BLE_SECTOR_SKIP`). It
passes `esp_ota_begin` a single erase sector -- the smallest request that still leaves
`need_erase == false` and hands every later erase to the module -- then buffers each 4 KB
sector of the reconstructed image, compares it against the slot, and erases and programs
only on a mismatch. Host-native tests cover it; **nothing has run on hardware.**

### What it is projected to save

At the 55.6 ms sector erase derived above, and 197 kB/s programming:

| | erase | program | flash work |
|---|---|---|---|
| today: erase the slot, write it all | 6.2 s | 8.5 s | **14.7 s** |
| skip identical, 4 KB granularity (built) | 8.2 s (147) | 3.1 s | **11.2 s** + 0.4 s compare reads |
| block granularity, rewrite dirty blocks | 4.2 s (19) | 6.3 s | **10.5 s** |

**The two post-fix rows are a tie**, and the one that was built is the marginally slower of
them -- at this erase time, trading 64 KB block erases for a third of the programming very
nearly cancels out. Both save ~3.5-4 s of a 19.5 s delta push. Do not describe sector
granularity as beating block granularity here; it does not.

Two things still argue for the sector-granular version that was built:

* **It scales with the quantity that is actually improving.** The 147 changed sectors are
  scattered across 19 of the 28 blocks, so block granularity is pinned near 10.5 s however
  much better the layout gets. Halve the changed sectors again and the sector-granular row
  falls to ~5.6 s while the block row barely moves.
* **It is the strategy that measures the number this whole section turns on.** Its
  `OTAB SKIP kept=.. wrote=.. erases=.. erase_ms=..` line reports the real sector erase time
  on every push. If 55.6 ms is an overestimate, the sector row falls and the block row does
  not.

### Two caveats on the 147/437 figure

The A/B destination holds the image from **two** pushes ago, not one. The 147/437 rate is
measured for consecutive builds; the n-vs-n-2 rate is not demonstrated here, and it can only
be worse.

The layout change alters every address in the image, so the **first** push after adopting it
rewrites the full image regardless.

## Status

Build-level prototype only. **Not flashed, not run on hardware.** Builds are
byte-reproducible (rebuilding `lf1` from the same source produced a byte-identical image).
Reviewed adversarially by codex 2026-09-08; the DROM-margin and erase-cost corrections
above came from that review. The OTA base-image cache must be re-seeded after adopting the
layout change.

The receiver-side skip is likewise **built and tested host-native, never run on hardware**:
`flu` and `fugu-rig` were held by another session all day (leg T of
`plans/BENCH-flu-E-separation.md`, heat gun on D9), so no board was taken. The first real
push is what turns the projection table above into measurements, and it reports the numbers
to do it with.
