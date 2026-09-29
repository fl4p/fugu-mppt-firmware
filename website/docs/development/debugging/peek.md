---
title: peek / peek-struct
sidebar_position: 3
---

*this document is an LLM generated placeholder*

# `peek` / `peek-struct` — Memory Inspection

Read arbitrary memory on a running device and (optionally) interpret it through the build's
DWARF debug info. Two layers:

- **Device** — `peek <addr> [len]` returns up to 256 raw bytes per request. Knows nothing about
  symbols or types.
- **Host** (`etc/fugu_console.py` + `etc/idf-devtools/peek_symbols.py`) — rewrites `peek <symbol>[.field…]`
  to a numeric address before sending; renders `peek-struct <obj>` by issuing one or more
  `peek` requests and decoding the byte image against the build ELF's DWARF.

The wire protocol contains no symbols and no type info — it is byte-oriented. All naming and
decoding lives client-side, against the same ELF that was flashed.

---

## 1. Device-side: `peek <addr> [len]`

### 1.1 Syntax

```
peek <addr> [len]
```

- `<addr>` — required. Parsed with `strtoul(s, &endp, 0)`: `0x…` (hex), `0…` (octal), or
  decimal. The entire token must be consumed; trailing junk is an error.
- `[len]` — optional, default `4`. Range `[1, 256]`. Decimal integer.

### 1.2 Memory-region validation

The (`addr`, `addr + len - 1`) interval must lie entirely in a single class of region:

| Region | Method | Constraints |
| --- | --- | --- |
| Internal RAM (DRAM/SRAM/RTC), DROM (flash-mapped const), external RAM (PSRAM) | `memcpy` | byte access; no alignment needed |
| IROM / IRAM (executable, instruction-bus only) | 32-bit volatile load | `addr % 4 == 0` and `len % 4 == 0` |
| Peripheral MMIO (S3 `0x60000000-0x600D1FFF`; ESP32 `0x3FF00000-0x3FF7FFFF`) | 32-bit volatile load | `addr % 4 == 0` and `len % 4 == 0`; not synchronised with the RT loop |

Acceptance only means the range lies in a known region, not that the read is harmless:

:::warning MMIO reads can crash or change state
Reading a register of a clock-gated peripheral faults the bus (panic and reboot, which stops conversion on
a converting device). FIFO, capture and `*_INT_ST` reads have side effects (UART/I2C FIFO pop, MCPWM
capture, interrupt-status latches).
:::

Addresses outside these classes (or that straddle them) are rejected with
`peek: 0x<addr> not safely readable`. Executable or MMIO ranges with non-aligned addr or len are rejected
with `peek: executable region needs 4-byte aligned addr+len` (or `mmio region …`).

### 1.3 Output

Two formats, chosen by `len`:

**Typed scalar** — `len ∈ {1, 2, 4, 8}`. One line:

```
peek 0x<addr> = 0x<value>
```

Width of `<value>` matches `len`: 2 / 4 / 8 / 16 hex digits. The value is the little-endian
integer read from the bytes (i.e. what `*(uintN_t*)addr` would return on the device).
Example: `peek 0x3fca7674 = 0x0000033a`.

**Hex+ASCII dump** — any other `len` in `[1, 256]`. One line per 16 bytes:

```
0x<addr>: aa bb cc dd ... 16 hex bytes ...  |....ASCII....|
```

- Address prefix `0x` followed by 8 hex digits and a colon.
- Up to 16 single-space-separated `%02x` byte tokens. Short final row pads with `   ` so the
  ASCII column aligns.
- ` |` then up to 16 ASCII chars (printable range `0x20..0x7e`, others rendered as `.`) then
  `|`.

Both modes terminate the command with the normal `OK: peek <addr> <len>` reply marker after
the data lines.

### 1.4 Failure modes

- `peek: expected <addr> [len]` — no arguments.
- `peek: invalid address '<s>'` — `strtoul` couldn't fully parse.
- `peek: len out of range (1..256)` — `len <= 0` or `len > 256`.
- `peek: executable region needs 4-byte aligned addr+len` — IROM/IRAM access without alignment.
- `peek: 0x<addr> not safely readable` — address class is unknown or mixed.

All failures emit `ERR: peek …` as the command's reply marker.

---

## 2. Host-side address resolution

The CLI rewrites the `<addr>` token before sending. Two forms are accepted by the wire — only
the second triggers a rewrite:

| Form | Action |
| --- | --- |
| `peek 0x<hex>[…] [len]` | passthrough |
| `peek <symbol>[.field…][+off] [len]` | resolve client-side, replace with `peek 0x<addr> <len>` |

A `<symbol>` matches the regex `[A-Za-z_][A-Za-z0-9_$:.]*`. Symbols are read once per session
from `nm -S --defined-only` over the build ELF (cached by `(path, mtime)`). C++ mangled
symbols (`_Z*`) also gain a demangled-alias entry when the demangled simple name is
unambiguous — so `setupCli` resolves to `_Z8setupCliv`.

### 2.1 Dotted member access

`<symbol>.<m1>.<m2>…` walks DWARF:

1. Find the variable's `DW_TAG_variable` DIE with a `DW_AT_type` attribute.
2. Strip type qualifiers (`DW_TAG_typedef`, `DW_TAG_const_type`, `DW_TAG_volatile_type`,
   `DW_TAG_restrict_type`, `DW_TAG_atomic_type`).
3. For each `.<mN>`: the current type must be a `DW_TAG_structure_type`,
   `DW_TAG_class_type`, or `DW_TAG_union_type`. BFS over `DW_TAG_member` children plus any
   `DW_TAG_inheritance` base classes; first match wins.
4. The final address is `nm[base] + Σ member_offsets`. The size hint is the leaf type's
   `DW_AT_byte_size`.

`DW_AT_data_member_location` is decoded for both constant forms (already an int) and a
`DW_OP_plus_uconst` exprloc (ULEB128).

### 2.2 Default length

When `[len]` is omitted, the host fills it in:

- Symbol with `nm` size > 0: use `min(nm_size, 256)`.
- Dotted member: use the leaf field's `DW_AT_byte_size`.
- Otherwise: `4` (matches the device default, hits typed-print).

---

## 3. Host-only: `sym <pattern>`

Lists ELF symbols matching `<pattern>` — never sent to the device.

```
sym <substring>
sym /<regex>/
```

- Bare pattern → case-insensitive substring match.
- `/.../` → Python regex.
- Output: one line per match, `  0x<addr> <size> <name>`, sorted by largest-first then name.
- Default limit 40 hits; refine pattern when truncated.

---

## 4. Host-only: `peek-struct <obj>[.field…] [depth]`

Render bytes at the target's address as a DWARF-typed field tree.

### 4.1 Resolution

Same address+type walk as §2.1. The leaf type must be a structure / class / union; scalars
and pointers are refused with `use peek <target> instead`. Sub-objects are addressed via
the dotted path.

### 4.2 Member enumeration

For each layer, walk `DW_TAG_member` and `DW_TAG_inheritance` children:

- **`DW_TAG_member`** is included only when `DW_AT_data_member_location` is present. Static
  `constexpr` class members carry `DW_AT_declaration` and no location — they share no storage
  with the instance, so including them would collide with the first real field.
- **`DW_TAG_inheritance`** contributes its base type's members at the inheritance offset
  (decoded the same way as `DW_AT_data_member_location`).

### 4.3 Byte fetch — chunked peek

The host reads the full byte image by issuing `peek 0x<addr+i> <n>` requests with
`n = min(256, remaining)`, parsing each reply (typed or dump format) back to little-endian
bytes and concatenating. The address range must remain readable on the device for every
chunk; the host does not split around unreadable holes.

A size > 4096 B prints a one-line `<N> round-trips` warning before reading. There is no
hard cap.

### 4.4 Decoding rules

Per member's stripped type DIE:

| Tag | Render |
| --- | --- |
| `DW_TAG_base_type` | `(encoding, byte_size)` → struct fmt: `int8..int64`, `uint8..uint64`, `bool`, `char`/`uchar`, `float`, `double`. Result printed as a decimal integer, `true`/`false`, or `%.6g`. |
| `DW_TAG_pointer_type` / `DW_TAG_reference_type` / `DW_TAG_rvalue_reference_type` | 32-bit little-endian hex (`0x<8 hex>`). Pointee is not followed. |
| `DW_TAG_enumeration_type` | Decimal value plus the matching `DW_TAG_enumerator` name, or `<unknown enum>`. |
| `DW_TAG_array_type` | `char[]` / `uchar[]` → quoted Python string up to the first NUL; other scalar element types → first 8 elements `[a, b, …]` with `… +N` suffix when truncated. Multi-dimensional `DW_TAG_subrange_type` lists are multiplied to one total count. |
| Aggregate (`structure_type` / `class_type` / `union_type`) | If `depth_left > 0` and `size > 0`, recurse one level deeper. Else render as `<TypeName, N B>`. |
| anything else | `<tag-name>` |

Field offsets in the rendered output are relative to the top-level `<obj>` (so they match
`peek 0x<base + offset>`). Indentation conveys nesting.

`_type_name` synthesises labels for unnamed type DIEs:
`*` / `&` suffix for unnamed pointer / reference types (`Pointee*`, `Pointee&`); `(anon
struct/class/union)` for unnamed aggregates.

### 4.5 Depth

`[depth]` defaults to `2`, range `[0, 16]`.

- `0` → no recursion, every embedded aggregate is the one-line summary. Equivalent to the
  flat field listing.
- `N > 0` → recurse up to `N` levels deep; beyond that, summary.

Drilling deeper without raising depth is done by extending the dotted path — `peek-struct
mppt.charger.batSt.coulombCounter` reads only the `coulombCounter` slice and renders it at
its own depth budget.

### 4.6 Output format

```
<label> @ 0x<addr>  (<TypeName>, <size> B, depth≤<N>)
  +0x<offset>  <type label>  <member name>  = <decoded value>
  +0x<offset>  <nested type>  <member name>
    +0x<offset>  ...
```

The `<offset>` column is hex, padded to 4 digits. The `<type label>` column is left-padded
to 24 chars, the member name to 28; both expand for longer names.

### 4.7 Failure modes

| Cause | Message |
| --- | --- |
| no arguments | `peek-struct: expected <symbol>[.field…] [depth]` |
| non-integer depth | `peek-struct: bad depth '<s>'` |
| depth out of range | `peek-struct: depth out of range [0,16]` |
| no ELF found | `peek-struct: no firmware ELF — build, set $FUGU_ELF, or pass --elf` |
| `pyelftools` missing | `pyelftools missing — pip install pyelftools (or source idf-export.sh)` |
| unknown symbol | `symbol not found: '<head>'` (or `…(base of '<dotted>')`) |
| field not in struct | `no member '<m>' in <TypeName>` |
| scalar/pointer target | `peek-struct: <target> is not a struct/class/union (<TypeName>); use peek <target> instead.` |
| chunked-read timeout | `peek-struct: peek '<cmd>' failed: <reply-text>` |

---

## 5. Build-ID matching

The host trusts that the local ELF matches the flashed firmware — there is no build-ID
verification yet. When the two diverge (a rebuild after a flash; OTA from a different host)
the addresses returned by `nm` / DWARF are silently stale and `peek` will read whatever
bytes happen to live at those addresses. Inspect `uptime` (which prints the running app's
build date and git description) to spot a mismatch.

---

## 6. Limitations

Not yet handled by the host decoder; the device protocol is unaffected:

- Bitfields (`DW_AT_bit_size` / `DW_AT_data_bit_offset`).
- Multi-base virtual inheritance (virtual-base offset resolution).
- Union variant discrimination — every member is rendered as if it were active, since DWARF
  carries no discriminator.
- Pointer dereferencing — `*ptr` requires a second `peek` round-trip and is not implemented.
- Array indexing inside the dotted path (`arr[3].x`).
- ELF / firmware build-ID equality check.

`pyelftools` is loaded lazily from the active environment; when absent the host searches
`~/.espressif/python_env/idf*/lib/python*/site-packages` and prepends the first match to
`sys.path` before importing.
