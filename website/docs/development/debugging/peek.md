---
title: peek / peek-struct
sidebar_position: 3
---

# `peek` / `peek-struct` — Memory inspection

`peek` reads arbitrary memory on a running device. The host can optionally interpret the bytes
through the build's DWARF debug info. The tool has two layers:

- Device: `peek <addr> [len]` returns up to 256 raw bytes per request. The device knows nothing
  about symbols or types.
- Host (`etc/fugu_console.py` + `etc/idf-devtools/peek_symbols.py`): the host rewrites
  `peek <symbol>[.field…]` to a numeric address before sending. It renders `peek-struct <obj>` by
  issuing one or more `peek` requests and decoding the byte image against the build ELF's DWARF.

The wire protocol is byte-oriented and carries no symbols or type info. The client does all naming
and decoding, against the same ELF that was flashed.

---

## 1. Device-side: `peek <addr> [len]`

### 1.1 Syntax

The device command takes an address and an optional length:

```
peek <addr> [len]
```

- `<addr>`: required. The device parses it with `strtoul(s, &endp, 0)`: `0x…` (hex), `0…`
  (octal), or decimal. The parse must consume the entire token. Trailing junk is an error.
- `[len]`: optional, default `4`. Range `[1, 256]`. Decimal integer.

### 1.2 Memory-region validation

The (`addr`, `addr + len - 1`) interval must lie entirely in a single class of region. The
following table lists the classes, how the device reads each, and its constraints:

| Region | Method | Constraints |
| --- | --- | --- |
| Internal RAM (DRAM/SRAM/RTC), DROM (flash-mapped const), external RAM (PSRAM) | `memcpy` | byte access; no alignment needed |
| IROM / IRAM (executable, instruction-bus only) | 32-bit volatile load | `addr % 4 == 0` and `len % 4 == 0` |
| Peripheral MMIO (S3 `0x60000000-0x600D1FFF`; ESP32 `0x3FF00000-0x3FF7FFFF`) | 32-bit volatile load | `addr % 4 == 0` and `len % 4 == 0`; not synchronised with the RT loop |

Passing this check means only that the range lies in a known region. The read itself can still
fault or have side effects:

:::warning MMIO reads can crash or change state
Reading a register of a clock-gated peripheral faults the bus (panic and reboot, which stops conversion on
a converting device). FIFO, capture and `*_INT_ST` reads have side effects (UART/I2C FIFO pop, MCPWM
capture, interrupt-status latches).
:::

The device rejects addresses outside these classes, or ranges that straddle them, with
`peek: 0x<addr> not safely readable`. It rejects executable or MMIO ranges with a non-aligned addr
or len with `peek: executable region needs 4-byte aligned addr+len` (or `mmio region …`).

### 1.3 Output

The device chooses one of two output formats by `len`.

For `len ∈ {1, 2, 4, 8}`, the typed-scalar format prints one line:

```
peek 0x<addr> = 0x<value>
```

The width of `<value>` matches `len`: 2, 4, 8, or 16 hex digits. The value is the little-endian
integer read from the bytes, which is what `*(uintN_t*)addr` would return on the device.
Example: `peek 0x3fca7674 = 0x0000033a`.

For any other `len` in `[1, 256]`, the hex+ASCII dump format prints one line per 16 bytes:

```
0x<addr>: aa bb cc dd ... 16 hex bytes ...  |....ASCII....|
```

Each dump line has these parts:

- Address prefix `0x` followed by 8 hex digits and a colon.
- Up to 16 single-space-separated `%02x` byte tokens. A short final row pads with `   ` so the
  ASCII column aligns.
- ` |` then up to 16 ASCII chars (printable range `0x20..0x7e`, others rendered as `.`) then
  `|`.

In both formats, the normal `OK: peek <addr> <len>` reply marker follows the data lines.

### 1.4 Failure modes

The device reports these errors:

- `peek: expected <addr> [len]`: no arguments.
- `peek: invalid address '<s>'`: `strtoul` couldn't fully parse.
- `peek: len out of range (1..256)`: `len <= 0` or `len > 256`.
- `peek: executable region needs 4-byte aligned addr+len`: IROM/IRAM access without alignment.
- `peek: 0x<addr> not safely readable`: address class is unknown or mixed.

All failures emit `ERR: peek …` as the command's reply marker.

---

## 2. Host-side address resolution

The CLI rewrites the `<addr>` token before sending. The wire accepts two forms, and only the
second triggers a rewrite:

| Form | Action |
| --- | --- |
| `peek 0x<hex>[…] [len]` | passthrough |
| `peek <symbol>[.field…][+off] [len]` | resolve client-side, replace with `peek 0x<addr> <len>` |

A `<symbol>` matches the regex `[A-Za-z_][A-Za-z0-9_$:.]*`. The host reads symbols once per
session from `nm -S --defined-only` over the build ELF and caches them by `(path, mtime)`. When the
demangled simple name of a C++ mangled symbol (`_Z*`) is unambiguous, the symbol also gains a
demangled-alias entry, so `setupCli` resolves to `_Z8setupCliv`.

### 2.1 Dotted member access

The host resolves `<symbol>.<m1>.<m2>…` by walking DWARF:

1. Find the variable's `DW_TAG_variable` DIE with a `DW_AT_type` attribute.
2. Strip type qualifiers (`DW_TAG_typedef`, `DW_TAG_const_type`, `DW_TAG_volatile_type`,
   `DW_TAG_restrict_type`, `DW_TAG_atomic_type`).
3. For each `.<mN>`, check that the current type is a `DW_TAG_structure_type`,
   `DW_TAG_class_type`, or `DW_TAG_union_type`. Then search `DW_TAG_member` children plus any
   `DW_TAG_inheritance` base classes breadth-first. The first match wins.
4. Compute the final address as `nm[base] + Σ member_offsets`. The size hint is the leaf type's
   `DW_AT_byte_size`.

The host decodes `DW_AT_data_member_location` in both its constant form (already an int) and its
`DW_OP_plus_uconst` exprloc form (ULEB128).

### 2.2 Default length

When `[len]` is omitted, the host fills it in by these rules:

- Symbol with `nm` size > 0: use `min(nm_size, 256)`.
- Dotted member: use the leaf field's `DW_AT_byte_size`.
- Otherwise: `4` (matches the device default, hits typed-print).

---

## 3. Host-only: `sym <pattern>`

`sym` lists ELF symbols that match `<pattern>`. The host never sends this command to the device.
It accepts a substring or a regex:

```
sym <substring>
sym /<regex>/
```

- A bare pattern is a case-insensitive substring match.
- `/.../` is a Python regex.
- The output has one line per match, `  0x<addr> <size> <name>`, sorted largest first, then by
  name.
- The default limit is 40 hits. When the list is truncated, refine the pattern.

---

## 4. Host-only: `peek-struct <obj>[.field…] [depth]`

`peek-struct` renders the bytes at the target's address as a DWARF-typed field tree.

### 4.1 Resolution

`peek-struct` uses the same address and type walk as §2.1. The leaf type must be a structure,
class, or union. The host refuses scalars and pointers with `use peek <target> instead`. You address
sub-objects through the dotted path.

### 4.2 Member enumeration

For each layer, the host walks the `DW_TAG_member` and `DW_TAG_inheritance` children:

- `DW_TAG_member` is included only when `DW_AT_data_member_location` is present. Static
  `constexpr` class members carry `DW_AT_declaration` and no location. They share no storage
  with the instance, so including them would collide with the first real field.
- `DW_TAG_inheritance` contributes its base type's members at the inheritance offset
  (decoded the same way as `DW_AT_data_member_location`).

### 4.3 Byte fetch — chunked peek

The host reads the full byte image by issuing `peek 0x<addr+i> <n>` requests with
`n = min(256, remaining)`, parsing each reply (typed or dump format) back to little-endian
bytes and concatenating. The address range must remain readable on the device for every
chunk, because the host does not split around unreadable holes.

For a size > 4096 B, the host prints a one-line `<N> round-trips` warning before reading. There is
no hard cap.

### 4.4 Decoding rules

The host renders each member according to its stripped type DIE:

| Tag | Render |
| --- | --- |
| `DW_TAG_base_type` | `(encoding, byte_size)` → struct fmt: `int8..int64`, `uint8..uint64`, `bool`, `char`/`uchar`, `float`, `double`. Result printed as a decimal integer, `true`/`false`, or `%.6g`. |
| `DW_TAG_pointer_type` / `DW_TAG_reference_type` / `DW_TAG_rvalue_reference_type` | 32-bit little-endian hex (`0x<8 hex>`). Pointee is not followed. |
| `DW_TAG_enumeration_type` | Decimal value plus the matching `DW_TAG_enumerator` name, or `<unknown enum>`. |
| `DW_TAG_array_type` | `char[]` / `uchar[]` → quoted Python string up to the first NUL; other scalar element types → first 8 elements `[a, b, …]` with `… +N` suffix when truncated. Multi-dimensional `DW_TAG_subrange_type` lists are multiplied to one total count. |
| Aggregate (`structure_type` / `class_type` / `union_type`) | If `depth_left > 0` and `size > 0`, recurse one level deeper. Else render as `<TypeName, N B>`. |
| anything else | `<tag-name>` |

Field offsets in the rendered output are relative to the top-level `<obj>`, so they match
`peek 0x<base + offset>`. Indentation conveys nesting.

`_type_name` synthesises labels for unnamed type DIEs. Unnamed pointer and reference types get a
`*` or `&` suffix (`Pointee*`, `Pointee&`). Unnamed aggregates get `(anon
struct/class/union)`.

### 4.5 Depth

`[depth]` defaults to `2`, with range `[0, 16]`:

- `0` means no recursion. Every embedded aggregate is the one-line summary, which is equivalent
  to the flat field listing.
- `N > 0` recurses up to `N` levels deep. Beyond that, aggregates are summaries.

To drill deeper without raising depth, extend the dotted path. `peek-struct
mppt.charger.batSt.coulombCounter` reads only the `coulombCounter` slice and renders it at
its own depth budget.

### 4.6 Output format

The output has a header line followed by one indented line per member:

```
<label> @ 0x<addr>  (<TypeName>, <size> B, depth≤<N>)
  +0x<offset>  <type label>  <member name>  = <decoded value>
  +0x<offset>  <nested type>  <member name>
    +0x<offset>  ...
```

The `<offset>` column is hex, padded to 4 digits. The `<type label>` column is left-padded
to 24 chars and the member name to 28. Both expand for longer names.

### 4.7 Failure modes

The host reports these errors:

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

The host trusts that the local ELF matches the flashed firmware. There is no build-ID
verification yet. The two diverge after a rebuild following a flash, or after an OTA from a
different host. The addresses returned by `nm` and DWARF are then silently stale, and `peek` will
read whatever bytes happen to live at those addresses. To spot a mismatch, inspect `uptime`, which
prints the running app's build date and git description.

---

## 6. Limitations

The host decoder has these open items. The device protocol is unaffected:

- Bitfields (`DW_AT_bit_size` / `DW_AT_data_bit_offset`).
- Multi-base virtual inheritance (virtual-base offset resolution).
- Union variant discrimination: the host renders every member as if it were active, because DWARF
  carries no discriminator.
- Pointer dereferencing: `*ptr` requires a second `peek` round-trip and is not implemented.
- Array indexing inside the dotted path (`arr[3].x`).
- ELF / firmware build-ID equality check.

The host loads `pyelftools` lazily from the active environment. When it's absent, the host
searches `~/.espressif/python_env/idf*/lib/python*/site-packages` and prepends the first match to
`sys.path` before importing.
