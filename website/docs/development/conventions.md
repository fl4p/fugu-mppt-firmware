---
title: Coding conventions
sidebar_position: 5
---

# Coding conventions

The rules on this page aren't obvious from reading the code. The compiler or linker enforces most of them. The rest
protect the real-time loop or the flash budget.

## Compiler and toolchain

The build flags and the toolchain impose the following rules:

| Rule | Why |
|---|---|
| Designated initializers must cover every field | `main/CMakeLists.txt` sets `-Werror=missing-field-initializers`; a missing field fails the build |
| `IRAM_ATTR` on anything called from the continuous-ADC ISR | `sdkconfig.defaults` sets `CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE=y`; the ISR may run while flash cache is off. `-Werror=attributes` catches mismatched `IRAM_ATTR` forward declarations |
| Exceptions are allowed in `setup()`, never out of the RT loop | `-fexceptions` / `CONFIG_COMPILER_CXX_EXCEPTIONS=y`. In the RT path wrap in `try`/`catch` and call `stopAndBackoff()` |
| No `<sstream>`, `std::stringstream`, `std::ostringstream`, `std::stringbuf` | The toolchain installed for this project has a patched `<sstream>` whose `basic_stringbuf` constructor calls an undefined `basic_stringbuf_nop()` (an anti-bloat hook; not in Espressif's released GCC 14.2 sources). With it, any TU that constructs one fails to link with `undefined reference to 'basic_stringbuf_nop'`; without it the rule still stands to keep the binary small. Use `snprintf` or `std::string` concatenation |
| No `%hh` / `%ll` printf specifiers, no 64-bit integers in format strings | `CONFIG_LIBC_NEWLIB_NANO_FORMAT=y` (smaller printf, ~40 KB flash) has no 64-bit or C99 `hh`/`ll` support; they misparse silently. Narrow to 32-bit before printing |
| C++ standard | The `main` component compiles with `--std=gnu++20` (`main/CMakeLists.txt`) |

## Real-time path

The control loop (`loopRT`) runs alone on core 1 (`RT_CORE`) and blocks on the next ADC sample inside
`adcSampler.update()`. See [Architecture](../internals/architecture.md). The following rules protect it:

- Don't call `vTaskDelay` or sleep voluntarily on the RT path.
- Keep every other task off core 1. `sdkconfig.defaults` pins the Arduino, lwIP and mDNS tasks to core 0; `loopRT` asserts
  it runs on `RT_CORE`. Pin new tasks to core 0.
- `vout` is the last sensor added in the sensor setup (`src/adc/sensor_setup.cpp`), so output over-voltage
  protection sees the freshest sample. Add new sensors before it.

## Code style

New code follows these style rules:

- Keep memory use low and code small. Reuse data that already exists. In non-time-critical code, derive or cast
  rather than store. Think twice before adding a member variable, and expose an existing private member with a getter
  instead.
- Don't duplicate constants. Never copy a `#define` into another file because the original header is not
  included. Move it to a header both files already include (e.g. `src/util.h`).
- Keep comments few and short, and drop ones that restate a name or call.
- Vendored libraries under `components/` are upstream code. Discuss changes before making them.

## Configuration keys

When you add, rename, or remove a `.conf` key, update both of these places together:

- The reference page for that file under [Configuration files](../reference/config/index.md).
- The editor metadata in `etc/config-tool/conf-editor.html` (`META` and `FILE_KEYS`).
