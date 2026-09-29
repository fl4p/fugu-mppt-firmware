---
title: Coding conventions
sidebar_position: 5
---

*this document is an LLM generated placeholder*

# Coding conventions

Rules that are not obvious from reading the code. Most of them are enforced by the compiler or linker, the rest
protect the real-time loop or the flash budget.

## Compiler and toolchain

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
`adcSampler.update()`. See [Architecture](../internals/architecture.md).

- **No `vTaskDelay` or other voluntary sleep** on the RT path.
- **Nothing else on core 1.** `sdkconfig.defaults` pins the Arduino, lwIP and mDNS tasks to core 0; `loopRT` asserts
  it runs on `RT_CORE`. Pin new tasks to core 0.
- **`vout` is the last sensor added** in the sensor setup (`src/adc/sensor_setup.cpp`), so output over-voltage
  protection sees the freshest sample. Add new sensors before it.

## Code style

- **Low memory, small code.** Reuse data that already exists; in non-time-critical code, derive or cast rather than
  store. Think twice before adding a member variable; expose an existing private member with a getter instead.
- **Don't duplicate constants.** Never copy a `#define` into another file because the original header is not
  included; move it to a header both already include (e.g. `src/util.h`).
- **Minimal comments.** Keep them short and drop ones that restate a name or call.
- **Vendored libraries** under `components/` are upstream code; discuss changes before making them.

## Configuration keys

When you add, rename or remove a `.conf` key, update together:

1. the reference page for that file under [Configuration files](../reference/config/index.md);
2. the editor metadata in `etc/config-tool/conf-editor.html` (`META` and `FILE_KEYS`).
