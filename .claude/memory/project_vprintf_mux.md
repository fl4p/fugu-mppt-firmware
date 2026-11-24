---
name: vprintf_mux: va_list, static loc_buf, vsnprintf OOB
description: vprintf_mux bugs: spent va_list, shared static loc_buf race, and the vsnprintf truncation-return OOB write
created: 2026-11-23T09:48:10.239Z
metadata:
  node_type: memory
  generator: pi-claude-memory
  origin: explicit
  type: project
  originSessionId: 01a207b9-17c4-72ed-9de3-1f02898edde0
---

Three bugs in `src/logging.cpp`, all producing the same user-visible symptom — **UART/serial console
clean, but the telnet/BLE/MQTT mirror shows float-byte gibberish** — which is why they were fixed
one at a time.

**1. Spent `va_list` reused (`va_copy`).** `vprintf_mux` consumed `argptr` in `old_vprintf` (the UART
consumer) and then reused the same spent `va_list` for the mirror `vsnprintf`. A `va_list` is
single-use, so the mirror read garbage — raw IEEE-754 stack bytes like `\x80\x3f` — for any line with
variadic args: literal lines (`help`) mirrored fine, the float-heavy `loopLF` status line corrupted.
Fixed with `va_copy(ap2, argptr)` before the mirror and `va_end(ap2)` after.

**2. Shared static mirror buffer.** The mirror was built in `static char loc_buf[300]`, but
`vprintf_mux` runs concurrently on core 0 from the network-loop task (the float-heavy `loopLF` status
line) and the MQTT client task (`cmd_input` → `handleCommand` → `UART_LOG`, plus every `ESP_LOGx` in
the mqtt event handler). Preemption mid-`vsnprintf` clobbered the shared buffer, again emitting
leftover float bytes (`B??`) inside replies such as `help`. UART is unaffected because
`old_vprintf(fmt, argptr)` consumes `argptr` directly and never touches `loc_buf`. Fix: drop `static`
→ stack-local `char loc_buf[300]` (also fixes re-entrancy when a log callback itself logs; 300 B is
fine on the 8192 B mqtt task stack).

**3. `vsnprintf` return value used as an offset.** When `vsnprintf(buf, n, fmt, args)` truncates it
returns the **untruncated** length — what it *would* have written — so `p += r` in a fixed `new
char[...]` buffer walks past the end and smashes the next heap block. The TLSF allocator then trips
later in `block_locate_free: block_size >= *size` with a backtrace pointing at an innocent `malloc`
(e.g. `rtcount`'s `unordered_map::operator[]`). Hit twice in `logging.cpp`: first in `vprintf_mux`
(`0b8f4fdf`), then in `enqueue_log` after a sweep-plot TLSF panic on 2026-05-24. **Audit every site
doing `r = vsnprintf(p, cap, ...); p += r;` or `buf[r] = '\n';` and clamp `r = min(r, cap-1)` first**
— `snprintf` too. `HEAP_POISONING_LIGHT` flags the next overrun directly instead of via a delayed
TLSF assert; leave it on while shaking these out.

Related: [[project_logging_stack_budget]] (the same mux path on a foreign task's stack),
[[project_build_toolchain]].
