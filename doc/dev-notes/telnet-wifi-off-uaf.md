*this document is an LLM generated placeholder*

# `wifi off` over telnet — lwip pbuf use-after-free (fixed)

Found 2026-05-30 by `etc/e2e-test/fuzz_extreme.py` flooding the console of an esp32-classic running a
vconv build. The fuzzer issued `wifi off <N>` over **telnet** and the device panicked.

## Symptom

Coredump decoded to `InstFetchProhibitedCause`, `PC = 0xfefefefe`. `0xfefefefe` is the ESP-IDF
`CONFIG_HEAP_POISONING` *free* fill pattern, so the CPU jumped through a **freed** function pointer.
Crashed task: `loopTask` (core 0). Backtrace (bottom-up):

```
loopNetwork_task → ServiceManager::tickAll()
  → TelnetService::onTick() → telnet.loop()
    → onInputReceived lambda → handleCommand() → cmdWifi("off")
        ├─ disconnect_wifi(true)            // WiFi.disconnect → deinits the netif/lwip
        └─ UART_LOG("WiFi off for N min")
              → vprintf_mux() → log_telnet->write()   // mirror to the telnet client
                 → NetworkClient::write (socket now dead) → stop() → lwip_close
                    → free_socket → pbuf_free → esp_pbuf_free → call 0xfefefefe  💥
```

## Root cause

`cmdWifi` ran the WiFi teardown **synchronously, from inside the telnet input callback**
(`telnet.loop()`). `WiFi.disconnect()` deinitialises the netif — including the lwip socket/pbuf pool
backing the very telnet client the command arrived on. The command's confirmation log is then
mirrored by `vprintf_mux` to that same client (`log_telnet`); the write touches freed pbuf memory and
calls an already-freed custom-free callback.

Reproducible in **any** `WITH_NETW` build, not just vconv/classic — it needs only a telnet client and
`wifi off`. COMPREHENSIVE heap poisoning made the jump a clean `0xfefefefe`; without poisoning the
freed memory holds stale data and the fault is less deterministic (but still a UAF). Same family as
the mDNS-on-WiFi-deinit UAF and the earlier `vprintf_mux` log-mirror bugs: **logging to a network
client while/after that client's transport is being destroyed by the command itself.**

## Fix

Defer the WiFi teardown out of any input callback (`src/cli.cpp`, `src/main.cpp`, `src/service.h`):

- `cmdWifi` off-branch only sets `g_app.disableWifi` now — it no longer calls `disconnect_wifi()`.
- `networkLoopTick()` gained a **WiFi-down edge** (latched once per edge) that runs the teardown on
  the next tick, *outside* any callback: `g_services.stopNetworkServices()` then `disconnect_wifi()`.
- `ServiceManager::stopNetworkServices()` (new) stops every `requiresNetwork()` service while the
  netif is still valid — symmetric to `startEnabledNetworkServices()` on the up edge. Stopping the
  telnet service runs its `onStop()`, which drops the `log_telnet` sink before the socket dies.

So the netif is never deinited from inside `telnet.loop()`, and the "WiFi off" confirmation mirrors
to a still-valid socket.

## Verification

esp32-classic vconv build `fry-brk1-69`: survives `wifi off 1` over telnet and a full `fuzz_extreme`
flood; the coredump partition stays empty (read with `parttool.py read_partition --partition-name
coredump`). Before the fix the same `wifi off` crashed instantly.

## Notes for reproducing on classic

Classic UART0 console RX is dead after an RTS reset, so the fuzz was driven over **telnet** (the
serial-only fuzz scripts were monkeypatched onto `SocketTransport`; see
`website/docs/reference/services.md` and the e2e notes). Coredump streaming over the console stalls ~13 KB, so pull the
partition over serial with `parttool.py` instead.
