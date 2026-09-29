*this document is an LLM generated placeholder*

# VirtualConverter — TODO

Design: [docs/superpowers/specs/2026-05-23-virtual-converter-design.md](../../doc/superpowers/specs/2026-05-23-virtual-converter-design.md)

## v1 — buck, single PV string, ideal switches

1. `src/sim/vconv.{h,cpp}` — pure model. Analytic per-cycle solver for I_L (HS-on / LS-on / off phases). Forward-Euler cap dynamics. PV exponential, stiff battery with backflow clamp.
2. `src/pwm/vconv.h` — `PWM_VConv` shim, LEDC-style API, forwards `update_pwm` into `g_vconv`.
3. `src/adc/vconv.h` — `ADC_VConv : AsyncADC<float>`, reuses `PeriodicTimer`/`TaskNotification` pattern from `ADC_Fake`.
4. `src/buck.h` selector + CMake gate. `WITH_VCONV` mutually exclusive with `WITH_MCPWM`.
5. `setupSensors()` in `main.cpp` recognises `*_adc=vconv`.
   - **Channel-mapping decision (open):** `ADC_VConv` has a fixed channel→quantity table (`0=Vin, 1=Vout, 2=I_out_avg, 4=NTC`) so the per-sensor `*_ch` knobs in conf are redundant. Pick one:
     - **A. Drop `*_ch` for vconv** — `setupSensors()` hard-codes the four-line handle assignment when backend is `vconv`. Simplest, no silently-wrong configs. Vconv looks different from real backends in conf.
     - **B. Honor `*_ch`** — add `ADC_VConv::bind(name, ch)`; `setupSensors()` calls it per sensor; `getSample(ch)` dispatches via the binding table. Uniform with other backends; ~20 lines of plumbing.
     - **C. Encode quantity in selector string** — `vin_adc=vconv:vin`; backend parses the suffix and auto-assigns channels. Cleanest semantically, deviates hardest from convention, needs conf-editor tooling update.
     - Recommendation: A. Decide before step 5.
6. `vconv.conf` parsing + wiring; update `website/docs/reference/config/<file>.md` + `etc/config-tool/conf-editor.html`.
7. `vconv` console command (`pv`, `bat`, `set`, dump).
8. `config/lab/vconv_mock/` board config.
9. `test/host-stub/vconv-test.cpp` — CCM steady-state, DCM boundary, PV IV-curve peak, cap time constant.
10. ~~On-target smoke test: MPPT finds MPP near `0.8 * Voc`, PD loops regulate, charger reaches CV.~~
    **Done (2026-05-30).** Validated on esp32-classic: plant `Isc=13 Voc=76 k=0.85`, MPPT converges
    sweep→P&O and holds `Vin≈64.6V (=0.85·Voc), ~815W`, no protection trips. Classic needs its own
    `config/lab/vconv_mock_esp32` (the original `vconv_mock` is S3-only: `mcu=esp32s3` + GPIOs 47/40/42
    crash `pinMode` on classic) and `sdkconfig.esp32vconv`. See `doc/dev-notes/telnet-wifi-off-uaf.md`
    for a crash this run surfaced.

## Config validation tool

Conf files live on the device's `littlefs` partition and are flashed/edited independently of the
firmware — CMake never reads them, so build-time refusal is the wrong layer. Build a checker
(host-side Python, or a `validate` console command on the device) that vets `vconv.conf` +
`coil.conf` + `board.conf` together and refuses combinations that violate the model's numerical
preconditions or the firmware's documented invariants:

- `r_bat > 0` and `c_out > 0` and `c_in > 0` and `L0 > 0`.
  (The forward-Euler stability cliff `T/(R_bat · C_out) < 2` is now gone — V_out uses
  backward-Euler, so any `r_bat > 0` is numerically safe.)
- `0 < pv_k < 1` (Newton solver for α degenerates outside this range).
- `voc > 0`, `isc > 0`.
- `vbat_ac_amp >= 0` (and zeroed automatically when `v_bat = 0`, but warn).
- Sanity: `2 * voc * 1.05 >= v_bat` (otherwise the V_in clamp would block the converter from ever
  pulling V_out up to V_bat).

Hooks into `etc/config-tool/` (already validates other keys) and/or a `vconv validate` console
verb that runs at boot from `configureVirtualConverter()` and refuses to start the plant if any
check fails.

## v2 — extensions (priority-ordered)

- **PV strings** — multiple PV "strings" in parallel, each with its own `isc/voc/k`. `I_pv_total(V) = Σ I_pv_string_i(V)`. Configured via `pv_strings=N` in `vconv.conf` and `string_i_{isc,voc,k}=...`. Enables partial-shading scenarios (multiple local maxima on the IV curve) for stressing the MPPT global-sweep / re-sweep logic.
- **Boost topology** — reuse the analytic per-cycle solver with sides swapped; gate on `vconv.conf::topo=boost` and `isBoost` mirrors `buck.h`.
- **Cap ESR** — `c_in_esr`, `c_out_esr`. `V_observed = V_cap + R_ESR * I_cap`. Only matters when testing ripple-sensitive code (scope output, ADC noise).
- **MOSFET losses** — `R_ds_on_hs`, `R_ds_on_ls`, switching-loss energy per edge. Adds I²R drops + a fixed per-cycle energy loss. Skews efficiency numbers toward realistic.
- **Coil saturation** — non-constant `L(I_L)`, e.g. linear knee model. Useful for testing protection under saturation events; current firmware uses constant `L0 * 0.95` (`InductivityDcBias`) which the model mirrors.
- **Time-varying scenarios** — built-in scripted insolation/load profiles (`vconv scenario sunrise 60s`, `vconv scenario cloud 5s`). For now use external test-driver loops.
- **Multiple sources / sinks** — e.g. dual-input MPPT, hybrid battery + grid sink. Out of scope unless the firmware grows to support these.

## v3 / nice-to-have

- Host-side firmware run — would need to grow `test/host-stub/` to cover FreeRTOS, `esp_timer`, MCPWM. Big lift, modest payoff while on-target testing works.
- Wokwi custom-chip port — same model exported as a Wokwi chip, runs out-of-MCU. Lets people simulate the whole board in the browser.
