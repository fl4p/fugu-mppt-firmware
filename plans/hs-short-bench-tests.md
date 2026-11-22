*this document is an LLM generated placeholder*

# Bench tests: HS short, backflow switch, LS clamp

Status: planned (2026-11-22). None run yet.

The firmware side is on main (b2332679): the idle HS-short check, `selftest bflow` (daily) and the night-gated LS
clamp (`board.conf::hs_clamp`, off by default). The backflow-switch test passed on fry and flat on 2026-11-21. The
clamp and the fault paths are validated only in the host sim and in SPICE
(`doc/research/2026-11-20-hs-short-bflow-spice.md`, decks in `dcdc-tools/examples/fugu2-hs-short/`). The SPICE
verdicts hinge on board quantities nobody has measured. These tests close that gap.

Never on fry or flat. Measurements 1-4 are passive and may run on any Fugu2 board; tests 5-8 short a FET on purpose
and need a sacrificial Fugu2 board.

## Passive measurements (any Fugu2 board)

1. **"+12V" rail under load.** DMM/scope on the AP3012 output while converting at about 10 A. Nominal 9.34 V
   (R37 33k / R38 5.1k). The SPICE corner uses 10.11 V. It sets the LS drive, the clamp peak and the static BF gate
   bias (1.63 V at 9.34 V).
2. **BF threshold voltage of the fitted TPH1R306PL.** Per die, Vgs at Id = 1 mA, Vds = 10 V, at room temperature (and
   one warm reading at about 60 °C). The datasheet window is 1.5-2.5 V; the digitized chart gives 1.99 V at 25 °C,
   −5.7 mV/K. It decides how far above ground the BF source may ring before the BF conducts.
3. **BF static gate bias with the LS held on.** Converter off, LS forced on (`dc`/diagnostic short with Vin from a
   current-limited supply at 0 V), measure BflowG−BflowS. SPICE: 1.63-1.65 V.
4. **Assembly facts.** On every board that may get `hs_clamp=1`: is D16 (ST15100S) fitted, is the R11/C8 snubber
   fitted, what is C10 (fry: 470 µF ZLH per HW Stories; flat: unknown), and the input caps (2 × ZLH 470 µF).
   Also the LS gate rise time and, if a scope with a Rogowski probe is available, the commutation-loop ringing
   frequency (extracted loop: 3.2 nH).

## Shorted-HS tests (sacrificial board)

Short the HS with a link sized for the test current, or use a board whose HS already failed short. Panel side from a
current-limited lab supply (start 1 A), battery side from a current-limited supply first, a small LFP pack only after
the supply runs pass. Scope channels: BflowS−GND, BflowG−BflowS, SW−GND, battery current (clamp probe), Vin.

5. **Idle HS-short detection.** Converter off, BF off. Expect `hs-short` to latch within about 3 s at forward current
   above the threshold, start blocked, `selftest` showing the state. Repeat with the battery supply disconnected
   (voltage-only path, 60 s hold). Confirms the thresholds against real sensor offsets (fry's learnt zero is
   −0.555 A).
6. **Night clamp engage.** `hs_clamp=1`, `bflow_bat=1`, fresh blocks pass. Drop the panel supply to 8 V, then 5 V,
   then 9.5 V (must not engage). Expect `clamp pending (Vin<9V ...)`, after 60 s the 20 ms LS ramp, then the hold.
   Pass: BflowS stays above −2 V, BF Vgs stays below its measured Vth (test 2), no battery current through the BF,
   LS case temperature rise negligible. SPICE: BflowS −0.8 V (flat) to −1.6 V (fry) with the 10-step staircase.
7. **Hold through "dawn".** With the clamp held, ramp the panel supply current 0 → panel Isc over about 10 minutes.
   Expect the LS to carry it with Vin near 0 and the hold to stay. Then let the battery side rise: the clamp must not
   start discharging the battery.
8. **BF stuck closed under the clamp.** Force the BF on (or short it) while clamped, battery from a current-limited
   supply. Expect release within two samples, gates off, `hsClampUnsafe` set and persisted across a reboot and
   `selftest clear`, until a fresh blocks pass.

## Not to run

- A hard (unramped) clamp above 9 V and any clamp at 16s / 55 V: SPICE puts the LS at 2-4 × its SOA and the BF in
  avalanche.
- The fuse-blow crowbar (`doc/plan-hs-bf-double-fault-fuse-blow.md`): SPICE shows the LS fails before F1 clears.
  Only after the board fixes in Fugu2 issue #37, and then on a sacrificial board with a supply first.

## After the tests

- Feed the measured rail, BF Vth and C10 back into the SPICE decks and re-run Q9 (night clamp).
- Only then consider `hs_clamp=1` on a live converter, one board first, with the daily BF test passing.
