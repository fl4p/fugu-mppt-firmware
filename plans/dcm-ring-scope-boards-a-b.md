*this document is an LLM generated placeholder*

# Scope capture of the DCM coil ring on fry and flat

Status: planned, not started (updated 2026-11-27). Needs Fab at the van with a scope.

## Goal

Measure the switch-node capacitance `C_sw` of fry (board B) and flat (board A) directly, from a
scope capture of the DCM coil ring, as was done for flu (board C, 2.98–3.04 nF, 2026-09-06).
Today both values are inferred from `L` and the ring frequency seen in `Iout`:

| Board | `L` (2026-11-25, BF closed) | ring `f_r` | inferred `C_sw` |
|---|---|---|---|
| A = flat | 49.5 µH | 317–330 kHz | ≈4.8 nF (on-device fit 4.9 nF) |
| B = fry | 76.5 µH | 241–257 kHz | ≈5.2 nF (on-device fit 5.4 nF) |

A direct fit checks the inference and the `Iout` ripple model behind it. Results go to the
private `dcdc-tools` repo; the published pages then change "inferred" to "measured".

## Method (as for flu)

- C1 on the switch node, C2 on the LS gate; trigger on the LS gate **falling** edge (LS turn-off,
  where the ring starts). Not a slew-rate trigger on the node: that fires on the HS edge instead
  (the 2026-08-14 flu archive mistake, see the docstring of `dcdc-tools/bench/flu_dcm_ring.py`).
- Converter in **diode emulation**, not forced PWM (forced PWM holds the LS on and removes the ring).
- Low duty deep in DCM with the battery connected, e.g. `dc 200` (D ≈ 0.05 at pwmMax 4103).
- About 12 events per point. Fit `v(t) = V_c − A·cos(ω(t − t₀))` over the unclamped part of the
  rise (3–93 %), `C_sw = 1/(ω²L)`; the swing clamps one diode drop above Vin.
- Optionally 2–3 Vin points (morning, noon) to confirm the quarter-cycle duration does not
  depend on Vin.

## Steps

1. Generalise `dcdc-tools/bench/flu_dcm_ring.py` to take the board, its `L` and the console
   transport (`fugu_console.py --mqtt --name <dev>`); it currently assumes flu and its rig.
2. On the day: check `lock.sh status <dev>` and `ota-<dev>` (other sessions OTA and A/B-test
   builds on both boards; `elf-archive/console-commands.jsonl` shows who else sends commands),
   then `lock.sh acquire <dev>`. Low power (morning), `dc 0`, then `dc 200`; capture; `mppt`
   afterwards. One board at a time. Send commands with `fugu_console.py --mqtt --name <dev>`:
   its lock guard refuses while another caller holds `ota-<dev>`.
3. Fit, store raw traces under `dcdc-tools/verifications/` with a write-up.
4. Update `website/docs/internals/dcm-ringing.mdx` (rows A, B), the field-sweep table on
   `website/docs/lab/coil-inductance.md` and `doc/lab/dcm-ringing-boards.md`.

## Safety

fry and flat are live converters (panel up to ~80 V, battery). Hold the device lock, stay in
manual PWM only for the capture, keep D low, never forced PWM, probe ground on power ground only,
restore `mppt` and check the status line afterwards.

## Open questions for Fab

- Which scope and probes are available at the van. The flu capture used an MXO44. The PicoScope
  ps2000 (`etc/pico_capture.py`) is 2-channel, 50 MS/s at tb=1, and caps a capture at about 3000
  samples (60 µs at 50 MS/s): enough for a ~250–330 kHz ring (period 3–4 µs), but the fitted
  quarter-cycle rise (~0.8–1 µs) gets only ~40–50 samples, so average more events (~30) than with
  the MXO44. Its ±20 V range needs a 10:1 probe for the ~80 V node.
- Where the switch node and the LS gate can be probed on the Fugu2 boards without removing them.
