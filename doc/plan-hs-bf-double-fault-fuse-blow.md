*this document is an LLM generated placeholder*

# Plan: blow the battery fuse on an HS + backflow double fault

Status: deferred (2026-11-20). Not implemented.

## Context

The HS-short self-test (`src/selftest/hs_short.h`, `hsShortRt` in `src/mppt.cpp`) latches a fault and clamps the LS
FET on. With a working backflow (BF) switch the clamp shorts the panel through HS + LS, BflowS drops to ~0 V and the
BF body diode blocks the battery. The battery stops receiving unregulated charge.

A daily `selftest bflow` (planned alongside this document) stops conversion when the BF switch fails. That makes the
double fault, a shorted HS plus a BF that no longer blocks, unlikely, but not impossible: the BF can fail between two
daily checks, or fail during the HS short itself.

## The double fault

Path: Solar+ → shorted HS → coil → BF (stuck closed) → Bat+ → F1 → BT+.

- The coil is a short at DC, so the panel charges the battery directly at Vbat, bounded only by the panel current.
  No duty cycle (buck or boost) changes this. The LS is the only working switch, and it is not in the path.
- At night the battery forward-biases the panel and discharges into it.
- The BMS is the only protection left: it must stop charge at EOC and stop discharge at low SOC.
- The LS clamp makes it worse: it shorts the battery through coil + LS. The simulator showed −436 A within ~9 ms
  (`hs-short-bfclosed` scenario). The current firmware therefore releases the clamp on reverse current and goes to
  all gates off.

## Idea: crowbar the battery fuse

On a confirmed double fault, keep the LS on so the battery current through coil + LS + BF clears F1. With F1 open:

- The panel (through the shorted HS) can no longer reach the battery.
- The board keeps its supply from the panel side (supply is max(Vin, Vout)), so it can still report the fault.
- The LS clamp then also shorts the panel, the same end state as the single-fault case.

## Open questions before implementing

1. **Fuse.** F1 is a blade fuse (Fugu2 PowerStage review F4: MPN not recorded, 40 A footprint; a 58 V part is
   required, a 32 V ATO must not be fitted). Collect I²t, the time-current curve at 10-50x rating and the DC breaking
   voltage for the fitted part. A fuse rated below Vbat may arc instead of clearing.
2. **LS FET survival.** The LS FET's I²t / SOA must outlast the fuse's clearing I²t with margin, including its
   temperature at the time of the fault. If the LS fails open first, the crowbar ends with an intact fuse and the
   original double fault; if it fails short, the result is the same as a held clamp.
3. **Coil saturation.** The battery current ramps at Vbat/L only until the core saturates; after that the current is
   limited by the loop resistance (FETs, coil DCR, shunt, wiring, pack). Estimate the peak and di/dt per board
   (L0 50-80 µH on the live boards).
4. **BF FETs.** They carry the same current. Check their SOA too; a BF that is only partly on (gate near Vth) runs
   linear and may fail first.
5. **Battery source.** A large LFP pack can deliver kA. The board fuse is not the primary battery protection
   (Class-T/MEGA or BMS required at the pack). Confirm the crowbar cannot exceed F1's breaking capacity.
6. **Detection confidence.** The crowbar is irreversible. Required evidence before committing, for example:
   HS-short latched for the full 3 s window, reverse current under the clamp beyond a threshold for N consecutive
   samples, sensor sanity (no NaN, Vout and Vin consistent), and no other explanation (PSU mode, boost topology,
   another charger on the bus does not pass the converter shunt).
7. **Current sensing during the event.** The INA226 saturates far below the fault current. Decide which signal
   confirms that F1 has cleared (Vout at BT+ collapsing while Vin stays up, reverse current back to ~0).
8. **User policy.** Whether a firmware-initiated fuse blow is acceptable at all, and whether it needs an explicit
   `limits.conf` opt-in (default off).

## Validation steps (later)

- Datasheet numbers for F1, the LS and BF FETs: I²t, SOA, avalanche.
- SPICE run of the crowbar with the board's coil saturation curve and loop parasitics.
- Bench test on a sacrificial board with a current-limited supply first, then a small pack, never on a live
  converter.
- vconv: model F1 (I²t integrator → open) and add a `hs-short-bfclosed` variant that expects the fuse to open.
