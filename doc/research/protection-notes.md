*this document is an LLM generated placeholder*

# Protection page: text removed from the published page (2026-11-24)

Removed from `website/docs/internals/protection.md` while trimming it to reference text. Kept here as rationale.

## High-side short check: voltage-only hold

- The voltage-only hold is 60 s with or without the low-side clamp. That criterion needs Vout above the OV threshold,
  so Vin is far above the 9 V at which the clamp may engage. A latch turns the gates off, which cannot stop the shorted
  high side, so an earlier latch would not take the panel voltage off the loads.
- A short holds Vout at Vin indefinitely. On a shared output bus, another charger can push the battery above the OV
  threshold while the panel voltage passes through the window, but rarely for a whole minute.

## Limits of the checks (dropped bullets, said elsewhere on the page)

- A battery driven above the OV threshold by another charger, while the panel voltage passes through the window,
  latches the voltage criterion after 60 s.
- The clamp still discharges the input capacitors through the shorted high side and the low side.
- A short that latches by day keeps the panel voltage on the output until the night.
- A backflow switch stuck closed turns the hold into a battery short for two samples, until the release.

## Backflow switch test, toggle variant: criterion detail

- A blocking switch adds its body-diode drop Vf while off: on a soft source the current stays and y rises by Vf (Vin
  rises by Vf / D); on a stiff source the current collapses instead. In between, y rises by Vf less the drop the lost
  current gave back on the loop resistance.
- Unsteady neighbours mean the source or the load changed during the cycle.
- The 0.1 V threshold also sits below a partial current collapse's step with Vf down to 0.15 V.
- Without a battery, the 1 V duty raise per turn-off shows a blocking switch even where the divider bleeds the output
  too slowly to show a drop.
