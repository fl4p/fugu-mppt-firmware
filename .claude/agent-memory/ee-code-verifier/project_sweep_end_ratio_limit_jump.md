---
name: sweep-end-ratio-limit-jump
description: protect() ends a sweep at the D>=0.9M ratio limit (bflow open, I<0.05A) and jumps duty to the MPP in one step; ungated forced PWM gets a reverse-current kick
metadata:
  type: project
---
Reviewed 2026-11-11 (integ-ota-1110, commits 8546c97/0b5c23b/3e0539c). After the jump the highD-bflowOpen guard still runs on the next protect pass, so the guard stays in force. Only gap found: with forced PWM ungated (fpwm_gate=0 or `sync forced!`), pwmPerturb keeps complementary LS on a large down-step. In the sim the jump 3190->605 gave Iout ewm -0.5 A. Lab configs only.

**Why:** update()'s normal sweep end fades at pwmCounts/64 per tick. This path skips the fade.
**How to apply:** the host sim's headline scenario (isc=0.1 k=0.7) never reaches this path. Use pv_k=0.6, dusk or ramp scenarios, or forced_pwm=1 fpwm_gate=0.
