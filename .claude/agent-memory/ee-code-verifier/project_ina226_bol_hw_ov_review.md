---
name: ina226-bol-hw-ov-review
description: ina22x_ov (INA226 BOL alert -> ISR gate force, CVRF polling) review 2026-11-13; GPIO ISR is non-IRAM, protect() only runs while enabled, poll phase lock is one-sided
metadata:
  type: project
---

Reviewed the uncommitted `ina22x_ov` change (worktree /private/tmp/ina-ov, 2026-11-13). Facts that hold beyond it:

- Arduino GPIO ISRs are registered with ARDUINO_ISR_FLAG=0 (CONFIG_ARDUINO_ISR_IRAM unset), so IRAM_ATTR on an ALERT-ISR call chain is cosmetic. `mcpwm_generator_set_force_level` is never IRAM (not in the IDF mcpwm linker.lf, even with MCPWM_CTRL_FUNC_IN_IRAM). Works on S3 only because CONFIG_SPI_FLASH_AUTO_SUSPEND=y keeps cache on; on classic ESP32 the ISR is deferred for a whole flash op (33-65 ms).
- `mppt.protect()` runs only when `!converter.disabled()` AND the sampler returned NewData (main.cpp loopRTNewData). An ISR-set flag consumed there stays latched while disabled and trips at the next enable, possibly hours later.
- `clearForce()` is only called from drvCommit(enabling); an ISR force during an enabled run holds until disable() + re-enable.
- ADC stall watchdog calls adcSampler.resetPeripherals() with the converter still running (shutdown only after 800 ms), so anything disarmed at the top of ADC_INA226::resetPeripherals is off during that window.
- INA226 MASK_EN (06h): SOL15 SUL14 BOL13 BUL12 POL11 CNVR10, AFF4 CVRF3 OVF2 APOL1 LEN0; CVRF is set regardless of CNVR and cleared by a MASK_EN read. tCT spec max is +10% over typ.
- CVRF polling that re-anchors at `now - lead` only corrects when the poll is early: if the chip is faster than the stored pair period by more than the lead, conversions get skipped (non-uniform fs).

**How to apply:** for any ISR-driven protection in this firmware, check the flag consumer runs while disabled, the arm survives runtime resetPeripherals, and the chain does not depend on IRAM it does not have.
