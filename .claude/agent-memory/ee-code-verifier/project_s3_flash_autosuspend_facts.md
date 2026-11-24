---
name: s3-flash-autosuspend-facts
description: ESP32-S3 flash auto-suspend HW/IDF facts (IDF 6.0.3) verified while reviewing 7faabfe — single tSUS/tRS register, no HW SUS check, os_func reset points, self-test data-check trap
metadata:
  type: project
---

Verified in IDF 6.0.3 source (2026-11-12, review of flash-autosuspend commit 7faabfe):
- S3 has NO SOC_SPI_MEM_SUPPORT_CHECK_SUS: HW never verifies the chip actually suspended; it waits the fixed
  delay then reads. A chip that ignores 75h returns junk to the cache silently.
- S3 has one delay field (ctrl1.cs_hold_dly_res, x256 SPI_CLK, both per/pes_dly_256=1) for tSUS AND after-resume;
  TSUS=100 us at 80 MHz -> 32 units = 102.4 us. Kconfig max 100, so ZB25VQ32's 128 us resume->suspend is unreachable.
- With AUTO_SUSPEND=y, HW auto-suspend (pes_en) is armed in dev_config on EVERY op from the first read_id, and
  erase_sector skips poll_cmd_done (relies on HW auto-wait-idle) even if caches are disabled.
- chip->os_func is reset only by esp_flash_app_enable_os_functions (init_flash, before detect) and
  esp_flash_app_disable_protect (coredump -> noos, cache-suspending). esp_flash_set_dangerous_write_protection
  only touches os_func_data. So a probe-time os_func swap survives in normal operation.
- Hidden-prompt trap: driver *_ISR_CACHE_SAFE prompts are `if !SPI_FLASH_AUTO_SUSPEND`; ESP_HW_SUPPORT_FUNC_IN_IRAM
  defaults n under AUTO_SUSPEND (its selected children still default y individually).
- Self-test trap in 7faabfe: reader checksums only on full 256 KB window wrap (8192 reads x 50 us = 410 ms), so
  reads during a ~60-200 ms erase are never data-checked -> "fail(data)" unreachable; junk reads would PASS.
**How to apply:** any S3 auto-suspend validation must check per-read data during the erase, and treat tRS<spec as
unverified until measured. See [[project_rt_loop_latency]].
