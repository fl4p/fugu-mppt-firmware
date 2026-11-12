#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// True when the build has flash auto-suspend and the start/end guard is installed on the main flash.
bool fugu_flash_sus_built(void);

// on: flash ops keep the caches on and rely on hardware auto-suspend.
// off (boot default): flash ops disable caches and the other core, as without auto-suspend.
void fugu_flash_sus_set(bool on);
bool fugu_flash_sus_on(void);

// +1 at the start and at the end of each flash op (odd = op running)
extern volatile uint32_t fugu_flash_seq;

#ifdef __cplusplus
}
#endif
