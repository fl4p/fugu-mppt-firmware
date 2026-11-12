#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// True when the build has flash auto-suspend and the start/end guard is installed on the main flash.
bool fugu_flash_sus_built(void);

// on: flash ops keep the caches on and rely on hardware auto-suspend.
// off (boot default): flash ops disable caches and the other core, as without auto-suspend.
void fugu_flash_sus_set(bool on);
bool fugu_flash_sus_on(void);

#ifdef __cplusplus
}
#endif
