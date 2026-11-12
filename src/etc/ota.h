#pragma once

// Pull-and-flash a firmware image over HTTP(S). Returns false on failure; on success the device
// reboots and never returns.
bool doOta(const char *url);

// Pull a bootloader image over HTTP(S) into free flash, verify it and copy it to the bootloader
// region. convOff() must hold before the download and again before the copy. Returns nullptr on
// success (written or already identical), else the reason. Takes effect on the next reset.
// url == nullptr skips the download and installs what is already staged.
const char *doOtaBootloader(const char *url, bool (*convOff)());
