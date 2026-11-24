#pragma once

#include <cstdint>

// On-device port of etc/measure_coil.py. Spawns a non-RT-core task that drives the converter through
// a duty (L0) or low-side-timing (rect_offset_ns) sweep and inverts the DCM transfer relation
// L = (Vin-Vout)*Vin*D^2 / (2*Vout*fsw*Iout). See website/docs/lab/coil-inductance.md.
//
// ls=false -> L0 duty sweep (arg1 = steps, 0 = default); ls=true -> LS-timing sweep (arg1 = hs,
// 0 = auto), dwellMs 0 = default (L0 1 s, LS 3 s). apply writes the result to coil.conf. fHz > 0: L0 sweep at
// that switching frequency, the previous one restored afterwards. Returns false if a run is already in progress.
bool measureCoilStart(bool ls, bool apply, int arg1, uint32_t dwellMs, uint32_t fHz = 0);

// lowest fHz [Hz]: the fixed bootstrap refresh pulse must still top up the HS supply above UVLO each period
constexpr uint32_t kMeasFswMin = 20000;

bool isMeasuring() ;