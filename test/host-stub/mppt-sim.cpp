// Host closed-loop simulator: the real RT control path (ADC_Sampler + Sensor filters, MpptController
// protect()/update(), PD controllers, tracker, SynchronousConverter) driving the VirtualConverter
// plant through the firmware's own vconv ADC backend (adc/vconv.h) and PWM driver (pwm/vconv.h), as
// on a CONFIG_FUGU_WITH_VCONV build. loopRT/loopRTNewData/stopAndBackoff and the low-frequency
// control work of loopLF (lfControl, lfWatchdog, lfStuckWatchdog) are replicated from main.cpp.
// Time is simulated: every ADC timer tick (1 / vconv.conf::adc_freq, quantized to 1 MHz like the
// gptimer) advances esp_timer_get_time() and steps the plant once (ADC_VConv::hasData).
// Config: config/lab/vconv_mock/conf is copied to /tmp/mppt-sim-XXXXXX/littlefs/conf and read from
// there through the /littlefs fopen redirect (esp/sim_prelude.h); converter.conf mode psu -> mppt,
// per-scenario keys appended to vconv.conf / limits.conf. Stubs: test/host-stub/esp/.
//
//   test/host-stub/mppt-sim.sh [scenario ...]   # builds /tmp/mppt-sim.<checkout cksum> (or $OUT), runs the scenarios (default all)
//   /tmp/mppt-sim <scenario>   # sanity | a | b | c | d-cloud | d-cloud-ramp | d-bus04 | d-bus08 | d-bus12 |
//                              # d-noise | e | e-bleed; suffix -p0 = limits.conf reverse_current_paranoia=0
//                              # e*: battery open at 60 s, back at 180 s: on within 5 s ("== RECONNECT")
//                              # shade | shade-cv: half-shaded string (two peaks)
//                              # idle-wake: start refused only by InhibitIdle while Vin rises (armed sweep and not)
//                              # bat-auto: charger.conf vout_max=auto, 26 V pack: no switching before detection, then 8 cells
//                              #   (28.8 V) and MPPT; -nobat (output open, divider load: C_out decays from 27 V), -gap (17 V): never detected,
//                              #   never switching, also not with a manual duty from 60 s
//                              # bat-auto-swap: 8-cell pack charging, removed, a 4-cell pack: re-detected as 4 cells;
//                              #   bat-swap-explicit: the same with vout_max=29, the count untouched
//                              # psu-boost-dark: fry in boost, battery in, dark PV string out, `psu` 32..80 V (SIM_DWELL)
//                              # psu-boost-cold: the same, `psu 32` from the night state: string at 6.85 V, battery behind the open backflow switch
//                              # hs-short, hs-short-bfclosed, hs-cal, hs-dawn, hs-dawn-nozero: HS-short check
//                              # bf-ok, bf-open, bf-closed: `selftest bflow` (vconv.conf bflow_model=1)
//                              # bf-daily-flat: the daily run from MPPT at ~160 W in CCM on flat's plant
//                              # mc-flat: `measure-coil l0` entered from MPPT at ~160 W in CCM on flat's plant
//                              # mc-flat-lowf: the same at 22 kHz (the sweep after `f=`, not the switch to it)
//                              # mc-flat-ramp: the same entered by the `dc 0` ramp-down (SIM_MC_RAMP=1)
//                              # mc-flat-f20 / mc-boost-dark-f20: a 48.6 uH coil at 20 kHz (f= under the 1 ohm floor)
//                              # mc-flat-ton: the same with a plant HS on-time offset of -100 ns (vconv.conf t_on_ofs)
//                              # mc-boost-dark: `measure-coil l0` in boost at night (battery in, dark string out); -lowf at 21.5 kHz,
//                              #   -ton with a plant LS on-time offset of +100 ns and no coil DC bias
//                              # mc-boost-open: the same with the output unloaded, the 80 V trip
//                              # mc-boost-bfopen: the same with the backflow switch stuck open: the start aborts
//                              # dcd-stop-flat|fry: manual ramp-down from CCM that stops just below Vout/Voc (dcd-slow-*: 3x slower)
//                              # dcd-ccm-flat|fry: deep CCM, P&O and a slow power-limiter retreat
//                              # dcd-hv-step|train-flat|fry: 700-count duty cut at Vin ~80 V near the CCM boundary, one
//                              #   step / the manual ramp: LS to its minimum, no reverse coil current, LS back
//                              # hs-night-ov*, hs-night-race*: OV alert / core-0 disable() around the LS ramp and hold
//   weather scenarios (PV string of 1..3 substrings, irradiance played from a time series; "== SUMMARY" line):
//     wx-cloud          recorded cloudy quarter hour (flat, 2026-11-14 11:20 UTC), full sun <-> cloud
//     wx-cloud-n1       same with the single peak known (pv_substrings=1): no re-sweep after a flagged sweep
//     wx-cloud-cv       same with the battery at the Vout cap in full sun; -ripple adds 100 Hz bus ripple
//     wx-shade2, wx-shade3            static partial shade on a 6-substring string (2 modules x 3 bypass diodes)
//     wx-shade2-move, wx-shade3-move  shade moves, the global maximum changes peak after the boot sweep
//     wx-shade-walk                   a shadow walks across the substrings, the global peak moves several times
//     wx-shade-cloud    shaded substring under the cloud series (second peak comes and goes)
//     wx-shade-learn    wx-shade2 shade for two sweeps (learns the substrings), clear at 1900 s, back at 2000..2300 s
//     wx-shade-gen      SIM_SHADE=k,s,mode[,k2,s0,tr]: k substrings at s, k2 at (1 + s) / 2; mode a: shade comes over
//                       120..120 + tr s (tr 300), l: lifts, r: ramps from s0, s: static, u: unshaded boot, steps
//                       to s0 at 120 s and to s at 120 + tr s, v: as u with the k2 unshaded before 120 + tr s (SIM_P, default 300 W; T 1800 s); needs
//                       SIM_CONF=tracker.conf:pv_substrings=6 for scans; SIM_VOCDRIFT=d[,t1,t0]: Voc x (1 + d) from t0 (0) to t1 (T)
//     scan-static       no closed loop: scan probe placement against the exact plant's maxima over a shade grid,
//                       one STATIC line per case and share variant (SIM_STATIC_KDEF=1: Vmpp/Voc not learned)
//     every wx run with scans: one "== PROBE" line per scan (probes vs the plant's maxima and power at the scan start;
//     outcome from the maximum the operating point is on 5 s after the scan: ok / HIT / MISS-place / MISS-noise /
//     WRONG / PARTIAL / light), "== PROBES" totals; probe factors below / above
//     the operating point: SIM_SCAN_LOW, SIM_SCAN_UP (PeakScan::probeLow/High; SIM_SCAN_UP=0.97: the rule before 11-24), SIM_SCAN_HEAT (PeakScan::heatCut, 0: off)
//     wx-dawn, wx-dusk  irradiance through the start threshold; wx-dawn: no-MPP backoff doubles 30..300 s ("== NOMPP")
//     wx-low, wx-mid    steady light rising slowly, MPP 2 -> 20 W in 30 min / 55 -> 110 W in 10 min, sensor noise
//     wx-calfail        wx-mid with +-1.3 V bus ripple, every calibration fails: off 4 s per failure, the restart keeps the old zeros,
//                       calibrations keep the 30 min cadence ("== CALFAIL")
//     wx-low-slow       wx-low with power-sensor filter length 60 (slow sensor profile; SIM_ADC_HZ=200 for ADS-like rates)
//     wx-sweep-fall, wx-sweep-rise, wx-sweep-shade   cloud edge during a sweep (-shade: two peaks)
//     wx-sweep-gust, wx-sweep-learn[-off], wx-sweep-scan, wx-sweep-return   sweep verdict: one re-sweep (one maximum,
//                       no substring count), no learning from a flagged sweep (-off: sweep_verify=0 learns from it),
//                       scan on a flag, the return pass picking the better of two maxima after a cloud edge
//     wx-bms-late, wx-bms-none, wx-bms-full   boot sweep under the 26.8 V fallback cap before BMS cell data (SIM_BMS_AT,
//                       default 20 s): -late re-sweeps once the data lifts the cap, -none/-full keep the boot sweep ("== BMSCAP")
//     wx-bms-high       wx-bms-late with the pack (27.1 V) above the cap at boot: the cap cuts the boot sweep at 0 W; its
//                       no-MPP backoff ends when the BMS data lifts the cap, then a sweep finds the MPP ("== BMSCAP")
//     wx-sweep-preempt  a requested sweep pre-empts a scheduled re-sweep, which then does not count to the 10 min cap
//     wx-sweep-fall-n1, wx-sweep-rise-n1, wx-sweep-one   single peak known (pv_substrings=1): no re-sweep; -one learns
//                       it from two verified sweeps; "== VERDICT" PASS/FAIL
//     wx-ovl-*          output limiter vs a left-side operating point (#70), Iout limit lv_i_max, thermal power derate by
//                       chip temperature; "== OVL" PASS = no Iout-OC trip:
//       wx-ovl-shade      wx-shade3-move shade, lv_i_max 4.7 A: limited on the 33 V peak, the shade lifts (0.3 -> 0.8)
//                         until the valley passes the limit (reproduces #70, not in the default list)
//       wx-ovl-shade-temp wx-shade3-move shade, lv_i_max 6.5 A: on the 33 V peak (144 W), derate at 600 s to 100 W, below
//                         the valley (reproduces #70, not in the default list)
//       wx-ovl-shade-hold the same with the derate to 125 W, above the valley: held on the 33 V peak's right shoulder
//       wx-ovl-sun        1500 W string, 35 A: sun returns (0.25 -> 1 in SIM_OVL_EDGE s, default 2) from the dim MPP
//       wx-ovl-ctl        1500 W string in full sun: the sweep ends at the Iout limit, a derate to 400 W from 300 s, a
//                         sweep into it at 450 s
//       wx-ovl-bad        detector calibration, must FAIL: light 0.2 -> 1 in one tick
//       SIM_IOUT_MAX=<A> overrides lv_i_max
//     wx-mp-ramp, wx-mp-psu, wx-mp-psu-nosink  source Voc slowly 1.25 -> 1.02 -> 1.25 x Vbat (LS min pulse ratio
//                       limit): weak PV / stiff sinking source into a full battery / non-sinking bench PSU; the
//                       == REVERSE line has the plant's reverse output current and the starts
//     wx-mp-manual      the -psu source with manual duty 0.8 (stop, then the manual restart waits for the start ratio)
//     wx-mp-ccm         stiff source into a charging battery, Voc 1.3 -> 1.02 x Vbat: CCM until the duty limit, then DCM
//     wx-sun-fry, wx-sun-flat   steady sun, MPP SIM_P W (default 70), the board's L, string and Vin (fry ~64 V, flat ~55 V);
//                       SIM_SWEEP_AT=t requests a sweep at t [s] and starts the stats there
//     wx-dusk-land-fry  wx-sun-fry at dusk (6 W, Voc ~66 V there, Vmpp ~55 V, bat 25.8 V), a sweep at 100 s: the ramp
//                       outruns Vin through the input caps, the captured duty lies past the MPP (field fry 2026-11-18)
//     wx-ramp-fry, wx-ramp-flat wx-sun with irradiance changing SIM_RAMP %/min (default 2, exponential) from 120 s
//     wx-wob-fry, wx-wob-flat   wx-sun with irradiance x (1 + a sin 2 pi f t), SIM_WOB=a,f (default 0.005,0.2)
//     wx-dstep          static P(D): dusk light SIM_GC (0.039), duty SIM_DSTEP=d0,d1,step,dwell up and back, one DSTEP
//                       line per step (Vin, PV and measured power, LS); manual mode, or SIM_DSTEP_MPPT=1 for MPPT with
//                       the tracker frozen (diode emulation as in tracking)
//   SIM_NOISE=fry|flat|<file.fnz>  replay sensor noise measured on a converter (test/host-stub/noise/, made by
//     noise_profile.py; looped): residuals added to Vin/Vout/Iout, Iout's scaled by Iout/Iout(capture)^SIM_NOISE_EXP (default 1),
//     x SIM_NOISE_GAIN, starting SIM_NOISE_OFS s into the profile; INA226 LSB quantisation, the boards' filt_len 20, SIM_ADC_HZ defaults to the capture rate
//     fry-wander, flat-wander, fry3-wander: Iout keeps its slow wander below 0.3 Hz (noise_profile.py --keep-slow;
//     fry3: a capture with 3 % p-p at ~0.2 Hz). The == TRACKER line ends with the tracking loss in J, the part in slow
//     mode and the largest Vin offset from the MPP in slow mode
//     fry-inv, var-inv, china-inv, pv8a-inv, heater-inv: inverter on the DC bus (2x-line ripple; china-inv and
//     pv8a-inv spiky with aliased harmonics, heater-inv a 500 W heater switching too), made by inverter_profiles.py
//   ripple filter: SIM_CONF=sensor.conf:ripple_filter=notch|comb|both; SIM_NOTCH_DB=-15 sets the notch depth (the
//     depth every build before the gainAmp fix had); bus ripple on any scenario with
//     SIM_CONF=vconv.conf:vbat_ac_amp=0.26,vconv.conf:vbat_ac_freq=100,vconv.conf:vbat_ac_shape=0|1|2 (sine, |sin|,
//     spiky); SIM_RIPPLE_DRIFT=dHz,period swings its frequency +-dHz with that period [s] (off-grid inverter)
//   despike: SIM_CONF=sensor.conf:despike=8 (fry/flat run 8; the sim default is the conf default 0)
//   SIM_T=600 wx run length [s], SIM_TR=5 wx trace interval [s], SIM_V=1 firmware log in wx/shade runs
//   SIM_KEEP_FS=1 keeps the temp littlefs dir (/tmp/mppt-sim-XXXXXX), removed at exit otherwise
//   SIM_FINE=t0,t1 /tmp/mppt-sim c              # per-tick sensor trace between t0 and t1 [s]
//   SIM_TTRACE=t0,t1 /tmp/mppt-sim wx-dusk      # one line per tracker decision (duty, Vin, P, direction, LS) in t0..t1
//   SIM_PDUMP=<file>  per RT tick float32 t, Iout ewm, Vout ewm, HS count, slow mode, PV power; SIM_HOLD=t: wx runs hold the duty from t [s]
//   SIM_DC=ta,duty,tb  wx runs: console `dc duty` at ta [s], `mppt` at tb [s]
//   SIM_ADC_HZ=3000 /tmp/mppt-sim a             # ADC tick rate [Hz], default 450
//   SIM_PHASE_US=500 /tmp/mppt-sim a          # a/b/c/e: plant runs n us past the event before the next ADC tick
//   SIM_CONF="coil.conf:L0=80e-6,vconv.conf:plant_c_out=470e-6" /tmp/mppt-sim a   # board overrides
//   rect-cal (rectCalRt/rectCalLf wired as in main.cpp; the automatic start is off, SIM_RCAL_AUTO=1 enables it):
//     rcal         `rect-cal force` at full power (CCM), LS body-diode drop 0.7 V, sensor noise
//     rcal-bus     same, battery +2.5 % linear over 30 s from the start (shared bus)
//     rcal-cloud   same, Isc x0.7 and Voc -1.8 % step 12 s after the start
//     rcal-ov      same, Vin trip (hv_max) 2 % above Voc: the run must stop short of it
//     rcal-low     same, a cloud 0.2 s after the start: Isc 2 A, Voc -3 % (soft panel, knee lifts Vin)
//     rcal-sun     80 uH board, Isc 5.6 A / Voc 79.6 V, 5 nF switch node ringing at Q 40 (non-monotonic late side)
//     rcal-flat    flat (51 uH, 326 kHz ring, 1470 sps), ~390 W at Vin ~55 V, Vout +0.6 V for 2 ms every 1-3 s:
//                  D-only Vout limiter touches below the cap must not abort the run
//     rcal-pv      plain rect-cal force on the SIM_RING board plant (default fry), panel SIM_PV="isc,voc,k" (default
//                  5.6,79.6,0.78): k moves the start Vin, isc the power, voc the calibration duty
//     rcal*: SIM_NOISE_IOUT=<A> Iout noise sigma (default 0.01); the plant's zero crossing at the calibration point
//     ("truth": LS count where the coil current at LS turn-off crosses zero, stiff sources at the run's mean plant
//     Vin/Vout, so dead time, diode drop and the ring current at HS turn-on are in it) is printed against ideal
//     rcal-low-bus rcal-low on a swinging bus (+-0.4 % / 23 s, +-0.2 % / 7.3 s, 0.15 % steps every 2-6 s)
//   switch-node ring (DCM Iout ripple vs duty; vconv.conf ring_c/ring_q), board = fry | flat (L0, L, ring f, Q
//   fitted to field sweeps), suffix -off = same board with the ring disabled, -damped = ring at Q 0.5 (no
//   oscillation, but the ring branch: node charge, and the HS body diode returns clamp charge to the input):
//     ring-sweep-<board>   ~150-180 W morning sun, global sweep at 60 s: ripple seen in the sweep, chosen peak
//     ring-weak-<board>    steady ~45 W, the whole operating range in DCM: does P&O sit on a ripple maximum
//     ring-coil-<board>    `measure-coil l0` flow (old 10-step median, new 48-step ring fit) against the plant,
//                          then the MPPT restore; -weak: at ~40 W (soft source)
//   SIM_RING=fry|flat|fry-hq|fry-off|flat-damped|... applies a board plant (test/host-stub/plant_boards.h: ring, diode
//     drop, dead time, conduction and switching loss, fitted to the LTspice bench) to any other scenario; wx-* default
//     to fry, SIM_RING=ideal = the lossless vconv_mock plant without ring. -hq: Q of the undamped bench (more ripple)
//     Board presets run the field boards' firmware timing: MCPWM 160 MHz (4103 counts at 39 kHz, minLS 320),
//     rect_offset_ns 0 (fry's live -1312: SIM_PLANT=fry:rect_offset_ns=-1312; never in rcal*, which start
//     uncalibrated). -ledc anywhere in the board tag (fry-ledc, fry-hq-ledc, fry-ledc-off) or SIM_DRV=ledc: the LEDC
//     timing (2047 counts, rect_offset 0) of every result before 2026-11-19; SIM_DRV=mcpwm also on the ideal plant
//   SIM_PLANT="fry:ring_q=40,fry:rect_offset_ns=0,..." overrides board fields
//   top of charge on a shared bus ("== CVHOLD" PASS/FAIL line), fry's board, charger.conf as on fry/flat (cv_eoc 3.5,
//   ceiling 3.55, 280 Ah, ibat_max 40, cv_hold_max_s 5400) and a balance run from the start. Unit 1 is the firmware,
//   unit 2 a second BatteryCharger driving a current source; one 8s pack (lfp_pack.h topOfCharge(), the 11-23 fit:
//   R0 2.1 mOhm, flat OCV, a 410 s polarization that grows with the charge in, so 3.500 V takes ~7 A tapering over
//   hours; the high cell 17 Ah ahead of the others) behind 20 mOhm of cable, a house load of SIM_BASE_A (2 A; field
//   median 2-4 A, 5th percentile 0.4 A) on the bus, one BMS feeding both chargers (ibat every 4 s, highest cell every
//   12 s, 1 mV); PV SIM_PV_W per unit (560):
//     cv-hold-pair     both units at ~20 A into cv_eoc: peak <= cv_eoc + 15 mV to hold + 300 s, then +-10 mV, no latch
//     cv-hold-offset   unit 2's Vout reads 0 / -0.15 / -0.9 V idle / both converting / alone, 90 min
//     cv-hold-gap      no BMS frames 1800..2000 s: both units pause on the stale data and run again within 60 s
//     cv-hold-load     900 W on 30 s / off 30 s from 1200 s: cell <= cv_eoc + 15 mV
//     cv-hold-reboot   unit 2 restarts at 1200 s
//     cv-hold-mask     a 1080 W load (with the base) for 900 s takes the PV current (ibat ~1 A): the line crosses low,
//                      no hold below the band
//     cv-hold-day      a day from ~80 % SoC ("== CVDAY" PASS/FAIL line; no balance run, the pack 70 Ah below the fit's
//                      start): clouds to 10..20 % for 1..5 min and 900 W loads for 1..5 min until the
//                      cell reaches the approach band, then clear sun to the latch (~8900 s simulated, ~3 min on an idle
//                      core). Events on the pack: a 200 s BMS gap at -62 Ah; at -56 Ah a 300 s cloud (20 %) with
//                      a 180 W load (with the base; ibat ~1 A) and unit 2 restarting 30 s into it (flat 2026-11-24);
//                      unit 2 restarts again
//                      when the BMS cell reaches cv_eoc - 25 mV. Checks: no hold or latch with the BMS cell below cv_eoc -
//                      cv_hold_band (fails on charger.h before bd9581b7: unit 2 holds at ~3.37 V), a hold, the cell <=
//                      cv_eoc + 15 mV from the first hold on (the cv-hold-pair/-load tolerance), both units latched or in
//                      a hold shorter than cv_hold_max_s at the end; reports Ah/Wh into the pack and the time at the top
//     all: no ceiling latch, the plant's highest cell stays below cv_ceiling. Unit 2's output current follows
//     min(its PV current, Iout_max(), the current that puts its sensed Vout at Vout_max()) with a 0.3 s lag: no
//     tracking loss, PD loop or diode emulation. Its charger updates every 3 s with main.cpp's authority (running and
//     iout > 0.5 A or holding). A restart: charger rebuilt 5 s later (state from its ChargerRtc), BMS frames from 10 s,
//     starts as loopRTNewData (not full; BMS data decided or a 12 s wait), then ramps to its PV current over 5 s
//     (the sweep). Vout error: linear in 0..2 A own and 0..5 A sibling current. Overrides:
//     SIM_VOUT_ERR=u:idle,both,alone[;u:...]  SIM_LOAD=W,on_s,off_s,from_s (W,1,0,0: constant)  SIM_REBOOT_AT=t
//     SIM_BMS_GAP=t,len[,t2,len2]  SIM_BMS=ibat_phase,cell_phase,lag (lag 1: ibat one frame late)  SIM_BASE_A=A  SIM_T
//     SIM_TR  SIM_PACK_RP=x (scales the pack's polarization, lfp_pack.h topOfCharge())
//
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <filesystem>
#include <functional>
#include <optional>
#include <random>
#include <unistd.h>

#include "adc/sampling.h"
#include "adc/sensor_setup.h"
#include "buck.h"
#include "mppt.h"
#include "app_state.h"
#include "sim/vconv.h"
#include "sim_alloc.h"
#include "selftest/coil_fit.h"
#include "selftest/bflow_test.h"
#include "plant_boards.h"
#include "lfp_pack.h"

#if __has_include("peak_scan.h")
#define SCAN_ACTIVE() mppt.scan.active()
#define SCAN_ACTIVE_DEFINED 1
#else
#define SCAN_ACTIVE() false // a tree without the partial scan
#define SCAN_ACTIVE_DEFINED 0
#endif

// ---- globals owned by main.cpp on target ----
ADC_Sampler adcSampler{};
VIinVout<const Sensor *> sensors{nullptr, nullptr, nullptr, nullptr};
SynchronousConverter converter;
static LCD simLcd;
MpptController mppt{adcSampler, sensors, converter, simLcd};
time_us loopWallClockUs_ = 0;
volatile uint32_t loopWallTicks_ = 0;
time_us lastLoopTime = 0, timeLastSampler = 0, delayStartUntil = 0, lastTimeOutUs = 0, lastLfWorkUs = 0;
uint32_t lastNSamples = 0;
unsigned long lastMpptUpdateNumSamples = 0;
AppState g_app{};
static const auto lfPeriod = 3000000ULL;

extern std::string g_simFsRoot;
extern uint32_t g_simTickUs;
extern void (*g_simLogHook)(char lvl, const char *tag, const char *msg);
extern float g_simChipTemp;

// ---- trace / event capture ----
struct Trip {
    double t;
    std::string who;
    unsigned sec;
};
static std::vector<Trip> g_trips;
static std::vector<double> g_sweepT; // "Start sweep" log times
static std::vector<double> g_calFailT, g_onT; // "Calibration failed" log times, converter off -> on edges
static unsigned g_calStarts = 0;
static std::vector<std::string> g_logSince; // log lines after the scenario event
static double g_eventT = -1;
static std::vector<double> g_slowDownT; // "Slow mode, duty" log times (slow-mode entries)
static std::vector<std::pair<double, char>> g_scanT; // "scan:" log times; m = moved, l = light changed, - = stayed
static char g_scanEnd = 0; // last "scan: P" line: m, l, -; 0 = consumed
static float g_scanBestV = 0;
// sweep verdict events: v verified, f flagged, F flagged + scan, p return-pass probe, r re-sweep scheduled, L learned,
// 1 single peak learned, s scan
static std::string g_verdict;
struct SweepStop {
    double t;
    float p, v;
};
static std::vector<SweepStop> g_stops; // "Stop sweep" lines with an MPP
static double g_fineT0 = 0, g_fineT1 = 0; // env SIM_FINE=t0,t1: per-tick sensor trace window
static double g_trkT0 = 0, g_trkT1 = 0; // env SIM_TTRACE=t0,t1: one line per tracker decision
static double g_rampSec = 0, g_rampT0 = 0, g_rampFrom = 0, g_rampTo = 0; // linear Isc ramp
static bool g_verbose = true;
static double g_qOut = 0; // plant output charge [C], per RT iteration (cycle-average Iout x dt)
static unsigned g_nStarts = 0; // converter off -> on edges

// ---- board plant (ring-* scenarios, SIM_RING; wx-* default to fry) ----
static PlantBoard g_board;
static const PlantBoard *g_ring = nullptr;
static bool g_ringOn = false, g_ringDamped = false; // damped: Q 0.5, ring branch without oscillation
// gate-driver timing of the board presets: MCPWM 160 MHz as live (4103 counts at 39 kHz), -ledc / SIM_DRV=ledc: the
// LEDC timing of the results before 2026-11-19 (2047 counts, rect_offset 0); SIM_DRV=mcpwm also on the ideal plant
static bool g_drvMcpwm = false, g_liveOffset = true; // rcal* start uncalibrated (no board rect_offset)
static bool g_batAuto = false; // bat-auto*: charger.conf vout_max=auto (vout_max_fallback dropped), board.conf bflow_bat=1
static void (*g_tickHook)() = nullptr; // called after every RT iteration of run()
static void (*g_afterHsHook)() = nullptr; // called in loopRTNewData right after hsShortRt (an ISR landing there)
static FILE *g_pDump = nullptr; // SIM_PDUMP=<file>: per tick t, Iout ewm, Vout ewm, HS count (float32)

static bool cutSuffix(std::string &t, const char *sfx) {
    const size_t n = strlen(sfx);
    if (t.size() <= n || t.compare(t.size() - n, n, sfx)) return false;
    t.resize(t.size() - n);
    return true;
}

// "...-fry[-hq][-off]" / "...-flat[-damped]" or SIM_RING: pick the board; returns the name without the suffix
static std::string ringBoardFrom(std::string sc) {
    plantBoardsFromEnv();
    const char *env = getenv("SIM_RING");
    std::string tag = sc;
    if (sc.rfind("ring-", 0) != 0) {
        if (!env || !strcmp(env, "ideal")) return sc;
        tag = std::string("x-") + env;
    }
    const size_t lp = tag.find("-ledc"); // anywhere in the board tag: fry-ledc, fry-hq-ledc, fry-ledc-off
    const bool ledc = lp != std::string::npos;
    if (ledc) tag.erase(lp, 5);
    g_ringOn = !cutSuffix(tag, "-off");
    g_ringDamped = cutSuffix(tag, "-damped");
    const bool hq = cutSuffix(tag, "-hq");
    for (auto &b: kPlantBoards) {
        const std::string sfx = std::string("-") + b.name;
        if (cutSuffix(tag, sfx.c_str())) {
            g_board = hq ? highQ(b) : b;
            if (g_ringDamped) g_board.ringQ = 0.5f;
            g_ring = &g_board;
            g_drvMcpwm = !ledc;
            if (sc.rfind("ring-", 0) == 0) sc = tag;
            return sc;
        }
    }
    printf("ring board? %s\n", tag.c_str());
    exit(1);
}

static double tNow() { return (double) esp_timer_get_time() * 1e-6; }

// ---- measured sensor noise (SIM_NOISE) ----
float (*g_simSenseHook)(uint8_t ch, float x) = nullptr;
bool g_simAdcMute = false;
static struct {
    float fs[3], lsb[3], iRef = 1, gain = 1, exp = 1;
    uint64_t ofs = 0; // ticks
    std::vector<int16_t> v[3]; // vin, vout, iout (ADC_VConv channels 0..2)
} g_noise;

static float noiseHook(uint8_t ch, float x) {
    if (ch > 2) return x;
    const auto &v = g_noise.v[ch];
    // by ADC tick: a channel at the tick rate replays sample for sample (a skipped or repeated sample is a
    // phase step in the ripple, which the notch passes)
    const uint64_t tick = esp_timer_get_time() / g_simTickUs + g_noise.ofs;
    const float k = g_noise.fs[ch] * g_simTickUs * 1e-6f;
    const size_t i = (size_t) (std::fabs(k - 1) < 0.01f ? tick : (uint64_t) ((double) tick * k)) % v.size();
    float r = (float) v[i] * g_noise.lsb[ch] * g_noise.gain;
    if (ch == 0) return x + r;
    if (ch == 2) r *= std::pow(std::max(x, 0.f) / g_noise.iRef, g_noise.exp);
    const float q = ch == 1 ? 0.0025f : 35.f / 32768; // INA226 bus (x2 divider) and current LSB
    return std::round((x + r) / q) * q;
}

static void noiseLoad(const char *name) {
    std::string p = name;
    if (p.find('/') == std::string::npos) p = std::string(SIM_REPO_ROOT) + "/test/host-stub/noise/" + p + ".fnz";
    FILE *f = ::fopen(p.c_str(), "rb");
    char magic[4];
    if (!f || fread(magic, 1, 4, f) != 4 || memcmp(magic, "FNZ1", 4) || fread(&g_noise.iRef, 4, 1, f) != 1) {
        printf("SIM_NOISE: cannot read %s\n", p.c_str());
        exit(1);
    }
    for (int c = 0; c < 3; ++c) {
        uint32_t n = 0;
        if (fread(&g_noise.fs[c], 4, 1, f) != 1 || fread(&g_noise.lsb[c], 4, 1, f) != 1 || fread(&n, 4, 1, f) != 1 || !n) exit(1);
        g_noise.v[c].resize(n);
        if (fread(g_noise.v[c].data(), 2, n, f) != n) exit(1);
    }
    fclose(f);
    if (const char *e = getenv("SIM_NOISE_GAIN")) g_noise.gain = atof(e);
    if (const char *e = getenv("SIM_NOISE_EXP")) g_noise.exp = atof(e);
    if (const char *e = getenv("SIM_NOISE_OFS")) g_noise.ofs = (uint64_t) (atof(e) * g_noise.fs[2]);
    if (!getenv("SIM_ADC_HZ")) setenv("SIM_ADC_HZ", std::to_string(std::lround(g_noise.fs[2])).c_str(), 1);
    g_simSenseHook = noiseHook;
    printf("# noise %s: %.0f s at %.0f Hz, Iout ref %.2f A, gain %.2f, exp %.2f\n", p.c_str(),
           g_noise.v[2].size() / g_noise.fs[2], g_noise.fs[2], g_noise.iRef, g_noise.gain, g_noise.exp);
}

static bool g_clampIgnoredLog = false;
static int g_psuTrips = 0; // PSU fast-retry / escalated / latch log lines (counted shutdownDcdc trips)
static int g_lsMinN = 0; // "-> pwmRectMin" log lines: a duty decrease dropped the LS to its minimum
static bool g_ioTrack = false; // track g_ioMin until the converter turns off
// min plant cycle-mean output current while tracked: g_ioMin with an LS window wider than its minimum + T/64 (the
// min pulse alone leaves ~V_o*t/L of reverse current near zero load), g_ioMin0 any
static float g_ioMin = 0, g_ioMin0 = 0;
static uint16_t g_ioMinD = 0, g_ioMinLs = 0; // HS / LS counts at g_ioMin
static void ioMinTick() {
    if (converter.disabled()) g_ioTrack = false;
    if (!g_ioTrack) return;
    const float i = g_vconv.getIoutAvg();
    g_ioMin0 = std::min(g_ioMin0, i);
    if (i < g_ioMin && converter.getRectOnPwmCnt() > converter.getRectOnPwmMin() + converter.pwmMaxDriver() / 64)
        g_ioMin = i, g_ioMinD = converter.getCtrlOnPwmCnt(), g_ioMinLs = converter.getRectOnPwmCnt();
}

static void logHook(char lvl, const char *tag, const char *msg) {
    if (strstr(msg, "hs_clamp ignored")) g_clampIgnoredLog = true;
    if (strstr(msg, "-> pwmRectMin=")) ++g_lsMinN;
    // backoff log of MpptController::shutdownDcdc: "backoff %lus [%s]..."
    unsigned sec;
    char who[64];
    if (sscanf(msg, "backoff %us [%63[^]]]", &sec, who) == 2) g_trips.push_back({tNow(), who, sec});
    if (!strncmp(msg, "PSU fast retry [", 16) || !strncmp(msg, "PSU escalated backoff", 21) || !strncmp(msg, "PSU latch [", 11))
        ++g_psuTrips;
    if (!strcmp(msg, "Start sweep")) g_sweepT.push_back(tNow());
    if (const char *m = strncmp(msg, "Stop sweep ", 11) ? nullptr : strstr(msg, "MPP=(")) {
        float p, v;
        unsigned d;
        if (sscanf(m, "MPP=(%fW,%u,%fV", &p, &d, &v) == 3) g_stops.push_back({tNow(), p, v});
    }
    if (!strncmp(msg, "Calibration failed", 18)) g_calFailT.push_back(tNow());
    if (!strcmp(msg, "Start calibration")) ++g_calStarts;
    if (!strncmp(msg, "Slow mode, duty", 15)) g_slowDownT.push_back(tNow());
    if (!strncmp(msg, "sweep verified", 14)) g_verdict += 'v';
    if (!strncmp(msg, "sweep flagged", 13)) g_verdict += strstr(msg, "scan now") ? 'F' : 'f';
    if (!strncmp(msg, "sweep return:", 13) && strstr(msg, "down")) g_verdict += 'p';
    if (!strncmp(msg, "re-sweep in", 11)) g_verdict += 'r';
    if (strstr(msg, "learned from the sweep")) g_verdict += 'L';
    if (strstr(msg, "single peak learned")) g_verdict += '1';
    if (!strncmp(msg, "scan: P", 7)) g_verdict += 's';
    if (!strncmp(msg, "scan: P", 7)) {
        const char *b = strstr(msg, " at ");
        b = b ? strstr(b + 4, " at ") : nullptr;
        g_scanBestV = b ? (float) atof(b + 4) : 0;
        g_scanEnd = strstr(msg, "moving") ? 'm' : strstr(msg, "light changed") ? 'l' : '-';
    }
    if (!strncmp(msg, "scan:", 5))
        g_scanT.emplace_back(tNow(), strstr(msg, "moving") ? 'm' : strstr(msg, "light changed") ? 'l' : '-');
    char line[640];
    snprintf(line, sizeof line, "%9.4f %c %-8s %s", tNow(), lvl, tag, msg);
    if (g_eventT >= 0 && g_logSince.size() < 400) g_logSince.emplace_back(line);
    // rate-limit the per-tick chatter ("Limiting!", "Current above/below threshold")
    static double winT = -1;
    static int winN = 0;
    if (tNow() - winT > 1.0) { winT = tNow(); winN = 0; }
    if (g_verbose && (lvl != 'U' || ++winN <= 6)) printf("%s\n", line);
}

// ---- replicated main.cpp RT path ----
void systemRestart() {
    printf("%9.4f !!! systemRestart() requested (not modelled, sim continues)\n", tNow());
}

void stopAndBackoff(uint32_t secondsDelay, const char *who = "stopAndBackoff") {
    const bool psuFast = g_app.psuMode() && secondsDelay <= 5;
    if (!converter.disabled() || mppt.isSweeping())
        mppt.shutdownDcdc(who, psuFast ? 0 : std::min<uint32_t>(secondsDelay, 5));
    delayStartUntil = wallClockUs() + (psuFast ? 100000ULL : static_cast<time_us>(secondsDelay) * 1000000ULL);
}

static void loopRTNewData(time_ms nowMs) {
    auto nSamples = sensors.Vout->numSamples;
    bool haveNewSample = (nSamples - lastMpptUpdateNumSamples) > 0;
    mppt.applyPendingPsuCommandRt(haveNewSample);
    if (haveNewSample) {
        timeLastSampler = wallClockUs();
        mppt.rectCalRt();
        mppt.hsShortRt((uint32_t) nowMs);
        if (g_afterHsHook) g_afterHsHook();
    }

    if (mppt.converter.disabled() && (!mppt.hsFaultState() || mppt.hsFaultState() == MpptController::HsOff))
        g_hwOvTrip = false;

    if (unlikely(adcSampler.isCalibrating())) {
        mppt.shutdownDcdc("calib", 0);
    } else {
        if (mppt.active() or g_app.manualPwm()) {
            bool mppt_ok = true;
            if (!mppt.converter.disabled()) {
                mppt_ok &= mppt.protect(g_app.manualPwm());
                mppt_ok &= mppt.protectLf(g_app.manualPwm());
            }
            if (mppt_ok) {
                if (haveNewSample) {
                    if (!g_app.manualPwm()) mppt.update();
                    else mppt.updateManual();
                    lastMpptUpdateNumSamples = nSamples;
                }
            } else {
                stopAndBackoff(4);
            }
        } else if (wallClockUs() > delayStartUntil && !mppt.startBlockReasonRt()) {
            if (g_app.psuMode()) {
                mppt.psuArm();
                delayStartUntil = wallClockUs() + 4 * 1000000ULL;
                ESP_LOGI("mppt", "PSU: arming converter from cold start");
            } else if (!g_app.manualPwm()) {
                static int64_t bmsBootDeadline = 0;
                static int64_t bmsRearmAllowedUs = 0;
                static bool bmsWasFresh = false;
                constexpr int64_t BMS_BOOT_WAIT_US = 12'000'000;
                constexpr int64_t BMS_REARM_MIN_US = 10ll * 60 * 1000000;
                bool bmsFresh = mppt.charger.batSt.haveValidCellVoltage();
                bool full = bool(mppt.charger.termCond) && bmsFresh;
                bool waitingForBms = mppt.charger.hasBmsCellSource()
                                     && (!mppt.charger.terminationDecided() || !bmsFresh);
                if (bmsWasFresh && !bmsFresh && esp_timer_get_time() > bmsRearmAllowedUs) {
                    bmsBootDeadline = 0;
                    bmsRearmAllowedUs = esp_timer_get_time() + BMS_REARM_MIN_US;
                }
                bmsWasFresh = bmsFresh;
                if (waitingForBms && bmsBootDeadline == 0)
                    bmsBootDeadline = esp_timer_get_time() + BMS_BOOT_WAIT_US;
                bool bmsWaitElapsed = bmsBootDeadline != 0 && esp_timer_get_time() > bmsBootDeadline;
                if (!full && (!waitingForBms || bmsWaitElapsed)) {
                    mppt.startSweep();
                    delayStartUntil = wallClockUs() + 4 * 1000000ULL;
                }
            }
        }
        if (haveNewSample) {
            auto lv = converter.boost() ? sensors.Vin : sensors.Vout;
            mppt.lvPgood.update(lv->med3.get(), lv->ewm.avg.get(), lv->numSamples, !mppt.inBackoff(),
                                (uint32_t) nowMs);
        }
    }
    if (g_app.manualPwm()) {
        if (!converter.disabled()) converter.pwmPerturb(0);
    }
}

// one iteration of loopRT's while(true) body
static void loopRTIterBody() {
    setWallClockUs(esp_timer_get_time());
    auto &nowUs(loopWallClockUs_);
    time_ms nowMs = nowUs / 1000ULL;

    if (unlikely(adcSampler.adcResetPending())) {
        if (adcSampler.halted) {
            adcSampler.dropAdcReset();
        } else {
            stopAndBackoff(6);
            bool ok = adcSampler.applyAdcResetRt();
            setWallClockUs(esp_timer_get_time());
            stopAndBackoff(ok ? 6 : 30);
            if (timeLastSampler) timeLastSampler = nowUs;
        }
    }

    auto samplerRet = adcSampler.update();
    converter.applyPendingDeadTimeRt();
    mppt.applyPendingPwmFreqRt();
    if (adcSampler.halted) return;
    if (samplerRet == ADC_Sampler::UpdateRet::CalibFailure)
        stopAndBackoff(4, "calib-fail");

    constexpr time_ms kAdcResetThrottleMs = 300, kAdcSustainedMs = 800, kAdcRestartMs = 60000;
    static time_ms adcStalledSinceMs = 0, lastAdcResetMs = 0;
    bool adcStalled = samplerRet == ADC_Sampler::UpdateRet::AdcError ||
                      (samplerRet != ADC_Sampler::UpdateRet::NewData && !adcSampler.isCalibrating() &&
                       timeLastSampler && nowUs - timeLastSampler > 200000);
    if (adcStalled) {
        if (!adcStalledSinceMs) adcStalledSinceMs = nowMs;
        mppt.lvPgood.release();
        if (nowMs - lastAdcResetMs > kAdcResetThrottleMs) {
            lastAdcResetMs = nowMs;
            adcSampler.resetPeripherals();
        }
        if (nowMs - adcStalledSinceMs > kAdcSustainedMs && !converter.disabled() && !mppt.inBackoff()) {
            ESP_LOGE("main", "ADC stall %llu ms, shutdown (nSamples=%lu)", nowMs - adcStalledSinceMs,
                     lastMpptUpdateNumSamples);
            stopAndBackoff(16);
        }
        if (nowMs - adcStalledSinceMs > kAdcRestartMs) systemRestart();
    } else {
        adcStalledSinceMs = 0;
    }

    if (samplerRet != ADC_Sampler::UpdateRet::NewData) {
        mppt.applyPendingPsuCommandRt(false);
        if (adcSampler.isCalibrating() && mppt.boardPowerSupplyUnderVoltage()) {
            ESP_LOGW("main", "Board power supply UV %.2f!", mppt.boardPowerSupplyVoltage());
            adcSampler.cancelCalibration();
        }
        if (!timeLastSampler and nowMs > 20000) {
            converter.disable();
            ESP_LOGE("main", "Never got a sample! Please check ADC");
        }
    } else {
        loopRTNewData(nowMs);
    }

    auto lag = nowUs - lastLoopTime;
    if (lastLoopTime && lag > g_app.maxLoopLag && !converter.disabled()) g_app.maxLoopLag = lag;
    lastLoopTime = nowUs;
}

// SIM_NOALLOC=<s>: heap use inside the RT iteration after <s> simulated seconds fails the run
static double g_noAllocAfter = -1;

static void loopRTIter() {
    g_simRtAlloc.armed = g_noAllocAfter >= 0 && tNow() >= g_noAllocAfter;
    g_simRtAlloc.active = true;
    loopRTIterBody();
    g_simRtAlloc.active = false;
}

// ---- replicated loopLF control work (core 0 on target, every lfPeriod) ----
static void lfWatchdog(time_us nowUs, uint32_t dt, uint32_t sps, uint32_t nSamples) {
    static uint8_t starvedWindows = 0;
    constexpr uint8_t TRIP_WINDOWS = 3;
    bool starved = (dt > (lfPeriod * 0.9f)) && sps < g_app.loopRateMin && !converter.disabled() &&
                   nSamples > max(g_app.loopRateMin * 5, 200) &&
                   !g_app.manualPwm() && lastTimeOutUs && (nowUs - adcSampler.getTimeLastCalibrationUs()) > 2000000;
    if (!starved) { starvedWindows = 0; return; }
    if (++starvedWindows < TRIP_WINDOWS) {
        ESP_LOGW("main", "Loop latency high (%lu<%hu Hz), window %d/%d", (unsigned long) sps, g_app.loopRateMin,
                 (int) starvedWindows, (int) TRIP_WINDOWS);
        return;
    }
    starvedWindows = 0;
    ESP_LOGE("main", "Loop latency high (%lu<%hu Hz), shutdown!", (unsigned long) sps, g_app.loopRateMin);
    stopAndBackoff(4);
}

// vout_max=auto pending (an older tree has no auto)
template<class P>
static bool awaitingPack(const P &p) {
    if constexpr (requires { p.awaitingDetect(); }) return p.awaitingDetect();
    return false;
}

static void lfStuckWatchdog() {
    static int64_t stuckSinceUs = 0;
    static bool triedRelease = false;
    constexpr int64_t RELEASE_US = 30ll * 1000000ll;
    constexpr int64_t TIMEOUT_US = 2ll * 3600ll * 1000000ll;
    const int64_t nowUs = esp_timer_get_time();
    bool headroom = sensors.Vin && sensors.Vout
                    && (converter.boost()
                            ? sensors.Vin->ewm.avg.get() > mppt.limits.Vin_min + 2.0f
                            : sensors.Vin->ewm.avg.get() > sensors.Vout->ewm.avg.get() + 8.0f);
    bool noPower = sensors.Iout && fabsf(sensors.Iout->ewm.avg.get()) < 0.3f;
    const float psuSetpoint = mppt.getPsuSetpoint();
    bool stuck = headroom && noPower && !g_app.manualPwm() && !mppt.noMppBackoff() && !awaitingPack(mppt.charger.params)
                 && (g_app.psuMode()
                         ? (std::isfinite(psuSetpoint) && !mppt.isPsuLatched() && sensors.Vout->ewm.avg.get() < psuSetpoint - 2.0f)
                         : !mppt.charger.chargeHold());
    if (!stuck) { stuckSinceUs = 0; triedRelease = false; return; }
    if (!stuckSinceUs) { stuckSinceUs = nowUs; return; }
    if (!triedRelease && (nowUs - stuckSinceUs) >= RELEASE_US) {
        triedRelease = true;
        if (!g_app.psuMode() && mppt.releaseCvFloorLatch("stuck watchdog")) {
            ESP_LOGW("main", "stuck watchdog: released CV-floor latch in place, retrying");
            return;
        }
    }
    if ((nowUs - stuckSinceUs) < TIMEOUT_US) return;
    ESP_LOGE("main", "No output despite headroom, restarting");
    systemRestart();
}

// main.cpp's charger.update() call; an older charger.h (sensitivity runs) has no sweep argument
template<class C>
static void chargerUpdate(C &c, float vout, float iout, bool authority, bool psu, bool sweep) {
    if constexpr (requires { c.update(vout, iout, authority, psu, sweep); }) c.update(vout, iout, authority, psu, sweep);
    else c.update(vout, iout, authority, psu);
}

// main.cpp's vout_max=auto detection; true while the charger waits for it (an older tree has none)
template<class M>
static bool batDetect(M &m) {
    if constexpr (requires { m.batDetectLf(); }) {
        m.batDetectLf();
        return awaitingPack(m.charger.params);
    }
    return false;
}

static void lfControl() {
    mppt.ntc.read();
    mppt.ucTemp.read();
    if (sensors.Vout && sensors.Iout) {
        const float vout = sensors.Vout->ewm.avg.get();
        const float iout = sensors.Iout->ewm.avg.get();
        uint16_t pwmMargin = mppt.converter.pwmCtrlMax / 256;
        if (pwmMargin < 8) pwmMargin = 8;
        const uint16_t minAuthorityPwm = mppt.converter.getCtrlOnPwmMin() + pwmMargin;
        const bool currentShowsAuthority = std::isfinite(iout) && (iout > 0.5f || mppt.charger.chargeHold());
        const bool voutAuthority = !mppt.converter.disabled()
                                   && mppt.converter.getCtrlOnPwmCnt() > minAuthorityPwm
                                   && currentShowsAuthority;
        if (batDetect(mppt)) return;
        chargerUpdate(mppt.charger, vout, iout, voutAuthority, g_app.psuMode(), mppt.isSweeping());
    }
}

static void loopLF(time_us nowUs) {
    auto &nSamples(sensors.Vout ? sensors.Vout->numSamples : lastNSamples);
    static time_us lastWindowUs = 0;
    auto dt = nowUs - lastWindowUs;
    uint32_t sps = (dt > 20000) ? (uint64_t) (nSamples - lastNSamples) * 1000000llu / dt : 0;
    lfWatchdog(nowUs, dt, sps, nSamples);
    lfStuckWatchdog();
    lfControl();
    mppt.rectCalLf();
    mppt.scanLf(false); // flash writes not simulated; saved and reloaded at the end of the run
    lastTimeOutUs = wallClockUs();
    lastNSamples = nSamples;
    lastWindowUs = nowUs;
}

// ---- harness ----
static const char *modeStr() {
    if (mppt.converter.disabled()) return mppt.inBackoff() ? "OFF/backoff" : "OFF";
    return MpptState2String[(int) mppt.getState()];
}

static void traceLine(const char *pfx = "") {
    auto s = mppt.teleSnap();
    printf("%9.4f %sVin=%5.1f Vout=%6.3f Iout=%6.2f P=%6.1f D=%4u %-11s lim=%2u | plant Vin=%5.1f Vout=%6.3f Iout=%6.2f\n",
           tNow(), pfx, sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(), sensors.Iout->ewm.avg.get(),
           sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get(), converter.getCtrlOnPwmCnt(), modeStr(),
           (unsigned) s.limIdx, g_vconv.getVin(), g_vconv.getVout(), g_vconv.getIoutAvg());
}

static void writeConf(const std::string &dir, const char *file,
                      const std::vector<std::pair<std::string, std::string>> &kv) {
    std::string p = dir + "/" + file;
    FILE *f = ::fopen(p.c_str(), "a");
    for (auto &[k, v]: kv) fprintf(f, "\n%s=%s\n", k.c_str(), v.c_str());
    fclose(f);
}

// The temp littlefs is removed at exit (also on SIGINT/SIGTERM/SIGHUP); SIM_KEEP_FS=1 keeps it.
static void simFsRemove() {
    std::error_code ec;
    if (!g_simFsRoot.empty()) std::filesystem::remove_all(g_simFsRoot, ec);
}

static void simFsSignal(int sig) { exit(128 + sig); }

static void boot(const std::vector<std::pair<std::string, std::string>> &vconvOverrides, bool paranoia0) {
    namespace fs = std::filesystem;
    char tmpl[] = "/tmp/mppt-sim-XXXXXX";
    g_simFsRoot = mkdtemp(tmpl);
    if (!getenv("SIM_KEEP_FS")) {
        atexit(simFsRemove);
        for (int sig: {SIGINT, SIGTERM, SIGHUP}) signal(sig, simFsSignal);
    }
    printf("# littlefs %s%s\n", g_simFsRoot.c_str(), getenv("SIM_KEEP_FS") ? " (kept)" : "");
    fflush(stdout);
    std::string confDir = g_simFsRoot + "/littlefs/conf";
    fs::create_directories(confDir);
    std::string src = std::string(SIM_REPO_ROOT) + "/config/lab/vconv_mock/conf";
    for (auto &e: fs::directory_iterator(src)) fs::copy_file(e.path(), confDir + "/" + e.path().filename().string());
    // vconv_mock ships mode=psu; BMS-cutoff detection is off in PSU mode, so run it as an MPPT charger
    {
        std::string p = confDir + "/converter.conf";
        std::string s;
        FILE *f = ::fopen(p.c_str(), "r");
        char buf[256];
        while (fgets(buf, sizeof buf, f)) {
            if (!strncmp(buf, "mode=", 5) || !strncmp(buf, "psu_vout=", 9)) continue;
            s += buf;
        }
        fclose(f);
        f = ::fopen(p.c_str(), "w");
        fputs(s.c_str(), f);
        fputs("\nmode=mppt\n", f);
        fclose(f);
    }
    if (g_batAuto) {
        std::string p = confDir + "/charger.conf", t;
        FILE *f = ::fopen(p.c_str(), "r");
        char buf[256];
        while (fgets(buf, sizeof buf, f)) if (strncmp(buf, "vout_max", 8)) t += buf;
        fclose(f);
        f = ::fopen(p.c_str(), "w");
        fputs(t.c_str(), f);
        fputs("\nvout_max=auto\n", f);
        fclose(f);
        writeConf(confDir, "board.conf", {{"bflow_bat", "1"}});
    }
    if (!vconvOverrides.empty()) writeConf(confDir, "vconv.conf", vconvOverrides);
    // pin the ADC tick to a board rate (SIM_ADC_HZ, default ~fmetal's 451 Hz) instead of the mock's
    const char *hzEnv = getenv("SIM_ADC_HZ");
    const int hz = hzEnv ? atoi(hzEnv) : 450;
    writeConf(confDir, "vconv.conf", {{"adc_freq", std::to_string(hz)}});
    if (g_simSenseHook)
        writeConf(confDir, "sensor.conf", {{"hv_v_filt_len", "20"}, {"lv_v_filt_len", "20"}, {"lv_i_filt_len", "20"}, {"hv_i_filt_len", "20"}});
    if (g_ring) writeConf(confDir, "vconv.conf", g_ring->conf(g_ringOn)); // ring_c is at the plant L, set after boot
    // SIM_CONF="coil.conf:L0=80e-6,vconv.conf:plant_c_out=470e-6" appends board overrides
    if (const char *c = getenv("SIM_CONF")) {
        std::string all = c;
        for (size_t i = 0, j; i < all.size(); i = j + 1) {
            j = all.find(',', i);
            if (j == std::string::npos) j = all.size();
            std::string kv = all.substr(i, j - i);
            size_t colon = kv.find(':'), eq = kv.find('=');
            if (colon != std::string::npos && eq != std::string::npos && eq > colon)
                writeConf(confDir, kv.substr(0, colon).c_str(), {{kv.substr(colon + 1, eq - colon - 1), kv.substr(eq + 1)}});
        }
    }
    if (paranoia0) writeConf(confDir, "limits.conf", {{"reverse_current_paranoia", "0"}});
    if (const char *d = getenv("SIM_DRV")) {
        if (strcmp(d, "mcpwm") && strcmp(d, "ledc")) { printf("SIM_DRV=%s: want mcpwm or ledc\n", d); exit(1); }
        g_drvMcpwm = !strcmp(d, "mcpwm");
    }
    if (g_drvMcpwm) writeConf(confDir, "vconv.conf", {{"pwm_clk", "160000000"}});
    if (g_ring) {
        char l0[24];
        snprintf(l0, sizeof l0, "%g", g_ring->l0Conf);
        writeConf(confDir, "coil.conf", {{"L0", l0}});
        if (g_drvMcpwm && g_liveOffset && g_ring->rectOffsetNs != 0) {
            snprintf(l0, sizeof l0, "%g", g_ring->rectOffsetNs);
            writeConf(confDir, "coil.conf", {{"rect_offset_ns", l0}, {"rect_offset_drv", "vconv"}, {"rect_offset_cal", "1"}});
        }
    }

    // setup(): the parts of setupConverterAndMppt() that the RT path needs
    ConfFile boardConf{"/littlefs/conf/board.conf"};
    ConfFile limitsConf{"/littlefs/conf/limits.conf"};
    Limits lim{limitsConf};
    setupSensors(boardConf, lim);
    if (const char *e = getenv("SIM_NOTCH_DB")) adcSampler.notchAmp = NotchFilter::gainAmp((float) atof(e));
    mppt.initSensors(boardConf);
    ConfFile coilConf{"/littlefs/conf/coil.conf"};
    ConfFile chargerConf{"/littlefs/conf/charger.conf"};
    ConfFile converterConf{"/littlefs/conf/converter.conf"};
    mppt.charger.begin(chargerConf);
    converter.init(converterConf, boardConf, coilConf);
    mppt.rectCal.autoRun = getenv("SIM_RCAL_AUTO") && coilConf.getByte("rect_cal_auto", 1);
    mppt.rectCal.persist = rectCalPersist;
    mppt.rectCal.init(converter.getRectOnOffset(), !converter.rectOffsetNeedsCal());
    ConfFile trackerConf{"/littlefs/conf/tracker.conf", true};
    mppt.begin(trackerConf, boardConf, converterConf, lim, TeleConf{});
    if (g_app.setupErr) { printf("setupErr\n"); exit(2); }
    adcSampler.begin(); // loopRT prologue
    setLoopRateMin();
    printf("# sim tick %u us, pwm_freq %lu, Vbat_max %.2f, Iout_max %.1f, p_max %.0f\n", g_simTickUs,
           (unsigned long) converter.getPwmFrequency(), mppt.charger.params.Vbat_max, lim.Iout_max, lim.P_max);
    printf("# gate timing %s: pwmMax %u, minLS %u, maxHS %u, count %.3f ns, rect_offset %d ct\n", g_drvMcpwm ? "mcpwm" : "ledc",
           converter.pwmMaxDriver(), converter.getRectOnPwmMin(), converter.pwmCtrlMax, 1e9f / converter.getPwmTickRate(),
           converter.getRectOnOffset());
}

struct Probe {
    double t0;
    float voutPeakSensor = -1, voutPeakPlant = -1;
    float ioutBefore, voutBefore, vinBefore, pBefore;
    size_t tripIdx0;
    double firstOffT = -1;
    // bmsCutoffSignature() terms, evaluated while the converter runs (bmsCutoff() is gated on that)
    uint32_t nOn = 0, nC[4] = {}, nThree = 0, nAll = 0;
    float minIRatio = INFINITY, maxDvRatio = -INFINITY;
};

// call after a tick that started with the converter on: sensors hold what protect() saw
static void probeDetector(Probe &p) {
    const float ioutMin = std::max(1.f, mppt.limits.Iout_max * 0.05f);
    const float dvMin = mppt.charger.params.Vbat_max * 0.02f;
    const float i = sensors.Iout->med3.get(), ia = sensors.Iout->ewm.avg.get();
    const float v = sensors.Vout->med3.get(), va = sensors.Vout->ewm.avg.get();
    const float vi = sensors.Vin->med3.get(), via = sensors.Vin->ewm.avg.get();
    bool c[4] = {ia > ioutMin, i < ia * 0.25f, v > va + dvMin, vi > via};
    int n = 0;
    for (int k = 0; k < 4; ++k) if (c[k]) { ++p.nC[k]; ++n; }
    ++p.nOn;
    if (n == 3) ++p.nThree;
    if (n == 4) ++p.nAll;
    if (c[0]) p.minIRatio = std::min(p.minIRatio, i / ia);
    p.maxDvRatio = std::max(p.maxDvRatio, (v - va) / dvMin);
}

static Probe probeStart() {
    Probe p{};
    p.t0 = tNow();
    g_vconv.takeVoutMax();
    p.ioutBefore = sensors.Iout->ewm.avg.get();
    p.voutBefore = sensors.Vout->ewm.avg.get();
    p.vinBefore = sensors.Vin->ewm.avg.get();
    p.pBefore = p.ioutBefore * p.voutBefore;
    p.tripIdx0 = g_trips.size();
    g_eventT = p.t0;
    g_logSince.clear();
    return p;
}

// run for `sec`, tracing every `traceEvery` s; per-tick trace for the first `fineMs` after start
static void run(double sec, double traceEvery, Probe *probe = nullptr, double fineMs = 0) {
    double tEnd = tNow() + sec, nextTrace = tNow();
    bool wasOn = !converter.disabled();
    while (tNow() < tEnd) {
        if (g_rampSec > 0) {
            double f = std::min(1.0, (tNow() - g_rampT0) / g_rampSec);
            g_vconv.setPv((float) (g_rampFrom + (g_rampTo - g_rampFrom) * f), g_vconv.getVoc(), g_vconv.getPvK());
            if (f >= 1.0) g_rampSec = 0;
        }
        bool onBefore = !converter.disabled();
        const double tIt = tNow();
        loopRTIter();
        g_qOut += g_vconv.getIoutAvg() * (tNow() - tIt);
        if (g_tickHook) g_tickHook();
        if (mppt.sensorPhysicalI) { // loopNetwork_task
            const uint64_t ms = esp_timer_get_time() / 1000;
            selftestUrgent(ms), bfTestTick((uint32_t) ms);
        }
        if (g_pDump) {
            const float r[6] = {(float) tNow(), sensors.Iout->ewm.avg.get(), sensors.Vout->ewm.avg.get(),
                                (float) converter.getCtrlOnPwmCnt(), (float) mppt.tracker.slowMode,
                                g_vconv.getVin() * g_vconv.pvCurrent(g_vconv.getVin())};
            fwrite(r, 4, 6, g_pDump);
        }
        if (probe && onBefore) probeDetector(*probe);
        mppt.tempPollStale(wallClockUs());
        if ((wallClockUs() - lastLfWorkUs) >= lfPeriod or !lastLfWorkUs) {
            loopLF(wallClockUs());
            lastLfWorkUs = wallClockUs();
        }
        bool on = !converter.disabled();
        if (g_trkT1 > 0 && tNow() >= g_trkT0 && tNow() <= g_trkT1) {
            static time_ms lastT = 0;
            const auto &tr = mppt.tracker;
            if (tr._time != lastT) {
                lastT = tr._time;
                const float v = g_vconv.getVin();
                printf("%9.3f trk: D=%u Vin=%.2f/%.2f Ppv=%.2f Pm=%.3f dP=%+.3f ref=%.3f dir=%c%s lim=%u mode=%s dcm=%d M=%.3f sr=%d LS=%u/%u bf=%d Io=%.3f/%.3f\n",
                       tNow(), converter.getCtrlOnPwmCnt(), v, sensors.Vin->ewm.avg.get(), v * g_vconv.pvCurrent(v),
                       tr._curPower, tr.dP, tr._lastPower, tr._direction ? '+' : '-', tr.slowMode ? " slow" : "",
                       (unsigned) mppt.teleSnap().limIdx, modeStr(), (int) converter.inDCM(), g_vconv.getVout() / v,
                       (int) converter.syncRectEnabled_(), converter.getRectOnPwmCnt(), converter.getRectOnPwmMax(),
                       (int) mppt.bflow.state(), g_vconv.getIoutAvg(), sensors.Iout->ewm.avg.get());
            }
        }
        if (g_fineT1 > 0 && tNow() >= g_fineT0 && tNow() <= g_fineT1) {
            printf("%9.4f fine: Vin last=%.2f med=%.2f ewm=%.2f | Vout last=%.3f med=%.3f ewm=%.3f n=%lu | Iout last=%.2f med=%.2f ewm=%.2f | D=%u %s cal=%d\n",
                   tNow(), sensors.Vin->last, sensors.Vin->med3.get(), sensors.Vin->ewm.avg.get(), sensors.Vout->last, sensors.Vout->med3.get(), sensors.Vout->ewm.avg.get(),
                   (unsigned long) sensors.Vout->numSamples, sensors.Iout->last, sensors.Iout->med3.get(),
                   sensors.Iout->ewm.avg.get(), converter.getCtrlOnPwmCnt(), modeStr(), (int) adcSampler.isCalibrating());
        }
        if (probe) {
            probe->voutPeakSensor = std::max(probe->voutPeakSensor, sensors.Vout->last);
            float vMax = g_vconv.takeVoutMax();
            if (tNow() - probe->t0 < 1.0) probe->voutPeakPlant = std::max(probe->voutPeakPlant, vMax);
            if (wasOn && !on && probe->firstOffT < 0) probe->firstOffT = tNow();
            if ((tNow() - probe->t0) * 1e3 < fineMs) traceLine("  ");
        }
        g_nStarts += on && !wasOn;
        if (on && !wasOn) g_onT.push_back(tNow());
        if (on != wasOn && g_verbose) printf("%9.4f --- converter %s\n", tNow(), on ? "ON" : "OFF");
        wasOn = on;
        if (traceEvery > 0 && tNow() >= nextTrace) {
            traceLine();
            nextTrace += traceEvery;
        }
    }
}

static void report(const char *name, const Probe &p) {
    printf("\n== %s: event at t=%.4f s, before: Vin=%.2f Vout=%.3f Iout=%.2f P=%.1f\n", name, p.t0, p.vinBefore,
           p.voutBefore, p.ioutBefore, p.pBefore);
    if (g_trips.size() > p.tripIdx0) {
        auto &t = g_trips[p.tripIdx0];
        printf("== first protection: [%s] backoff %us after %.1f ms\n", t.who.c_str(), t.sec, (t.t - p.t0) * 1e3);
        for (size_t i = p.tripIdx0; i < g_trips.size() && i < p.tripIdx0 + 12; ++i)
            printf("==   trip %zu: t=%.4f (+%.1f ms) [%s] %us\n", i - p.tripIdx0, g_trips[i].t,
                   (g_trips[i].t - p.t0) * 1e3, g_trips[i].who.c_str(), g_trips[i].sec);
        if (g_trips.size() - p.tripIdx0 > 12) printf("==   ... %zu trips total\n", g_trips.size() - p.tripIdx0);
    } else {
        printf("== no protection trip\n");
    }
    printf("== detector terms over %u on-ticks: ioutAvg>min %u, iout<avg/4 %u, dVout>2%%Vbat_max %u, Vin>avg %u;"
           " 3-of-4 %u, all %u; min iout/avg %.3f, max dVout/dvMin %.2f\n",
           p.nOn, p.nC[0], p.nC[1], p.nC[2], p.nC[3], p.nThree, p.nAll, p.minIRatio, p.maxDvRatio);
    if (p.firstOffT >= 0) printf("== converter first off at +%.1f ms\n", (p.firstOffT - p.t0) * 1e3);
    printf("== peak Vout: sensor(last) %.3f V, plant (per PWM cycle, 1 s) %.3f V\n", p.voutPeakSensor, p.voutPeakPlant);
}

static void settle(double sec) {
    run(sec, 2.0);
    traceLine("settled ");
}

// ---- weather scenarios (wx-*) ----
using Track = std::vector<std::pair<float, float>>; // (t [s], value): piecewise linear, held outside

static float trackAt(const Track &tr, double t, float dflt) {
    if (tr.empty()) return dflt;
    if (t <= tr.front().first) return tr.front().second;
    if (t >= tr.back().first) return tr.back().second;
    size_t i = 1;
    while (tr[i].first < t) ++i;
    return tr[i - 1].second + (tr[i].second - tr[i - 1].second) * ((float) t - tr[i - 1].first) / (tr[i].first - tr[i - 1].first);
}

// flat, 2026-11-14 11:20:00..11:35:00 UTC: P / 776 W (1 s means, reduced to 2 % max error)
static const Track kCloud1120 = {
    {0, 0.948f}, {10, 0.943f}, {12, 0.890f}, {14, 0.922f}, {16, 0.630f}, {17, 0.540f}, {18, 0.521f}, {20, 0.387f},
    {25, 0.285f}, {36, 0.249f}, {42, 0.248f}, {48, 0.294f}, {49, 0.328f}, {50, 0.452f}, {52, 0.571f}, {54, 0.784f},
    {56, 0.703f}, {57, 0.712f}, {59, 0.953f}, {61, 0.946f}, {63, 0.834f}, {64, 0.894f}, {65, 0.892f}, {66, 0.815f},
    {67, 0.794f}, {68, 0.889f}, {69, 0.768f}, {70, 0.563f}, {73, 0.522f}, {75, 0.441f}, {77, 0.505f}, {79, 0.504f},
    {81, 0.435f}, {85, 0.397f}, {89, 0.438f}, {91, 0.539f}, {93, 0.762f}, {94, 0.807f}, {95, 0.911f}, {96, 0.940f},
    {101, 0.892f}, {110, 0.901f}, {113, 0.870f}, {114, 0.902f}, {119, 0.883f}, {187, 0.924f}, {189, 0.917f}, {191,
    0.807f}, {192, 0.799f}, {194, 0.880f}, {196, 0.814f}, {197, 0.863f}, {198, 0.826f}, {205, 0.932f}, {211,
    0.955f}, {212, 0.940f}, {213, 0.876f}, {215, 0.892f}, {217, 0.652f}, {218, 0.615f}, {222, 0.369f}, {223,
    0.337f}, {225, 0.392f}, {226, 0.346f}, {228, 0.461f}, {229, 0.354f}, {230, 0.328f}, {232, 0.365f}, {233,
    0.423f}, {234, 0.513f}, {235, 0.659f}, {236, 0.669f}, {237, 0.557f}, {240, 0.521f}, {241, 0.549f}, {243,
    0.879f}, {244, 0.974f}, {251, 0.947f}, {253, 0.900f}, {254, 0.932f}, {258, 0.940f}, {293, 0.927f}, {296,
    0.909f}, {298, 0.830f}, {299, 0.830f}, {301, 0.890f}, {303, 0.904f}, {304, 0.877f}, {306, 0.920f}, {314,
    0.892f}, {315, 0.916f}, {319, 0.909f}, {323, 0.869f}, {324, 0.907f}, {329, 0.913f}, {332, 0.877f}, {336,
    0.937f}, {342, 0.920f}, {347, 0.942f}, {349, 0.888f}, {351, 0.926f}, {353, 0.817f}, {355, 0.883f}, {356,
    0.850f}, {357, 0.740f}, {358, 0.672f}, {360, 0.688f}, {362, 0.574f}, {363, 0.558f}, {364, 0.669f}, {367,
    0.569f}, {369, 0.577f}, {370, 0.548f}, {372, 0.419f}, {375, 0.344f}, {384, 0.302f}, {389, 0.340f}, {390,
    0.380f}, {391, 0.465f}, {393, 0.548f}, {396, 0.524f}, {397, 0.607f}, {398, 0.801f}, {399, 0.937f}, {401,
    0.982f}, {406, 0.979f}, {408, 0.948f}, {415, 0.969f}, {418, 0.920f}, {420, 0.971f}, {429, 0.947f}, {431,
    0.983f}, {438, 0.991f}, {441, 0.978f}, {442, 0.939f}, {443, 0.978f}, {447, 0.989f}, {450, 0.956f}, {451,
    0.983f}, {457, 0.983f}, {458, 0.944f}, {463, 0.987f}, {468, 0.942f}, {471, 0.999f}, {472, 0.968f}, {474,
    1.000f}, {478, 0.985f}, {480, 0.932f}, {482, 0.991f}, {483, 0.956f}, {486, 0.684f}, {488, 0.570f}, {490,
    0.572f}, {491, 0.640f}, {492, 0.628f}, {495, 0.446f}, {496, 0.463f}, {501, 0.382f}, {508, 0.399f}, {512,
    0.502f}, {513, 0.497f}, {514, 0.553f}, {516, 0.828f}, {518, 0.963f}, {519, 0.963f}, {521, 0.890f}, {522,
    0.964f}, {523, 0.972f}, {525, 0.864f}, {526, 0.662f}, {527, 0.531f}, {528, 0.550f}, {530, 0.688f}, {532,
    0.661f}, {533, 0.737f}, {534, 0.904f}, {535, 0.962f}, {536, 0.956f}, {537, 0.926f}, {538, 0.809f}, {539,
    0.782f}, {540, 0.839f}, {542, 0.810f}, {544, 0.626f}, {545, 0.606f}, {547, 0.830f}, {548, 0.842f}, {549,
    0.662f}, {550, 0.432f}, {551, 0.379f}, {563, 0.311f}, {569, 0.325f}, {570, 0.350f}, {572, 0.480f}, {573,
    0.647f}, {574, 0.738f}, {576, 0.645f}, {578, 0.617f}, {586, 0.722f}, {587, 0.760f}, {589, 0.734f}, {590,
    0.812f}, {591, 0.799f}, {594, 0.533f}, {597, 0.378f}, {602, 0.272f}, {606, 0.239f}, {620, 0.247f}, {630,
    0.302f}, {635, 0.368f}, {638, 0.455f}, {642, 0.469f}, {646, 0.284f}, {656, 0.229f}, {808, 0.111f}, {899, 0.116f}
};

struct Wx {
    float voc = 70, k = 0.78f, pClear = 776; // clear-sky string: Voc, Vmpp/Voc, MPP power
    uint8_t n = 1;                           // series substrings (bypass-diode sections), up to 6
    float w[6] = {1, 0, 0, 0, 0, 0};         // their Voc shares (all 0 = equal)
    float vd = 0;                            // bypass-diode drop [V]
    Track cloud, sub[6];                     // irradiance factor of the whole string / per substring
    bool beamShade = false;                  // shade blocks the beam only: substring gets min(cloud, sub)
    float vocLog = 0;                        // Voc(t) = voc + vocLog * ln(irradiance)
    float vbat = 26.6f, rbat = 0.036f;
    Track vbatT;                             // battery EMF profile
    double sweepAt = -1, statsFrom = 0, T = 600;
    std::vector<double> sweepsAt; // further requested sweeps, ascending
    std::string verdict; // sweep verdict events expected (g_verdict), "" = not checked; with sweeps after statsFrom
    size_t sweeps = 0, maxCals = 0; // maxCals: CALFAIL check, 0 = off
    float isc = 0, g[6] = {1, 1, 1, 1, 1, 1}, vocNow = 0;
    float half = 0, vocDrift = 0;           // half-substring shade, Voc drift over T (fraction)
    double driftT = 0, driftT0 = 0;          // Voc drift from driftT0 to driftT (0: T)
    bool exact = false;                      // exact series-string plant
    float gm[6] = {1, 1, 1, 1, 1, 1};        // per-substring Isc mismatch
    float ramp = 0, wobA = 0, wobF = 0;      // irradiance x exp(ramp (t - 120 s)), x (1 + wobA sin(2 pi wobF t))
    Track vocT;                              // source Voc profile (overrides vocLog)
    float iscFix = 0;                        // Isc [A] instead of the one from pClear
    bool noSink = false;                     // source does not sink current (bench PSU); default: sinks above Voc
    float manual = 0;                        // > 0: manual duty (fraction of pwmMax) instead of MPPT
    double bmsAt = -1;                       // BMS frames from then on (ibat every 4 s, highest cell every 12 s)
    float vcell = 0;                         // highest cell [V] the BMS reports
    int bmsResweep = -1;                     // BMSCAP check: 1 a re-sweep must find the MPP, 0 no further sweep
    bool ovl = false;                        // OVL check (wx-ovl-*)
    float ioutMax = 0;                       // limits.conf lv_i_max, 0 = the mock's
    Track chipT;                             // chip temperature [C] (thermal power derate above temp_derate)

    void init() {
        PvModel m;
        m.set(1, voc, k);
        float best = 0;
        for (float v = 1; v < voc; v += 0.05f) best = std::max(best, v * m.current(v));
        isc = iscFix > 0 ? iscFix : pClear / best;
        vocNow = voc;
        g_vconv.setPv(isc, voc, k);
        g_vconv.setPvBlocking(noSink);
        g_vconv.setBat(vbat, rbat);
        if (manual > 0)
            g_app.opMode = OpMode::Manual,
            mppt.manualTarget = (uint16_t) std::lround(manual * (float) converter.pwmMaxDriver());
    }

    void apply(double t) {
        const float c = trackAt(cloud, t, 1) * std::exp(ramp * (float) std::max(0.0, t - 120)) *
                        (1 + wobA * (float) std::sin(6.283185 * wobF * t));
        float gMax = 0;
        for (uint8_t i = 0; i < n; ++i) {
            const float s = trackAt(sub[i], t, 1);
            if (half > 0) g_vconv.setPvHalf(i, s < 1 ? 1.f / s : 1.f); // half the cells shaded, the other half in full light
            g[i] = (beamShade ? std::min(c, s) : c * s) * gm[i];
            gMax = std::max(gMax, g[i]);
        }
        g_vconv.setPvString(n, g, n > 1 && w[1] > 0 ? w : nullptr, vd);
        if (vocDrift != 0) {
            const double t1 = driftT > 0 ? driftT : T;
            const float v = voc * (1 + vocDrift * (float) std::clamp((t - driftT0) / (t1 - driftT0), 0.0, 1.0));
            if (std::fabs(v - vocNow) > 1e-3f) g_vconv.setPv(isc, vocNow = v, k);
        }
        if (vocLog > 0 || !vocT.empty()) {
            const float v = !vocT.empty() ? trackAt(vocT, t, voc)
                                          : std::max(1.f, voc + vocLog * std::log(std::max(gMax, 1e-30f)));
            if (std::fabs(v - vocNow) > 1e-3f) g_vconv.setPv(isc, vocNow = v, k);
        }
        if (!vbatT.empty()) g_vconv.setBat(trackAt(vbatT, t, vbat), rbat);
        if (!chipT.empty()) g_simChipTemp = trackAt(chipT, t, 40);
    }
};

struct Peak {
    float v, p;
};

// local maxima of the plant's P(V) in ascending V; valley[i] separates peak i and i+1
static int pvPeaks(Peak pk[6], float valley[6]) {
    int n = 0;
    float p0 = 0;
    bool rising = true;
    for (float v = 1; v < g_vconv.getVoc() + 0.2f && n < 6; v += 0.1f) {
        const float p = v * g_vconv.pvCurrent(v);
        if (rising && p < p0) pk[n++] = {v - 0.1f, p0}, rising = false;
        else if (!rising && p > p0) valley[n - 1] = v - 0.1f, rising = true;
        p0 = p;
    }
    return n;
}

// ---- switch-node ring analysis (ring-* scenarios) ----
struct SweepPt {
    uint16_t h;
    float vin, vout, iout, ioutSensor;
};
static std::vector<SweepPt> g_sweepPts;
static uint16_t g_sweepEndH = 0, g_sweepEndH6 = 0;
static float g_sweepEndV = 0, g_sweepEndV6 = 0;
static double g_sweepEndT = -1, g_ringRecFrom = 0; // first sweep starting after g_ringRecFrom

static void ringTick() {
    static bool was = false;
    static uint16_t last = 0;
    const bool sw = mppt.getState() == MpptControlMode::Sweep && !converter.disabled();
    const uint16_t h = converter.getCtrlOnPwmCnt();
    if (sw && h > last && g_sweepEndT < 0 && tNow() >= g_ringRecFrom)
        g_sweepPts.push_back({h, g_vconv.getVin(), g_vconv.getVout(), g_vconv.getIoutAvg(), sensors.Iout->ewm.avg.get()});
    if (was && !sw && g_sweepEndT < 0 && !g_sweepPts.empty() && tNow() >= g_ringRecFrom) g_sweepEndT = tNow();
    if (g_sweepEndT >= 0 && !g_sweepEndH && tNow() >= g_sweepEndT + 2.0) g_sweepEndH = h, g_sweepEndV = g_vconv.getVin();
    if (g_sweepEndT >= 0 && !g_sweepEndH6 && tNow() >= g_sweepEndT + 6.0) g_sweepEndH6 = h, g_sweepEndV6 = g_vconv.getVin();
    if (!sw) last = 0;
    else last = std::max(last, h);
    was = sw;
}

// steady PV power versus HS count at the present PV/battery: a copy of the plant stepped to equilibrium at
// each count, LS as the firmware sets it in DCM (voltage ratio with its 1 % worst-case margins)
static std::vector<std::pair<uint16_t, float>> staticCurve(uint16_t hi, uint16_t step) {
    VirtualConverter v = g_vconv;
    const uint16_t pmax = v.getPwm().pwmMax, lsMin = converter.getRectOnPwmMin();
    std::vector<std::pair<uint16_t, float>> out;
    for (uint16_t h = step; h <= hi; h += step) {
        for (int k = 0; k < 3; ++k) {
            const float r = v.getVin() / v.getVout() * (0.99f / 1.01f) - 1.f;
            long ls = std::max<long>(lsMin, std::lround(h * std::max(0.f, r)));
            v.setPwmCount(0, h), v.setPwmCount(1, (uint32_t) std::min<long>(ls, pmax - h - 1));
            v.stepSeconds(0.08f, 39000);
        }
        out.emplace_back(h, v.getVin() * v.pvCurrent(v.getVin()));
    }
    return out;
}

// local maxima of P(count) within +-w points above 50 % of the maximum
static std::vector<std::pair<uint16_t, float>> localMaxima(const std::vector<std::pair<uint16_t, float>> &c, int w) {
    std::vector<std::pair<uint16_t, float>> m;
    float pMax = 0;
    for (auto &p: c) pMax = std::max(pMax, p.second);
    for (int i = 0; i < (int) c.size(); ++i) {
        bool top = c[i].second > 0.5f * pMax;
        for (int j = std::max(0, i - w); top && j <= std::min((int) c.size() - 1, i + w); ++j)
            if (c[j].second > c[i].second) top = false;
        if (top) m.push_back(c[i]);
    }
    return m;
}

static float curveAt(const std::vector<std::pair<uint16_t, float>> &c, uint16_t h) {
    float best = 0;
    int d = 1 << 30;
    for (auto &p: c) if (std::abs((int) p.first - (int) h) < d) d = std::abs((int) p.first - (int) h), best = p.second;
    return best;
}

static void ringSweepReport() {
    const uint16_t pmax = g_vconv.getPwm().pwmMax;
    const float fsw = (float) converter.getPwmFrequency(), tick = 1.f / (fsw * (float) pmax), L = g_vconv.getL();
    float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
    std::vector<float> rLo, rHi, sLo, sHi;
    int n = 0;
    const size_t stride = std::max<size_t>(1, g_sweepPts.size() / 200);
    std::vector<SweepPt> dcm;
    for (auto &p: g_sweepPts) {
        const float D = (float) p.h / (float) pmax, cf = D * p.vin / p.vout;
        if (cf > 0.85f || cf < 0.1f || p.iout < 0.05f) continue;
        dcm.push_back(p);
        const float ideal = (p.vin - p.vout) * p.vin * D * D / (2 * p.vout * fsw * L);
        (cf < 0.5f ? rLo : rHi).push_back(p.iout / ideal - 1);
        (cf < 0.5f ? sLo : sHi).push_back(p.ioutSensor / ideal - 1);
    }
    // thin to <= kMaxPts evenly for the ring fit
    for (size_t k = 0; k < dcm.size() && n < coilfit::kMaxPts; k += std::max<size_t>(1, dcm.size() / coilfit::kMaxPts)) {
        auto &p = dcm[k];
        d[n] = (float) p.h / (float) pmax, vi[n] = p.vin, vo[n] = p.vout, io[n] = p.iout, ++n;
    }
    (void) stride;
    auto pp = [](std::vector<float> v) {
        if (v.size() < 5) return 0.f;
        std::sort(v.begin(), v.end());
        return v[(size_t) (0.95 * (double) (v.size() - 1))] - v[(size_t) (0.05 * (double) (v.size() - 1))];
    };
    const auto f = coilfit::fit(d, vi, vo, io, n, fsw);
    float ratio = 0;
    for (int i = 0; i < n; ++i) ratio += vi[i] / vo[i] / (float) n;
    const float N = f.f > 0 ? 1.f / (f.f * tick * ratio) : 0;
    printf("== RING sweep: %zu rising-duty ticks, %zu in DCM; plant Iout vs ideal DCM, p-p (5-95 %%): conduction <0.5"
           " %.0f %%, 0.5..0.85 %.0f %% (sensor ewm %.0f / %.0f %%)\n", g_sweepPts.size(), dcm.size(), 100 * pp(rLo),
           100 * pp(rHi), 100 * pp(sLo), 100 * pp(sHi));
    printf("== RING sweep fit: ring %.0f kHz (plant %.0f), period %.0f counts here = %.0f at 6.25 ns/count, Vin/Vout %.2f,"
           " ripple +-%.0f %%, %.1f periods; L from the sweep %.1f uH (plant %.1f)\n", f.f * 1e-3f,
           g_vconv.getRingF() * 1e-3f, N, N * tick / 6.25e-9f, ratio, 100 * f.amp, f.periods, f.L * 1e6f, L * 1e6f);
}

static void ringStaticReport(const char *what, const std::vector<double> &dutyHist) {
    const uint16_t hi = converter.pwmCtrlMax;
    const auto c = staticCurve(hi, 2);
    float pMax = 0;
    uint16_t hMax = 0;
    for (auto &p: c) if (p.second > pMax) pMax = p.second, hMax = p.first;
    const auto lm = localMaxima(c, 3);
    printf("== RING static P(count) now: max %.1f W at %u; %zu local maxima above 50 %%:", pMax, hMax, lm.size());
    for (size_t i = 0; i < lm.size() && i < 12; ++i) printf(" %u:%.1f%%", lm[i].first, 100 * lm[i].second / pMax);
    printf("\n");
    if (g_sweepEndT >= 0)
        printf("== RING sweep chose count %u (t=%.1f + 2 s): static P there %.1f %% of the max (max at %u); Vin %.1f V,"
               " +6 s count %u %.1f %% Vin %.1f V\n", g_sweepEndH, g_sweepEndT, 100 * curveAt(c, g_sweepEndH) / pMax, hMax,
               g_sweepEndV, g_sweepEndH6, 100 * curveAt(c, g_sweepEndH6) / pMax, g_sweepEndV6);
    if (!dutyHist.empty()) {
        double tot = 0;
        for (double x: dutyHist) tot += x;
        std::vector<std::pair<double, int>> top;
        for (int b = 0; b < (int) dutyHist.size(); ++b) if (dutyHist[b] > 0) top.emplace_back(dutyHist[b], b);
        std::sort(top.rbegin(), top.rend());
        printf("== RING %s: time at count (4-count bins):", what);
        for (size_t i = 0; i < top.size() && i < 6; ++i)
            printf(" %d: %.0f%% (static P %.1f%%)", top[i].second * 4 + 2, 100 * top[i].first / tot,
                   100 * curveAt(c, (uint16_t) (top[i].second * 4 + 2)) / pMax);
        printf("\n");
    }
}

#if SCAN_ACTIVE_DEFINED
// integration checks around a running partial peak scan (SIM_ACT=psu|manual|limit on a scan scenario)
struct ItState {
    int act = 0, step = 0;
    double t1 = 0;
    float vMax0 = 0;
    bool done = false, fail = false;
    const char *why = "";
};

static void itFail(ItState &it, const char *why) {
    if (!it.fail) it.fail = true, it.why = why;
}

static void itStep(ItState &it, double t) {
    auto &sc = mppt.scan;
    if (it.act == 1 && it.step >= 1 && g_app.psuMode() && mppt.targetDutyCycle) itFail(it, "PSU mode with a duty fade target");
    switch (it.step) {
        case 0: // wait for a probe
            if (sc.phase != PeakScan::Probe) return;
            it.t1 = t, it.step = 1;
            printf("%9.2f >>> IT action during the probe\n", t);
            if (it.act == 1) {
                if (!mppt.requestPsuSetpoint(g_vconv.getVbat() + 0.3f)) itFail(it, "psu request refused");
            } else if (it.act == 2) {
                (void) mppt.requestPsuManual(converter.getCtrlOnPwmCnt(), -1);
            } else {
                it.vMax0 = mppt.charger.params.Vbat_max;
                mppt.charger.params.Vbat_max = sensors.Vout->ewm.avg.get() - 0.2f;
            }
            return;
        case 1:
            if (t - it.t1 < 0.5) return;
            if (sc.active()) itFail(it, "scan still active 0.5 s after the action");
            if (it.act == 1 && mppt.targetDutyCycle) itFail(it, "PSU mode with a duty fade target");
            if (it.act == 2 && !g_app.manualPwm()) itFail(it, "not in manual mode");
            if (it.act == 3 && mppt.teleSnap().limIdx == 15) itFail(it, "no limiter after the cap drop");
            it.step = 2, it.t1 = t;
            return;
        case 2:
            if (it.act == 1 && (sc.active() || mppt.targetDutyCycle)) itFail(it, "PSU: scan or fade target came back");
            if (t - it.t1 < 5) return;
            if (it.act == 2) (void) mppt.requestPsuOff(); // manual -> MPPT, no sweep
            if (it.act == 3) mppt.charger.params.Vbat_max = it.vMax0;
            it.step = 3, it.t1 = t;
            return;
        case 3:
            // back in MPPT: no scan from the old reference before a new one is taken (5 s) and the period passed
            if (it.act == 2 && sc.active() && t - it.t1 < 6) itFail(it, "stale scan resumed after manual -> MPPT");
            if (t - it.t1 > 10) it.done = true, it.step = 4;
            return;
        default:
            return;
    }
}
#endif

// wx-ovl-*: per-tick maxima of Iout ewm / med3 over limits.Iout_max (the Iout-OC terms) from statsFrom
static float g_ovlEwm = 0, g_ovlMed = 0;
static double g_ovlFrom = 0;
static void ovlTick() {
    if (g_ring) ringTick();
    if (tNow() < g_ovlFrom) return;
    g_ovlEwm = std::max(g_ovlEwm, sensors.Iout->ewm.avg.get() / mppt.limits.Iout_max);
    g_ovlMed = std::max(g_ovlMed, sensors.Iout->med3.get() / mppt.limits.Iout_max);
}

static int runWx(const std::string &name, Wx &wx) {
    if (const char *e = getenv("SIM_T")) wx.T = atof(e);
    g_verbose = getenv("SIM_V") != nullptr;
    const double trEvery = getenv("SIM_TR") ? atof(getenv("SIM_TR")) : wx.T / 60;
    // exact series string by default on the bypass-diode scenarios; SIM_PV_EXACT=0/1 overrides
    g_vconv.setPvExact(getenv("SIM_PV_EXACT") ? atoi(getenv("SIM_PV_EXACT")) != 0 : wx.exact);
    wx.init();
    wx.apply(0);
    // a BMS cell topic is configured, as on a live board: the start path waits for its data
    if (wx.bmsResweep >= 0) mppt.charger.beginMqtt(ConfFile{{"cell_voltages_max_topic", "bms"}});
    std::vector<double> dutyHist;
    if (g_ring) g_tickHook = ringTick, dutyHist.assign(1024, 0.0), g_ringRecFrom = std::max(0.0, wx.sweepAt);
    printf("# %s: clear-sky MPP %.0f W, Isc %.2f A, Voc %.0f V, %u substring(s), bat %.2f V / %.3f ohm, %.0f s\n",
           name.c_str(), wx.pClear, wx.isc, wx.voc, (unsigned) wx.n, wx.vbat, wx.rbat, wx.T);
    printf("#        t  g(substrings)     | PLANT Vin   Ppv | GMPP V     P  peaks | CTRL state D lim\n");

    enum { OFF, SWEEP, LIMIT, WRONG, TRACK, SCAN, NCAT };
    double eG = 0, eUse = 0, ePv = 0, eOut = 0, eCtl = 0, tCat[NCAT] = {}, loss[NCAT] = {}, tStat = 0;
    struct Span { // vOp..pG: at the largest gap
        double t0, t1, lost;
        float vOp, pOp, vG, pG;
    };
    std::vector<Span> wrong;
    bool inWrong = false;
    float pGLast = 0;
    Peak pk[6];
    float valley[6], key[7] = {-1, -1, -1, -1, -1, -1, -1};
    int nPk = 0, maxPk = 0;
    // scan probes against the plant at the scan start (maxima and probe powers from one plant state); the outcome from
    // where the operating point landed 5 s after the scan ended, on the plant then (PROBE lines)
    struct ProbeRec {
        Peak pk[6];
        float valley[6], vc[6], rel[6], v0, bestV;
        int n, nc, iG, iOp;
        char end;
        double tIdle;
    } pr{};
    bool prAct = false;
    int prN[7] = {}; // ok, hit, miss (placement), miss (noise), wrong, partial, light
    double prErrG = 0, prErrGMax = 0, prRelG = 0;
    int prNG = 0;
    auto basin = [](const ProbeRec &r, float v) {
        int b = 0;
        while (b + 1 < r.n && v > r.valley[b]) ++b;
        return b;
    };
    // tracker steady state: duty and Vin while tracking, slow-mode time, reversals, slow-mode entries
    double dN = 0, dS = 0, dS2 = 0, vS = 0, vS2 = 0, tSlow = 0, lossSlow = 0;
    float slowDvMax = 0; // largest |Vin - Vmpp| / Vmpp in slow mode
    float dMin = 1e9f, dMax = 0, vMin = 1e9f, vMax = 0;
    uint32_t nRev = 0, nSlowIn = 0;
    time_ms lastRev = 0;
    bool wasSlow = false;
    double holdAt = getenv("SIM_HOLD") ? atof(getenv("SIM_HOLD")) : -1; // tracker output x0 from then on
    // reverse output current per 20 ms slice (plant truth); time on above the LS min pulse's carry-over ratio
    double iMin = 0, tNeg = 0, qNeg = 0, tAbove = 0, tAboveDcm = 0;
    float mOnMax = 0;
    const unsigned starts0 = g_nStarts;
    struct OvlEp { double t0, t1; float pIn, pMax; };
    std::vector<OvlEp> ovl;
    if (wx.ovl) g_tickHook = ovlTick, g_ovlFrom = wx.statsFrom;
    const float mCrit = 1.f - (float) converter.getRectOnPwmMin() /
                              ((float) converter.pwmMaxDriver() + (float) converter.getDtLhTicks());

#if SCAN_ACTIVE_DEFINED
    ItState it{};
    if (const char *e = getenv("SIM_ACT")) it.act = !strcmp(e, "psu") ? 1 : !strcmp(e, "manual") ? 2 : !strcmp(e, "limit") ? 3 : 0;
#endif
    float ds[4] = {0, 0, 0, 0};
    const bool dstep = name == "wx-dstep" && getenv("SIM_DSTEP") && sscanf(getenv("SIM_DSTEP"), "%f,%f,%f,%f", ds, ds + 1, ds + 2, ds + 3) == 4;
    int dsN = dstep ? 2 * (int) std::lround((ds[1] - ds[0]) / ds[2]) + 1 : 0, dsI = -1;
    double dsT0 = 30, dsAcc[5] = {};
    if (dstep) wx.T = dsT0 + ds[3] * dsN + 1;
    for (double nextTr = 0; tNow() < wx.T;) {
        const double tb = tNow();
        if (dstep && tb >= dsT0) {
            const int i = (int) ((tb - dsT0) / ds[3]);
            if (i != dsI) {
                if (dsI >= 0 && dsAcc[0] > 0)
                    printf("DSTEP %c D=%u/%u Vin=%.2f Ppv=%.3f Pm=%.3f LS=%.0f | pwmMax %u f %u Vo %.3f plant dcm %d iL %.3f iLsOff %.3f lim %u\n", dsI <= dsN / 2 ? 'u' : 'd', mppt.manualTarget.load(),
                           converter.getCtrlOnPwmCnt(), dsAcc[1] / dsAcc[0], dsAcc[2] / dsAcc[0], dsAcc[3] / dsAcc[0], dsAcc[4] / dsAcc[0],
                           g_vconv.getPwm().pwmMax, (unsigned) g_vconv.getPwm().pwmFreq, g_vconv.getVout(), (int) g_vconv.inDcm(), g_vconv.getIL(), g_vconv.getILsOff(), (unsigned) mppt.teleSnap().limIdx);
                dsI = i, std::fill(dsAcc, dsAcc + 5, 0.0);
                const int k = i <= dsN / 2 ? i : dsN - 1 - i;
                if (i < dsN) {
                    const auto d = (uint16_t) std::lround(ds[0] + ds[2] * (float) k);
                    mppt.manualTarget = d;
                    if (getenv("SIM_DSTEP_MPPT")) mppt.speedScale = 0, converter.pwmPerturb((int16_t) (d - converter.getCtrlOnPwmCnt()));
                }
            }
            if (tb - dsT0 - ds[3] * i > ds[3] * 0.5) {
                const float v = g_vconv.getVin();
                dsAcc[0] += 1, dsAcc[1] += v, dsAcc[2] += v * g_vconv.pvCurrent(v),
                    dsAcc[3] += sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get(), dsAcc[4] += converter.getRectOnPwmCnt();
            }
        }
        wx.apply(tb);
        {
            static double openAt = getenv("SIM_OPEN_AT") ? atof(getenv("SIM_OPEN_AT")) : -1;
            static bool opened = false;
            static float vmx = 0;
            if (openAt >= 0 && tb >= openAt) {
                printf("%9.3f >>> OPEN bat: D=%u Vin=%.2f Vout=%.2f Iout=%.2f mode=%s\n", tb, converter.getCtrlOnPwmCnt(),
                       g_vconv.getVin(), g_vconv.getVout(), g_vconv.getIoutAvg(), modeStr());
                g_vconv.setBat(0, 1e9f), g_vconv.takeVoutMax(), openAt = -1, opened = true;
            } else if (opened) {
                const float m = g_vconv.takeVoutMax();
                if (m > vmx) vmx = m, printf("%9.3f >>> VOUTMAX %.2f\n", tb, vmx);
            }
        }
        if (holdAt >= 0 && tb >= holdAt) mppt.speedScale = 0, holdAt = -1;
        {
            // SIM_RIPPLE_DRIFT=dHz,period: the bus ripple frequency swings +-dHz around vbat_ac_freq (off-grid inverter)
            static float rd[3] = {0, 1, -1};
            static bool rdOn = getenv("SIM_RIPPLE_DRIFT") && sscanf(getenv("SIM_RIPPLE_DRIFT"), "%f,%f", rd, rd + 1) == 2;
            if (rdOn) {
                if (rd[2] < 0) rd[2] = g_vconv.getVbatAcFreq();
                g_vconv.setBatRipple(g_vconv.getVbatAcAmp(), rd[2] + rd[0] * std::sin(2 * (float) M_PI * (float) tb / rd[1]),
                                     g_vconv.getVbatAcShape());
            }
        }
        {
            // SIM_DC=ta,duty,tb: console `dc duty` at ta, `mppt` at tb
            static float dc[3] = {-1, 0, -1};
            static bool dcInit = getenv("SIM_DC") && sscanf(getenv("SIM_DC"), "%f,%f,%f", dc, dc + 1, dc + 2) == 3;
            if (dcInit && dc[0] >= 0 && tb >= dc[0]) {
                printf("%9.2f >>> dc %.0f (D=%u Vin=%.2f sweeping=%d)\n", tb, dc[1], converter.getCtrlOnPwmCnt(),
                       g_vconv.getVin(), (int) mppt.isSweeping());
                (void) mppt.requestPsuManual((uint16_t) dc[1], -1), dc[0] = -1;
            } else if (dcInit && dc[0] < 0 && dc[2] >= 0 && tb >= dc[2]) {
                printf("%9.2f >>> mppt (D=%u Vin=%.2f)\n", tb, converter.getCtrlOnPwmCnt(), g_vconv.getVin());
                (void) mppt.requestPsuOff(), dc[2] = -1;
            }
        }
        if (wx.sweepAt >= 0 && tb >= wx.sweepAt) {
            printf("%9.2f >>> sweep requested\n", tb);
            mppt.startSweep();
            wx.sweepAt = -1;
            if (!wx.sweepsAt.empty()) wx.sweepAt = wx.sweepsAt.front(), wx.sweepsAt.erase(wx.sweepsAt.begin());
        }
        if (wx.bmsAt >= 0 && tb >= wx.bmsAt) {
            static double nextI = 0, nextC = 0;
            if (tb >= nextI) mppt.charger.batSt.updateBatCurrent(g_vconv.getIoutAvg()), nextI = tb + 4;
            if (tb >= nextC) mppt.charger.batSt.setVcellHigh(wx.vcell), nextC = tb + 12;
        }
        const double q0 = g_qOut;
        run(0.02, 0);
        const double dt = tNow() - tb;
#if SCAN_ACTIVE_DEFINED
        if (it.act) itStep(it, tb);
#endif
        if (tb >= wx.statsFrom && dt > 0) {
            const double im = (g_qOut - q0) / dt;
            iMin = std::min(iMin, im);
            if (im < -0.05) tNeg += dt;
            if (im < 0) qNeg -= im * dt;
            if (!converter.disabled() && g_vconv.getVin() > 1) {
                const float m = g_vconv.getVout() / g_vconv.getVin();
                mOnMax = std::max(mOnMax, m);
                if (m > mCrit) tAbove += dt, tAboveDcm += converter.inDCM() ? dt : 0;
            }
        }

        const float now[7] = {wx.g[0], wx.g[1], wx.g[2], wx.g[3], wx.g[4], wx.g[5], wx.vocNow};
        if (memcmp(now, key, sizeof key)) nPk = pvPeaks(pk, valley), memcpy(key, now, sizeof key);
        // reference: highest peak a buck can reach (above the output voltage)
        const float vReach = g_vconv.getVout() * 1.05f, vin = g_vconv.getVin();
        int iG = -1, iOp = 0, nReach = 0;
        for (int i = 0; i < nPk; ++i) {
            if (pk[i].v < vReach) continue;
            ++nReach;
            if (iG < 0 || pk[i].p > pk[iG].p) iG = i;
        }
        while (iOp + 1 < nPk && vin > valley[iOp]) ++iOp;
        const float pG = iG < 0 ? 0 : pk[iG].p, pPv = vin * g_vconv.pvCurrent(vin);
        pGLast = pG;
        const bool on = !converter.disabled();
        const bool sweeping = mppt.isSweeping() || (mppt.getState() == MpptControlMode::Sweep && !SCAN_ACTIVE());
        const bool isWrong = on && !sweeping && !SCAN_ACTIVE() && iG >= 0 && iOp != iG && pG > pk[iOp].p * 1.02f;
#if SCAN_ACTIVE_DEFINED
        {
            const auto &s = mppt.scan;
            const bool act = s.active() && !s.ret;
            if (g_scanEnd && pr.n && !pr.end) pr.end = g_scanEnd, pr.bestV = g_scanBestV, pr.tIdle = -1;
            g_scanEnd = 0;
            if (pr.end && !s.active() && pr.tIdle < 0) pr.tIdle = tb;
            // landed: 5 s after the scan went idle, or earlier when a sweep or the next scan starts
            if (pr.end && (pr.end == 'l' || (pr.tIdle >= 0 && tb >= pr.tIdle + 5) || sweeping || (act && !prAct))) {
                const float pOp = pr.pk[pr.iOp].p, g = pr.pk[pr.iG].p / pOp;
                float relG = 0, errG = NAN;
                char buf[400];
                int o = snprintf(buf, sizeof buf, "== PROBE t=%.0f v0 %.1f op %.1fV/%.1fW G %.1fV/%.1fW x%.3f |", tb, pr.v0,
                                 pr.pk[pr.iOp].v, pOp, pr.pk[pr.iG].v, pr.pk[pr.iG].p, g);
                for (int i = 0; i < pr.nc; ++i) {
                    const int b = basin(pr, pr.vc[i]);
                    const float e = pr.vc[i] / pr.pk[b].v - 1;
                    if (b == pr.iG && pr.rel[i] > relG) relG = pr.rel[i], errG = e;
                    o += snprintf(buf + o, sizeof buf - o, " %.1fV@%.1f %+.1f%% x%.3f", pr.vc[i], pr.pk[b].v, 100 * e, pr.rel[i]);
                }
                // the plant now: start basin, landed basin, global
                int b0 = 0;
                while (b0 + 1 < nPk && pr.v0 > valley[b0]) ++b0;
                const int bl = iOp;
                const float gNow = iG < 0 ? 1 : pk[iG].p / pk[bl].p;
                int c;
                const char *cn[] = {"ok", "HIT", "MISS-place", "MISS-noise", "WRONG", "PARTIAL", "light"};
                if (pr.end == 'l') c = 6;
                else if (pr.end == 'm' && bl != b0) c = bl == iG ? 1 : pk[bl].p > pk[b0].p ? 5 : 4;
                else c = gNow <= 1.03f ? 0 : relG > 1.03f ? 3 : 2;
                ++prN[c];
                if (pr.iOp != pr.iG && g > 1.03f && !std::isnan(errG))
                    ++prNG, prErrG += std::fabs(errG), prErrGMax = std::max(prErrGMax, (double) std::fabs(errG)), prRelG += relG / g;
                printf("%s | landed %.1fV, G now %.1fV x%.3f | %s%s\n", buf, vin, iG < 0 ? 0.f : pk[iG].v, gNow, cn[c],
                       pr.iOp != pr.iG && std::isnan(errG) ? " (G not probed)" : "");
                pr.n = 0, pr.end = 0;
            }
            if (act && !prAct && iG >= 0) {
                std::copy_n(pk, 6, pr.pk), std::copy_n(valley, 6, pr.valley), std::copy_n(s.vc, 6, pr.vc);
                pr.n = nPk, pr.nc = s.nc, pr.v0 = s.v0, pr.iG = iG, pr.iOp = basin(pr, s.v0), pr.end = 0;
                for (int i = 0; i < pr.nc; ++i) pr.rel[i] = pr.vc[i] * g_vconv.pvCurrent(pr.vc[i]) / pr.pk[pr.iOp].p;
            }
            prAct = act;
        }
#endif

        if (tb >= nextTr) {
            printf("%9.2f  %.3f %.3f %.3f | %5.1f %6.1f | %5.1f %6.1f  %d(%d) | %-11s %4u %2u%s\n", tNow(), wx.g[0],
                   wx.n > 1 ? *std::min_element(wx.g + 1, wx.g + wx.n) : 0.f,
                   wx.n > 2 ? *std::max_element(wx.g + 1, wx.g + wx.n) : 0.f, vin, pPv, iG < 0 ? 0.f : pk[iG].v, pG, nPk, nReach,
                   modeStr(), converter.getCtrlOnPwmCnt(), (unsigned) mppt.teleSnap().limIdx, isWrong ? "  WRONG PEAK" : "");
            nextTr += trEvery;
        }
        if (tb < wx.statsFrom) continue;
        if (wx.ovl) {
            // IoutCTRL (3) / PowerCTRL (4) episodes, gaps below 1 s merged
            const unsigned li = mppt.teleSnap().limIdx;
            if (on && (li == 3 || li == 4)) {
                if (ovl.empty() || tb - ovl.back().t1 > 1) ovl.push_back({tb, tb, pPv, pPv});
                ovl.back().t1 = tNow(), ovl.back().pMax = std::max(ovl.back().pMax, pPv);
            }
        }
        if (g_ring && on && !sweeping) dutyHist[std::min<size_t>(1023, converter.getCtrlOnPwmCnt() / 4)] += dt;

        maxPk = std::max(maxPk, nReach);
        tStat += dt;
        eG += pG * dt;
        // what the battery takes at the controller's Vout cap
        const float vCap = mppt.charger.Vout_max();
        eUse += std::min(pG, std::max(0.f, vCap * (vCap - g_vconv.getVbat()) / g_vconv.getRbat())) * dt;
        eOut += g_vconv.getVout() * g_vconv.getIoutAvg() * dt;
        ePv += pPv * dt;
        const float pCtl = sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get();
        if (std::isfinite(pCtl)) eCtl += pCtl * dt;
        const int cat = !on ? OFF : sweeping ? SWEEP : SCAN_ACTIVE() ? SCAN
                        : mppt.teleSnap().limIdx != 15 ? LIMIT : isWrong ? WRONG : TRACK;
        tCat[cat] += dt;
        loss[cat] += (pG - pPv) * dt;
        const auto &trk = mppt.tracker;
        if (cat == TRACK) {
            const float d = converter.getCtrlOnPwmCnt();
            dN += dt, dS += d * dt, dS2 += d * d * dt, vS += vin * dt, vS2 += vin * vin * dt;
            dMin = std::min(dMin, d), dMax = std::max(dMax, d), vMin = std::min(vMin, vin), vMax = std::max(vMax, vin);
        }
        if (trk.slowMode) {
            tSlow += dt;
            if (cat == TRACK && iG >= 0) lossSlow += (pG - pPv) * dt, slowDvMax = std::max(slowDvMax, std::fabs(vin / pk[iG].v - 1));
        }
        nSlowIn += trk.slowMode && !wasSlow, wasSlow = trk.slowMode;
        nRev += trk._timeLastReverse != lastRev, lastRev = trk._timeLastReverse;
        if (isWrong) {
            if (!inWrong) wrong.push_back({tb, 0, 0, 0, 0, 0, 0});
            auto &w = wrong.back();
            if (w.pOp <= 0 || pG * w.pOp > w.pG * pk[iOp].p) w.vOp = pk[iOp].v, w.pOp = pk[iOp].p, w.vG = pk[iG].v, w.pG = pG;
            wrong.back().t1 = tNow();
            wrong.back().lost += (pG - pPv) * dt;
        }
        inWrong = isWrong;
    }

    std::string trips;
    size_t nTrips = 0;
    {
        std::vector<std::pair<std::string, int>> byName;
        for (auto &t: g_trips) {
            if (t.t < wx.statsFrom) continue;
            ++nTrips;
            auto it = std::find_if(byName.begin(), byName.end(), [&](auto &p) { return p.first == t.who; });
            if (it == byName.end()) byName.emplace_back(t.who, 1);
            else ++it->second;
        }
        for (auto &[who, cnt]: byName) trips += (trips.empty() ? "" : ", ") + who + " x" + std::to_string(cnt);
    }
    size_t nSweeps = 0;
    for (double t: g_sweepT) nSweeps += t >= wx.statsFrom;
    double tWrong = 0;
    for (auto &s: wrong) tWrong += s.t1 - s.t0;
    const double pc = eG > 0 ? 100 / eG : 0;
    const char *hz = getenv("SIM_ADC_HZ");
    printf("\n== %s @%s Hz, %.0f s, up to %d reachable peak(s)\n", name.c_str(), hz ? hz : "450", tStat, maxPk);
    printf("== PLANT TRUTH: PV energy %.2f Wh = %.1f %% of the reachable GMPP (%.2f Wh), %.1f %% of what the battery"
           " takes below the Vout cap (%.2f Wh); lost %% of GMPP while off %.1f / sweep %.1f / limiter %.1f /"
           " wrong peak %.1f / tracking %.1f; on a non-global peak %.1f s, off %.1f s; output energy %.2f Wh\n",
           ePv / 3600, ePv * pc, eG / 3600, eUse > 0 ? 100 * ePv / eUse : 0, eUse / 3600, loss[OFF] * pc, loss[SWEEP] * pc,
           loss[LIMIT] * pc, loss[WRONG] * pc, loss[TRACK] * pc, tWrong, tCat[OFF], eOut / 3600);
    printf("== CONTROLLER: %zu sweeps, limiter active %.1f %% of the time, sweeping %.1f %%, %zu trips [%s],"
           " measured output energy %.2f Wh\n", nSweeps, 100 * tCat[LIMIT] / tStat, 100 * tCat[SWEEP] / tStat, nTrips,
           trips.c_str(), eCtl / 3600);
    // plant truth: harvest, wrong-peak, off; controller: sweeps, limiter, trips
    size_t nScan = 0, nMoved = 0, nLight = 0;
    for (auto &[t, c]: g_scanT) if (t >= wx.statsFrom) ++nScan, nMoved += c == 'm', nLight += c == 'l';
    printf("== SUMMARY %-18s %5s Hz | harvest %5.1f %% of GMPP, %5.1f %% of cap-limited | wrong-peak %6.1f s | off %6.1f s |"
           " sweeps %2zu | limiter %4.1f %% | trips %zu [%s]\n", name.c_str(), hz ? hz : "450", ePv * pc,
           eUse > 0 ? 100 * ePv / eUse : 0, tWrong, tCat[OFF], nSweeps, 100 * tCat[LIMIT] / tStat, nTrips, trips.c_str());
    printf("== SCAN %-18s %zu scans (%zu moved, %zu light changed), scanning %.1f s, lost while scanning %.3f Wh = %.2f %% of"
           " GMPP, %.1f J per scan; wrong-peak loss %.2f %%\n", name.c_str(), nScan, nMoved, nLight, tCat[SCAN],
           loss[SCAN] / 3600, loss[SCAN] * pc, nScan ? loss[SCAN] / (double) nScan : 0.0, loss[WRONG] * pc);
    for (auto &[t, c]: g_scanT) if (t >= wx.statsFrom && nScan <= 40) printf("==   scan at t=%.1f s %c\n", t, c);
    printf("== PROBES %-18s ok %d hit %d miss-place %d miss-noise %d wrong %d partial %d light %d | probe in the global"
           " basin (op off it, gain > 3 %%): %d, |dV| mean %.1f %% max %.1f %%, read / true gain %.3f\n", name.c_str(), prN[0],
           prN[1], prN[2], prN[3], prN[4], prN[5], prN[6], prNG, prNG ? 100 * prErrG / prNG : 0.0, 100 * prErrGMax,
           prNG ? prRelG / prNG : 0.0);
    if (mppt.scan.unsaved()) { // learned in this run: the save and the reload
        mppt.scanSave();
        ConfFile tc{"/littlefs/conf/tracker.conf"};
        printf("== SCAN learned %u substrings, tracker.conf pv_substrings=%u\n", mppt.scan.n, tc.getByte("pv_substrings", 0));
    }
    {
        size_t nMp = 0;
        for (auto &t: g_trips) nMp += t.t >= wx.statsFrom && t.who == "minpulse-M";
        printf("== REVERSE %-18s plant Iout 20 ms mean min %.3f A, %.1f s below -0.05 A, %.3f C back | starts %u,"
               " minpulse-M stops %zu | on at plant M > %.3f (1 - t_min/T) %.1f s (%.1f s DCM), max on M %.3f\n",
               name.c_str(), iMin, tNeg, qNeg, g_nStarts - starts0, nMp, mCrit, tAbove, tAboveDcm, mOnMax);
    }
    {
        size_t nLogged = 0;
        for (double t: g_slowDownT) nLogged += t >= wx.statsFrom;
        const double dm = dN > 0 ? dS / dN : 0, vm = dN > 0 ? vS / dN : 0, k = 1000.0 / converter.pwmCtrlMax;
        printf("== TRACKER %-18s tracking %.0f s: duty mean %.0f rms %.1f p-p %.0f counts of %u (%.2f / %.1f permille),"
               " Vin mean %.2f rms %.3f p-p %.2f V | tracking loss %.2f %% | slow %.1f %% of the time, %u entries"
               " (%zu logged, every %.0f s) | %.1f reversals/min | harvest %.2f %% | tracking loss %.0f J, in slow mode"
               " %.0f J, slow-mode Vin up to %.1f %% off the MPP\n", name.c_str(), dN, dm,
               std::sqrt(std::max(0.0, dS2 / std::max(dN, 1e-9) - dm * dm)), dMax - dMin, converter.pwmCtrlMax,
               std::sqrt(std::max(0.0, dS2 / std::max(dN, 1e-9) - dm * dm)) * k, (dMax - dMin) * k, vm,
               std::sqrt(std::max(0.0, vS2 / std::max(dN, 1e-9) - vm * vm)), vMax - vMin, loss[TRACK] * pc,
               100 * tSlow / tStat, nSlowIn, nLogged, nLogged ? tStat / nLogged : 0.0, nRev * 60 / tStat, ePv * pc,
               loss[TRACK], lossSlow, 100 * slowDvMax);
    }
    size_t shown = 0;
    for (auto &s: wrong) {
        if (s.t1 - s.t0 < 2 || ++shown > 12) continue;
        printf("==   wrong peak t=%.1f..%.1f s: on the %.1f V peak (%.1f W) while the %.1f V peak gives %.1f W (up to +%.0f %%),"
               " %.2f Wh lost\n", s.t0, s.t1, s.vOp, s.pOp, s.vG, s.pG, 100 * (s.pG / s.pOp - 1), s.lost / 3600);
    }
    shown = 0;
    for (double t: g_sweepT) if (t >= wx.statsFrom && ++shown <= 12) printf("==   sweep at t=%.1f s\n", t);
    shown = 0;
    for (auto &t: g_trips)
        if (t.t >= wx.statsFrom && ++shown <= 12) printf("==   trip at t=%.1f s [%s] %us\n", t.t, t.who.c_str(), t.sec);
    if (nTrips > 12) printf("==   ... %zu trips total\n", nTrips);
    if (wx.maxCals) {
        // each failure: off for the 4 s backoff, then back on (a sweep on the old zeros). Harvest floor: 97.7 % on
        // LEDC timing, 97.0 % on the MCPWM preset (more tracking loss under the ripple); the failures cost ~0.4 %
        double offMin = 1e9, offMax = 0;
        for (double tf: g_calFailT) {
            auto on = std::upper_bound(g_onT.begin(), g_onT.end(), tf);
            const double off = on == g_onT.end() ? 1e9 : *on - tf;
            offMin = std::min(offMin, off), offMax = std::max(offMax, off);
        }
        const bool ok = g_calFailT.size() >= 2 && g_calStarts <= wx.maxCals && offMin >= 3.9 && offMax <= 15 &&
                        ePv * pc >= 96.5;
        printf("== CALFAIL %s: %zu failed calibrations (want >= 2), %u started (want <= %zu), off %.2f..%.2f s after a"
               " failure (want 3.9..15), harvest %.1f %% (want >= 96.5): %s\n", name.c_str(), g_calFailT.size(),
               g_calStarts, wx.maxCals, offMin, offMax, ePv * pc, ok ? "PASS" : "FAIL");
        if (!ok) return 1;
    }
    if (name == "wx-dawn") {
        // no-MPP backoff doubles 30..300 s and the converter stays off for all of it
        unsigned n = 0;
        bool ok = true;
        for (auto &tr: g_trips) {
            if (tr.who != "sweep-no-mpp") continue;
            auto s = std::upper_bound(g_sweepT.begin(), g_sweepT.end(), tr.t);
            ok &= tr.sec == std::min(30u << std::min(n, 4u), 300u) && (s == g_sweepT.end() || *s >= tr.t + tr.sec - 0.1);
            ++n;
        }
        ok &= n >= 2 && n <= 6;
        printf("== NOMPP %s: %u no-MPP backoffs (want 2..6), doubling and held: %s\n", name.c_str(), n, ok ? "PASS" : "FAIL");
        if (!ok) return 1;
    }
    if (wx.bmsResweep >= 0) {
        for (auto &s: g_stops) printf("==   sweep stop t=%.1f s MPP %.1f W at %.1f V\n", s.t, s.p, s.v);
        const SweepStop last = g_stops.empty() ? SweepStop{0, 0, 0} : g_stops.back();
        unsigned noMpp = 0;
        for (auto &tr: g_trips) noMpp += tr.who == "sweep-no-mpp";
        // -high: the sweep that finds the MPP starts within 10 s of the BMS data
        double t0 = 1e9;
        for (double t: g_sweepT) if (!g_stops.empty() && t < g_stops.front().t) t0 = t;
        const bool ok = wx.bmsResweep == 2 ? last.p >= 0.95f * pGLast && t0 < wx.bmsAt + 10
                        : wx.bmsResweep ? g_stops.size() >= 2 && last.p >= 0.95f * pGLast : g_stops.size() == 1;
        printf("== BMSCAP %s: %zu sweep stops (want %s), last MPP %.1f W of GMPP %.1f W, %u no-MPP backoffs, Vout cap"
               " %.2f V at the end: %s\n", name.c_str(), g_stops.size(),
               wx.bmsResweep == 2 ? "its sweep by BMS data + 10 s" : wx.bmsResweep ? ">= 2, last >= 95 %" : "1",
               last.p, pGLast, noMpp, mppt.charger.Vout_max(), ok ? "PASS" : "FAIL");
        if (!ok) return 1;
    }
    if (wx.ovl) {
        size_t nOc = 0, nRise = 0;
        for (auto &t: g_trips) nOc += t.t >= wx.statsFrom && t.who == "Iout-OC";
        const OvlEp *w = nullptr;
        for (auto &e: ovl) {
            nRise += e.pMax > 1.05f * e.pIn;
            if (!w || e.pMax / e.pIn > w->pMax / w->pIn) w = &e;
        }
        for (auto &e: ovl) if (ovl.size() <= 12) printf("==   limiter t=%.1f..%.1f s: PV %.1f W at entry, peak %.1f W\n", e.t0, e.t1, e.pIn, e.pMax);
        printf("== OVL %s: %zu Iout-OC trips, %zu limiter episodes (%zu with PV power rising > 5 %%), max Iout/Iout_max %.2f"
               " (ewm, med3 %.2f; Iout_max %.1f A), PV power at limit entry vs peak during episode %.1f -> %.1f W (worst, t=%.1f s): %s\n",
               name.c_str(), nOc, ovl.size(), nRise, g_ovlEwm, g_ovlMed, mppt.limits.Iout_max, w ? w->pIn : 0.f, w ? w->pMax : 0.f,
               w ? w->t0 : 0.0, nOc ? "FAIL" : "PASS");
        if (nOc) return 1;
    }
    if (!wx.verdict.empty()) {
        const bool ok = g_verdict == wx.verdict && nSweeps == wx.sweeps;
        printf("== VERDICT %s: events %s (want %s), %zu sweeps after %.0f s (want %zu): %s\n", name.c_str(),
               g_verdict.c_str(), wx.verdict.c_str(), nSweeps, wx.statsFrom, wx.sweeps, ok ? "PASS" : "FAIL");
        if (!ok) return 1;
    }
    if (g_ring) {
        g_tickHook = nullptr;
        if (!g_sweepPts.empty()) ringSweepReport();
        ringStaticReport("tracking", dutyHist);
    }
#if SCAN_ACTIVE_DEFINED
    if (it.act) {
        printf("== IT %s: %s%s\n", getenv("SIM_ACT"), it.fail ? "FAIL " : it.done ? "PASS" : "FAIL scan never active",
               it.fail ? it.why : "");
        if (it.fail || !it.done) return 1;
    }
#endif
    return 0;
}

// wx-* scenario table; false = unknown name
static bool wxScenario(const std::string &sc, Wx &wx) {
    // fry-like string: Voc 80 V, peaks near 68 V (all), 57 V and 30 V (substrings bypassed)
    auto fry = [&](float pClear) { wx.voc = 80, wx.k = 0.86f, wx.pClear = pClear; };
    // fry/flat geometry: 2 modules x 3 bypass diodes, local peaks near m/6 of the string's Vmpp
    auto fry6 = [&](float pClear) {
        fry(pClear), wx.n = 6, wx.vd = 0.45f, wx.exact = true;
        if (const char *e = getenv("SIM_K")) wx.k = atof(e);
        if (const char *e = getenv("SIM_W")) sscanf(e, "%f,%f,%f,%f,%f,%f", wx.w, wx.w + 1, wx.w + 2, wx.w + 3, wx.w + 4, wx.w + 5);
        if (const char *e = getenv("SIM_HALF")) wx.half = atof(e);
        if (const char *e = getenv("SIM_G")) // per-substring Isc mismatch factors
            sscanf(e, "%f,%f,%f,%f,%f,%f", wx.gm, wx.gm + 1, wx.gm + 2, wx.gm + 3, wx.gm + 4, wx.gm + 5);
        if (const char *e = getenv("SIM_VOCDRIFT")) sscanf(e, "%f,%lf,%lf", &wx.vocDrift, &wx.driftT, &wx.driftT0);
    };
    if (sc == "wx-cloud" || sc == "wx-cloud-n1") { // -n1: single peak known (pv_substrings=1)
        wx.cloud = kCloud1120, wx.T = 900;
    } else if (sc == "wx-cloud-cv" || sc == "wx-cloud-cv-ripple") {
        // battery high enough that full sun runs into the Vout cap and every cloud releases it
        wx.cloud = kCloud1120, wx.T = 900, wx.vbat = 28.2f;
    } else if (sc == "wx-shade2" || sc == "wx-shade2-move") {
        // one of 6 substrings at 0.7: 57 V peak 18 % above the 68.8 V peak (fry, mornings); -move: starts unshaded
        fry6(215);
        if (sc == "wx-shade2") wx.sub[5] = {{0, 0.70f}};
        else wx.sub[5] = {{120, 1}, {420, 0.70f}}, wx.T = 2400;
    } else if (sc == "wx-shade3" || sc == "wx-shade3-move") {
        fry6(300);
        if (sc == "wx-shade3") {
            wx.sub[3] = wx.sub[4] = {{0, 0.75f}}, wx.sub[5] = {{0, 0.5f}}; // peaks 34 / 57 / 68.8 V, global = 57 V
        } else {
            // one module shaded 0.3 -> 0.72: global 34 V peak -> 68.8 V peak 50 % above it
            wx.sub[3] = wx.sub[4] = wx.sub[5] = {{120, 0.3f}, {420, 0.72f}}, wx.T = 2400;
        }
    } else if (sc == "wx-shade-gen") {
        // SIM_SHADE=k,s,mode[,k2,s0,tr]: the last k substrings at s, k2 before them at (1 + s) / 2; mode a: shade ramps
        // in over 120..120 + tr s (tr 300; boot sweep unshaded), l: lifts, r: ramps from s0 to s, s: static, u: boot sweep
        // unshaded, steps to s0 at 120 s and to s at 120 + tr s, v: as u, the k2 unshaded until 120 + tr s. SIM_P (300 W)
        fry6(getenv("SIM_P") ? atof(getenv("SIM_P")) : 300.f), wx.T = 1800;
        int k = 2, k2 = 0;
        float sh = 0.6f, s0 = 1, tr = 300;
        char mode = 'a';
        if (const char *e = getenv("SIM_SHADE")) sscanf(e, "%d,%f,%c,%d,%f,%f", &k, &sh, &mode, &k2, &s0, &tr);
        if (mode == 'l') s0 = sh, sh = 1;
        for (int i = 6 - k - k2; i < 6; ++i) {
            const float f = i >= 6 - k ? sh : 0.5f * (1 + sh), f0 = i >= 6 - k ? s0 : 0.5f * (1 + s0);
            wx.sub[i] = mode == 's' ? Track{{0, f}}
                        : mode == 'u' || mode == 'v'
                            ? Track{{120, 1}, {125, mode == 'v' && i < 6 - k ? 1 : f0}, {120 + tr, mode == 'v' && i < 6 - k ? 1 : f0},
                                    {125 + tr, f}}
                                      : Track{{120, f0}, {120 + tr, f}};
        }
    } else if (sc == "wx-shade-learn") {
        fry6(215), wx.T = 3400;
        wx.sub[5] = {{0, 0.70f}, {1900, 0.70f}, {1960, 1}, {2000, 1}, {2300, 0.70f}};
    } else if (sc == "wx-shade-walk") {
        // a shadow (0.35) walks across the substrings, about 2.5 of them at a time: the global peak moves
        fry6(400), wx.T = 2400;
        for (int i = 0; i < 6; ++i) {
            const float t0 = 200 + 150.f * (float) i;
            wx.sub[i] = {{t0, 1}, {t0 + 60, 0.35f}, {t0 + 400, 0.35f}, {t0 + 460, 1}};
        }
    } else if (sc == "wx-shade-cloud") {
        // beam shade on one substring under the cloud series: every cloud removes the second peak
        fry6(615);
        wx.cloud = kCloud1120, wx.sub[5] = {{0, 0.664f}}, wx.beamShade = true, wx.T = 900;
    } else if (sc == "wx-dawn" || sc == "wx-dusk") {
        // irradiance through the start threshold, doubling every 350 s (flat 2026-11-14 06:59..07:34 UTC)
        // ln(irradiance): below the start threshold, 500 s up to a 1.8 W MPP, then doubling
        wx.vocLog = 2.6f, wx.T = 2200;
        for (float t = 0; t <= wx.T; t += 10) {
            const float lg = t < 120 ? -17.7f + t / 120 * 5.7f : t < 620 ? -12 + (t - 120) / 500 * 6.2f
                                                                 : -5.8f + (t - 620) / 350 * 0.6931f;
            wx.cloud.emplace_back(sc == "wx-dawn" ? t : (float) wx.T - t, std::exp(lg));
        }
        if (sc == "wx-dusk") std::reverse(wx.cloud.begin(), wx.cloud.end());
    } else if (sc == "wx-dstep") {
        // dusk light held at SIM_GC (default 0.039), manual duty stepped by SIM_DSTEP=d0,d1,step,dwell up and back
        const float g = getenv("SIM_GC") ? atof(getenv("SIM_GC")) : 0.039f;
        wx.vocLog = 2.6f, wx.cloud = {{0, g}}, wx.manual = getenv("SIM_DSTEP_MPPT") ? 0 : 0.3f, wx.vbat = getenv("SIM_VBAT") ? atof(getenv("SIM_VBAT")) : 28;
        wx.T = 1e9;
    } else if (sc == "wx-low" || sc == "wx-mid" || sc == "wx-low-slow" || sc == "wx-calfail") {
        // steady light rising slowly after the boot sweep, with sensor noise: MPP 2 -> 20 W in 30 min (x2 per 9 min),
        // 55 -> 110 W in 10 min (x2 per 10 min, dawn rate) then 10 min level
        const bool low = sc == "wx-low" || sc == "wx-low-slow";
        const float g0 = (low ? 2.5f : 61.f) / wx.pClear, r = low ? 10.f : 2.f, tr = low ? 1800 : 600;
        wx.vocLog = 2.6f, wx.T = low ? 1860 : 1260;
        if (sc == "wx-calfail") wx.T = 2400, wx.maxCals = 3; // boot + 1800 s (+1 spare)
        for (float t = 0; t <= wx.T; t += 10) wx.cloud.emplace_back(t, g0 * std::pow(r, std::clamp(t - 60, 0.f, tr) / tr));
    } else if (sc.rfind("wx-sun-", 0) == 0 || sc.rfind("wx-ramp-", 0) == 0 || sc.rfind("wx-wob-", 0) == 0 ||
               sc == "wx-dusk-land-fry") {
        // steady sun, MPP SIM_P W; fry: Vin ~64 V at 70 W, flat: ~55 V
        const bool land = sc == "wx-dusk-land-fry";
        const float p = getenv("SIM_P") ? atof(getenv("SIM_P")) : land ? 6 : 70;
        if (sc.compare(sc.size() - 4, 4, "-fry") == 0) fry(615), wx.vocLog = 2.6f;
        if (land) wx.voc = 77;
        if (const char *e = getenv("SIM_VOC")) wx.voc = atof(e);
        if (sc.rfind("wx-ramp-", 0) == 0) wx.ramp = (getenv("SIM_RAMP") ? atof(getenv("SIM_RAMP")) : 2) / 6000;
        if (sc.rfind("wx-wob-", 0) == 0) {
            wx.wobA = 0.005f, wx.wobF = 0.2f;
            if (const char *e = getenv("SIM_WOB")) sscanf(e, "%f,%f", &wx.wobA, &wx.wobF);
        }
        float g = p / wx.pClear;
        for (int i = 0; i < 6; ++i) g = p / wx.pClear * wx.voc / (wx.voc + wx.vocLog * std::log(g));
        wx.cloud = {{0, g}}, wx.vbat = getenv("SIM_VBAT") ? atof(getenv("SIM_VBAT")) : land ? 25.8f : 26.1f, wx.statsFrom = 120, wx.T = 900;
        if (land) wx.sweepAt = wx.statsFrom = 100, wx.T = 200;
        if (const char *e = getenv("SIM_SWEEP_AT")) wx.sweepAt = wx.statsFrom = atof(e); // a requested sweep in the stats
    } else if (sc == "wx-mp-manual") {
        // manual duty 0.8 from a stiff sinking source, Voc 1.25 -> 1.02 x Vbat and back: DCM up through the stop ratio;
        // after the stop the manual restart must wait for the start ratio
        wx.vbat = 28.95f, wx.voc = 1.25f * wx.vbat, wx.T = 1500, wx.k = 0.95f, wx.iscFix = 20, wx.manual = 0.8f;
        wx.vocT = {{120, wx.voc}, {720, 1.02f * wx.vbat}, {780, 1.02f * wx.vbat}, {1380, wx.voc}};
        wx.statsFrom = 60;
    } else if (sc == "wx-mp-ccm") {
        // stiff sinking source into a battery below the cap, Voc 1.3 -> 1.02 x Vbat: MPPT runs in CCM (no floor there)
        // until the duty limit, then the current falls and the converter enters DCM near or above the stop ratio
        wx.vbat = 26.6f, wx.voc = 1.3f * wx.vbat, wx.T = 700, wx.k = 0.95f, wx.iscFix = 20;
        wx.vocT = {{150, wx.voc}, {500, 1.02f * wx.vbat}};
        wx.statsFrom = 60;
    } else if (sc == "wx-mp-ramp" || sc == "wx-mp-psu" || sc == "wx-mp-psu-nosink") {
        // operating point from 1.25 x Vbat down to ~1.02 x and back over 2 x 600 s from 120 s. -ramp: weak PV (k 0.78,
        // Voc 1.6 -> 1.3 x Vbat, 20 W at the start) in DCM near the ratio limit. -psu: stiff source (k 0.95, Isc 20 A,
        // sinks above Voc like a battery bus; Voc 1.25 -> 1.02 x) into a battery at the Vout cap, so the converter runs
        // light in DCM; -nosink: a bench PSU that cannot sink
        const bool pv = sc == "wx-mp-ramp";
        wx.vbat = pv ? 26.6f : 28.95f, wx.voc = (pv ? 1.6f : 1.25f) * wx.vbat, wx.T = 1500;
        if (pv) wx.k = 0.78f, wx.pClear = 20;
        else wx.k = 0.95f, wx.iscFix = 20, wx.noSink = sc == "wx-mp-psu-nosink";
        const float lo = (pv ? 1.3f : 1.02f) * wx.vbat;
        wx.vocT = {{120, wx.voc}, {720, lo}, {780, lo}, {1380, wx.voc}};
        wx.statsFrom = 60;
    } else if (sc == "wx-sweep-fall" || sc == "wx-sweep-rise" || sc == "wx-sweep-fall-n1" || sc == "wx-sweep-rise-n1") {
        // cloud edge (73 % in 10 s) starting 2 s into a sweep, one maximum, no substring count: flagged, one re-sweep
        // (-rise: 5 s after the light holds); -n1: single peak known (pv_substrings=1), P&O carries on, no re-sweep.
        // -rise: the paced ramp ends inside the edge and peaks late at a low Vin (flagged brighter).
        // SIM_EDGE_AT=t moves the edge start (the expected events then do not apply)
        const bool fall = sc.rfind("wx-sweep-fall", 0) == 0, n1 = sc.size() > 3 && sc.compare(sc.size() - 3, 3, "-n1") == 0;
        const float e = getenv("SIM_EDGE_AT") ? atof(getenv("SIM_EDGE_AT")) : 62;
        wx.cloud = {{e, fall ? 1 : 0.27f}, {e + 10, fall ? 0.27f : 1}};
        wx.sweepAt = wx.statsFrom = 60, wx.T = 240;
        wx.verdict = n1 ? "vf" : "vfrv", wx.sweeps = n1 ? 1 : 2;
    } else if (sc == "wx-bms-late" || sc == "wx-bms-none" || sc == "wx-bms-full" || sc == "wx-bms-high") {
        // reboot in sun into a charging pack while the Vout cap sits at vout_max_fallback (26.8 V) until BMS cell data
        // arrives (SIM_BMS_AT, default 20 s): the boot sweep stops at the cap. -late: cells 3.33 V, the data lifts the cap
        // to vout_max; -none: no BMS data; -full: cells above cv_eoc, the cap stays
        const float p = 590;
        float g = p / wx.pClear;
        for (int i = 0; i < 6; ++i) g = p / wx.pClear * wx.voc / (wx.voc + wx.vocLog * std::log(g));
        wx.cloud = {{0, g}}, wx.vbat = sc == "wx-bms-high" ? 27.1f : 26.3f, wx.T = 300;
        if (sc != "wx-bms-none") wx.bmsAt = getenv("SIM_BMS_AT") ? atof(getenv("SIM_BMS_AT")) : 20;
        wx.vcell = sc == "wx-bms-full" ? 3.62f : 3.33f;
        wx.bmsResweep = sc == "wx-bms-late" ? 1 : sc == "wx-bms-high" ? 2 : 0;
    } else if (sc == "wx-sweep-preempt") {
        // wx-sweep-fall, a requested sweep at 90 s pre-empts the re-sweep; a later flagged sweep still gets its re-sweep
        wx.cloud = {{62, 1}, {72, 0.27f}, {150, 0.27f}, {160, 1}, {202, 1}, {212, 0.27f}};
        wx.sweepAt = wx.statsFrom = 60, wx.sweepsAt = {90, 200}, wx.T = 400;
        wx.verdict = "vfrvfrv", wx.sweeps = 4;
    } else if (sc == "wx-sweep-one") {
        // one peak, steady sun: the boot and the 60 s sweep (verified) learn the single peak (pv_substrings=1); a sweep
        // under a falling cloud edge is flagged and starts no re-sweep
        wx.cloud = {{0, 1}, {302, 1}, {312, 0.27f}};
        wx.sweepAt = 60, wx.sweepsAt = {180, 300}, wx.statsFrom = 60, wx.T = 600;
        wx.verdict = "vv1vf", wx.sweeps = 3;
    } else if (sc == "wx-sweep-gust") {
        // falling and rising cloud edges after a flagged sweep: one re-sweep however gusty (flagged too, 10 min cap)
        wx.cloud = {{62, 1}, {72, 0.27f}, {120, 0.27f}, {130, 1}, {137, 1}, {147, 0.27f}};
        wx.sweepAt = wx.statsFrom = 60, wx.T = 400;
        wx.verdict = "vfrf", wx.sweeps = 2;
    } else if (sc == "wx-sweep-learn" || sc == "wx-sweep-learn-off") {
        // two peaks (one substring at 0.7) brightening during the sweep (a dim boot sweep: no verdict, no learning): the
        // return pass probes both and flags the sweep, which does not learn. -off: sweep_verify=0, the distorted sweep
        // learns
        fry6(615), wx.sub[5] = {{0, 0.70f}};
        wx.cloud = {{0, 0.04f}, {40, 0.04f}, {45, 0.3f}, {62, 0.3f}, {90, 1}};
        wx.sweepAt = wx.statsFrom = 60, wx.T = 240;
        const bool off = sc == "wx-sweep-learn-off";
        // -off: the scan trigger (power off its reference while the light still rises) probes once
        wx.verdict = off ? "ppLs" : "ppf", wx.sweeps = 1;
    } else if (sc.rfind("wx-ovl-", 0) == 0) {
        wx.ovl = true, wx.statsFrom = 5;
        if (sc == "wx-ovl-shade" || sc == "wx-ovl-shade-temp" || sc == "wx-ovl-shade-hold") {
            // wx-shade3-move: the 33 V peak is global at the boot sweep, the shade lifts from 120 s
            fry6(300), wx.T = 900;
            const bool lift = sc == "wx-ovl-shade";
            wx.sub[3] = wx.sub[4] = wx.sub[5] = {{120, 0.3f}, {420, lift ? 0.8f : 0.72f}};
            if (lift) wx.ioutMax = 4.7f;
            else wx.ioutMax = 6.5f, wx.chipT = {{600, 40}, {610, sc == "wx-ovl-shade-temp" ? 88.f : 87.5f}};
        } else if (sc == "wx-ovl-sun" || sc == "wx-ovl-bad" || sc == "wx-ovl-ctl") {
            fry(1500), wx.vocLog = 2.6f, wx.T = 600;
            const float e = getenv("SIM_OVL_EDGE") ? atof(getenv("SIM_OVL_EDGE")) : 2;
            if (sc == "wx-ovl-ctl") wx.chipT = {{300, 40}, {310, 82}}, wx.sweepAt = 450;
            else if (sc == "wx-ovl-sun") wx.cloud = {{0, 0.25f}, {200, 0.25f}, {200 + e, 1}};
            else wx.cloud = {{0, 0.2f}, {200, 0.2f}, {200.003f, 1}}, wx.T = 260;
        } else {
            return false;
        }
        if (const char *e = getenv("SIM_IOUT_MAX")) wx.ioutMax = atof(e);
    } else if (sc == "ring-sweep" || sc == "ring-weak") {
        // field-like string at the board's morning Voc; -sweep: a global sweep at 60 s in ~150-180 W sun
        const bool fry = !strcmp(g_ring->name, "fry");
        wx.voc = fry ? 78 : 70, wx.k = 0.86f, wx.vbat = fry ? 26.8f : 26.3f;
        if (sc == "ring-sweep") wx.pClear = fry ? 150 : 180, wx.sweepAt = wx.statsFrom = 60, wx.T = 150;
        else wx.pClear = 45, wx.statsFrom = 150, wx.T = 900;
    } else if (sc == "wx-sweep-return") {
        // two peaks, the 57 V one 18 % higher (one substring at 0.7); a cloud edge (-35 % in 1 s) after the sweep passed
        // the 68.8 V peak ranks it first on the way down: the return pass finds the 57 V one
        fry6(615), wx.sub[5] = {{0, 0.70f}};
        const float e = getenv("SIM_EDGE_AT") ? atof(getenv("SIM_EDGE_AT")) : 62;
        wx.cloud = {{e, 1}, {e + 1, 0.65f}};
        wx.sweepAt = wx.statsFrom = 60, wx.T = 240;
        wx.verdict = "ppvLppf", wx.sweeps = 1;
    } else if (sc == "wx-sweep-shade" || sc == "wx-sweep-scan") {
        // two peaks, the 68.8 V one 10 % higher; a rising cloud edge during the sweep favours the later 57 V one.
        // The edge hides the dip between them: one maximum, no return pass, one re-sweep. -scan: substring count known
        // (pv_substrings=6), the flagged sweep starts a scan
        fry6(615), wx.sub[5] = {{0, 0.91f}};
        wx.cloud = {{62, 0.27f}, {72, 1}};
        wx.sweepAt = wx.statsFrom = 60, wx.T = 660;
        wx.verdict = sc == "wx-sweep-scan" ? "vFs" : "vfrv", wx.sweeps = sc == "wx-sweep-scan" ? 1 : 2;
    } else {
        return false;
    }
    return true;
}

// LS count (fractional) where the plant's coil current at LS turn-off first turns negative as LS grows, HS fixed,
// stiff Vin/Vout; i0: coil current at HS turn-on there. NAN: none below the period.
static double plantZc(float vin, float vout, int hs, double &i0) {
    const uint32_t f = converter.getPwmFrequency();
    const auto at = [&](int ls, double &a) {
        VirtualConverter p = g_vconv;
        p.setPvBlocking(true), p.setBat(vout, 1e-6f), p.setBatRipple(0, 100);
        p.setPwmCount(0, hs), p.setPwmCount(1, ls);
        double s = 0, s0 = 0;
        for (int k = 0; k < 600; ++k) {
            p.setVin(vin), p.setVout(vout);
            const float il = p.getIL();
            p.stepSeconds(1.f / (float) f, f);
            if (k >= 300) s += p.getILsOff(), s0 += il;
        }
        a = s0 / 300;
        return s / 300;
    };
    const double id = (vin / vout - 1) * hs;
    double a, prev = at((int) (0.6 * id), a);
    for (int ls = (int) (0.6 * id) + 1; ls < converter.pwmMaxDriver() - hs - 1; ++ls) {
        const double c = at(ls, a);
        if (prev > 0 && c <= 0) return i0 = a, ls - c / (c - prev);
        prev = c;
    }
    return NAN;
}

// rect-cal force at full power: ramp-down, calibration, ramp back, against the plant. Reports trips,
// the learned offset (no gate delay in the plant: the truth is 0), the ramp and the power after.
static int runRectCal(const std::string &name, const std::string &sc) {
    settle(60);
    printf("# trips during settle: %zu\n", g_trips.size());
    auto &rc = mppt.rectCal;
    const uint16_t hs0 = converter.getCtrlOnPwmCnt();
    const float p0 = sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get();
    const bool dcm0 = converter.inDCM();
    Probe p = probeStart();
    const float vbat0 = g_vconv.getVbat(), isc0 = g_vconv.getIsc(), voc0 = g_vconv.getVoc();
    if (!rc.requestStart(true)) { printf("requestStart refused\n"); return 1; }
    printf("%9.4f >>> rect-cal force at HS %u, %s, P %.0f W\n", tNow(), hs0, dcm0 ? "DCM" : "CCM", p0);
    double tCal = -1, tEnd = -1;
    float iMin = INFINITY, ilMin = INFINITY, vinMax = 0, iRunMin = INFINITY, ilAfter = INFINITY;
    int stepMax = 0, lsOffMin = 0, nPv = 0;
    double pvSum = 0, poSum = 0;
    uint16_t hsPrev = hs0;
    const uint8_t clamps0 = converter.lsClamps();
    std::mt19937 busRng{7};
    std::uniform_real_distribution<float> busU{0.f, 1.f};
    double busNext = 0;
    float busStep = 0;
    while (tNow() - p.t0 < 60) {
        const double t = tNow() - p.t0;
        if (sc == "rcal-bus") g_vconv.setBat(vbat0 * (1.f + 0.025f * (float) std::min(1.0, t / 30.0)), g_vconv.getRbat());
        if (sc == "rcal-cloud" && t >= 12.0) g_vconv.setPv(isc0 * 0.7f, voc0 * 0.982f, g_vconv.getPvK());
        if (sc.rfind("rcal-low", 0) == 0 && t >= 0.2) g_vconv.setPv(2.f, voc0 * 0.97f, g_vconv.getPvK());
        // bus transients: Vout +0.6 V for 2 ms every 1-3 s, D-only Vout limiter touches far below the 29 V cap
        // as logged on flat 2026-11-15 (ctrl -92, act 26.7, tgt 29); too short to move a window average
        if (sc == "rcal-flat") {
            busStep = t >= busNext ? 0.6f : 0.f;
            while (t >= busNext) busNext += 1.0 + 2.0 * busU(busRng);
            g_vconv.setBat(vbat0 + busStep, g_vconv.getRbat());
        }
        if (sc == "rcal-low-bus") {
            while (t >= busNext) busNext += 2.0 + 4.0 * busU(busRng), busStep = (busU(busRng) - 0.5f) * 0.003f;
            g_vconv.setBat(vbat0 * (1.f + 0.004f * (float) std::sin(6.2832 * t / 23.) + 0.002f * (float) std::sin(6.2832 * t / 7.3)
                                    + busStep), g_vconv.getRbat());
        }
        run(0.0022, 0, &p);
        const auto st = rc.cal.get();
        if (st == RectCal::Run) {
            const uint16_t hs = converter.getCtrlOnPwmCnt();
            stepMax = std::max(stepMax, std::abs((int) hs - (int) hsPrev));
            hsPrev = hs;
            if (hs > rc.cal.hs && converter.manualRect_() && converter.getRectOnPwmCnt() != converter.getRectOnPwmMin())
                ++lsOffMin;
            if (tCal < 0 && hs == rc.cal.hs) tCal = t;
            iRunMin = std::min(iRunMin, g_vconv.getIoutAvg());
            if (hs == rc.cal.hs && converter.manualRect_()) pvSum += g_vconv.getVin(), poSum += g_vconv.getVout(), ++nPv;
        }
        iMin = std::min(iMin, g_vconv.getIoutAvg());
        ilMin = std::min(ilMin, g_vconv.getIL());
        vinMax = std::max(vinMax, g_vconv.getVin());
        if (tEnd < 0 && (st == RectCal::Done || st == RectCal::Fail)) tEnd = t;
        if (tEnd >= 0 && t > tEnd + 3) ilAfter = std::min(ilAfter, g_vconv.getIL());
        if (tEnd >= 0 && t > tEnd + 15) break;
    }
    report(name.c_str(), p);
    const float p1 = sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get();
    printf("== rect-cal: %s%s%s after %.1f s; cal HS %u reached at +%.2f s (from %u, %s), max HS step %d/sample, "
           "LS off its minimum during the ramp: %d samples, LS clamps %u\n",
           rc.cal.get() == RectCal::Done ? "done" : "failed", rc.cal.get() == RectCal::Fail ? ": " : "",
           rc.cal.get() == RectCal::Fail ? rc.cal.why : "", tEnd, rc.cal.hs, tCal, hs0, dcm0 ? "DCM" : "CCM", stepMax,
           lsOffMin, (unsigned) (uint8_t) (converter.lsClamps() - clamps0));
    printf("== rect-cal: ideal %.0f peak %.0f learned %+d applied %+d ct%s%s (plant timing error 0), %u pts, %u redone, dV %.2f%%, Iout rise "
           "%.1f mA (bound %.1f), offset now %d ct, rect_offset_cal %u\n", rc.cal.ideal, rc.cal.peak, rc.cal.learned,
           rc.cal.applied, rc.cal.clamped ? " (bound)" : "", rc.cal.cut ? " (fit ended at a dip)" : "", rc.cal.n,
           rc.cal.retries, 100 * rc.cal.dv, 1e3f * rc.cal.rise, 1e3f * rc.cal.iLs, rc.offset(), converter.rectOffsetCal());
    if (nPv && rc.cal.hs) {
        const float tr = converter.getPwmTickRate(), vi = (float) (pvSum / nPv), vo = (float) (poSum / nPv);
        double i0 = 0;
        const double zc = plantZc(vi, vo, rc.cal.hs, i0), t = zc - rc.cal.ideal;
        const float zq = [](const auto &c) { if constexpr (requires { c.zc; }) return (float) c.zc; else return NAN; }(rc.cal);
        printf("== truth: plant Vin %.2f Vout %.2f at HS %u: zero crossing %.1f ct, %+.1f ct (%+.0f ns) off ideal %.1f "
               "(plant voltages: %+.1f ct), coil current at HS turn-on %+.3f A; charge zc %+.0f ct; learned %+.0f, applied "
               "%+.0f ct off the truth\n", vi, vo, rc.cal.hs, zc, t, t / tr * 1e9, rc.cal.ideal, zc - (vi / vo - 1) * rc.cal.hs,
               i0, zq - rc.cal.ideal, rc.cal.learned - t, rc.cal.applied - t);
    }
    printf("== rect-cal points (LS - ideal : dI A):");
    for (int i = 0; i < rc.cal.n; ++i) printf(" %.0f:%+.3f", rc.cal.xs[i] - rc.cal.ideal, rc.cal.ys[i]);
    printf("\n");
    printf("== plant: min Iout avg %.2f A (during the run %.2f A), min coil current at cycle end %.3f A (from 3 s after "
           "the run %.3f A), max Vin %.2f V\n", iMin, iRunMin, ilMin, ilAfter, vinMax);
    printf("== after: HS %u (before %u), P %.0f W (before %.0f W), %s, manual LS %d\n", converter.getCtrlOnPwmCnt(), hs0,
           p1, p0, modeStr(), converter.manualRect_());
    printf("\n== log after event (first %zu lines):\n", g_logSince.size());
    for (size_t i = 0; i < g_logSince.size() && i < 60; ++i) printf("   %s\n", g_logSince[i].c_str());
    // a late offset eats the 2 % margin of the LS ratio near D = M: never past LateMaxNs
    const int hi = pwmCountsFromNs(RectCal::LateMaxNs, converter.getPwmTickRate());
    if (rc.offset() > hi || (rc.cal.get() == RectCal::Done && (rc.cal.applied > hi || (rc.cal.clamped && rc.cal.applied < rc.offset())))) {
        printf("FAIL: offset past LateMaxNs applied (%+d ct, active %+d ct)\n", rc.cal.applied, rc.offset());
        return 1;
    }
    return 0;
}

// `measure-coil l0` against the plant: mirrors measureCoilTask()/sweepL0() of src/selftest/measure_coil.cpp
// (the console task's waits become simulated time), estimator shared through selftest/coil_fit.h.
static bool simPsuWait(uint32_t ticket) {
    for (int i = 0; i < 1000 && !mppt.isPsuCommandDone(ticket); ++i) run(0.001, 0);
    return mppt.isPsuCommandDone(ticket);
}

static float med5(float *a) { return coilfit::detail::median(a, 5); }
static float g_mcIpk = 0, g_mcIpkPl = 0; // largest row Ipk of the last simMeasureL0 (L-free DCM estimate; the plant's)

// sweepL0()'s fit: uncorrected (g_mcRes0), then with the on-time offset (g_mcResT, status g_mcSt); returns the one the
// firmware reports (uncorrected unless the offset is applicable)
static coilfit::Result g_mcRes0, g_mcResT;
static int g_mcSt = 1;
static coilfit::Result mcFit(const float *d, const float *vi, const float *vo, const float *io, int n, float fsw, float zsig) {
    g_mcRes0 = coilfit::fit(d, vi, vo, io, n, fsw, zsig);
    g_mcResT = coilfit::fitOnTime(d, vi, vo, io, n, fsw, zsig, g_mcRes0, g_mcSt);
    const auto &a = g_mcRes0, &b = g_mcResT;
    printf("  measure: uncorrected L %.2f uH slope %+.2f %%/A | on-time %+.0f +-%.0f noise +-%.0f syst ns, span %.2f, status %d"
           " | corrected L %.2f uH +-%.1f/%.1f %% slope %+.2f %%/A | ring %.0f kHz %.1f periods\n", a.L * 1e6f, a.k * 100, b.dl * 1e9f, b.dlse * 1e9f,
           b.dlsys * 1e9f, b.dspan, g_mcSt, b.L * 1e6f, b.se * 100, b.sys * 100, b.k * 100, a.f * 1e-3f, a.periods);
    if (coilfit::persistL(a, b, g_mcSt) != coilfit::persistL(a, b, g_mcSt)) printf("  measure: L not persisted\n");
    return g_mcSt ? a : b;
}
// -ton scenarios: the plant's on-time offset (vconv.conf t_on_ofs) against the estimate; L vs lPlant
static int mcTonCheck(const std::string &sc, float lPlant, const std::function<void(bool, const char *)> &chk) {
    const char *e = getenv("SIM_MC_TON");
    if (!e) return 0;
    const float ton = (float) atof(e), u = std::hypot(g_mcResT.dlse, g_mcResT.dlsys);
    printf("== %s: on-time plant %+.0f ns, fitted %+.0f +-%.0f ns; L uncorrected %+.2f %%, corrected %+.2f %% vs plant\n",
           sc.c_str(), ton * 1e9f, g_mcResT.dl * 1e9f, u * 1e9f, (g_mcRes0.L / lPlant - 1) * 100, (g_mcResT.L / lPlant - 1) * 100);
    chk(g_mcSt == 0, "on-time offset applied");
    chk(std::fabs(g_mcResT.dl - ton) < 2 * u, "on-time offset within 2 sigma of the plant's");
    chk(std::fabs(g_mcResT.L / lPlant - 1) < std::fabs(g_mcRes0.L / lPlant - 1), "corrected L closer to the plant");
    return 0;
}

static coilfit::Result simMeasureL0(int steps, uint32_t dwellMs, double &secs) {
    const double t0 = tNow();
    coilfit::Result res;
    // SIM_MC_RAMP=1: the entry before the hard stop (a duty ramp-down)
    const bool ramp = getenv("SIM_MC_RAMP") != nullptr;
    if (!simPsuWait(mppt.requestPsuManual(0, -1, false, !ramp))) printf("  measure: RT transition timed out\n");
    for (const double dl = tNow() + 10; tNow() < dl && !ramp;) { // let the filters drain
        if (converter.disabled() && fabsf(sensors.Iout->ewm.avg.get()) < 1.f) break;
        run(0.01, 0);
    }
    const float fsw = (float) converter.realizedFreqHz();
    const uint16_t pwmMax = converter.pwmMaxDriver();
    const float iMaxLim = std::min(2.0f, mppt.limits.Iout_max), ipkMax = mppt.limits.Iout_max;
    g_mcIpk = g_mcIpkPl = 0;
    printf("  measure: fsw %.0f Hz pwmMax %u pwmCtrlMax %u\n", fsw, pwmMax, converter.pwmCtrlMax);
    const float vin0 = sensors.Vin->ewm.avg.get(), vout0 = sensors.Vout->ewm.avg.get();
    int lo, hi, step;
    if (coilfit::band(vin0, vout0, pwmMax, converter.pwmCtrlMax, steps, lo, hi, step)) {
        float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
        int n = 0;
        float iMaxSeen = 0;
        for (int H = lo; H <= hi && n < coilfit::kMaxPts; H += step) {
            if (!(sensors.Vin->ewm.avg.get() > sensors.Vout->ewm.avg.get() + 1.f)) break;
            if (!getenv("SIM_MC_BFOPEN") && sensors.Iout->ewm.avg.get() > 0.1f) mppt.bflow.enable(true); // SIM_MC_BFOPEN=1: the sweep before the fix
            mppt.setManualTarget((uint16_t) H);
            const double dl = tNow() + std::max<uint32_t>(dwellMs, 3000) * 1e-3 + H * 4e-3;
            while (converter.getCtrlOnPwmCnt() != H && tNow() < dl) run(0.02, 0);
            run(dwellMs * 1e-3, 0);
            float a[5], b[5], c[5];
            for (int k = 0; k < 5; ++k) {
                a[k] = sensors.Vin->ewm.avg.get(), b[k] = sensors.Vout->ewm.avg.get(), c[k] = sensors.Iout->ewm.avg.get();
                run(0.05, 0);
            }
            if (converter.disabled()) { printf("  measure: converter disabled; aborting\n"); break; }
            const float vin = med5(a), vout = med5(b), iout = med5(c), D = (float) converter.getCtrlOnPwmCnt() / converter.periodCounts();
            const bool dcm = converter.inDCM(), ok = iout > 0.02f && vin > vout;
            if (dcm && ok) d[n] = D, vi[n] = vin, vo[n] = vout, io[n] = iout, ++n, iMaxSeen = std::max(iMaxSeen, iout);
            const float ipk = coilfit::ipkDcm(D, vin, vout, iout);
            if (dcm) g_mcIpk = std::max(g_mcIpk, ipk), g_mcIpkPl = std::max(g_mcIpkPl, (g_vconv.getVin() - g_vconv.getVout()) * D / (fsw * g_vconv.getL()));
            if (iout > iMaxLim || !(ipk <= ipkMax) || !dcm) {
                printf("  measure: stop at H %d D %.3f Iout %.2f Ipk %.2f (plant %.2f) %s\n", H, D, iout, ipk,
                       (g_vconv.getVin() - g_vconv.getVout()) * D / (fsw * g_vconv.getL()), dcm ? "DCM" : "CCM");
                break;
            }
        }
        const float floor = std::max(0.15f, 0.2f * iMaxSeen);
        int nk = 0;
        for (int i = 0; i < n; ++i)
            if (io[i] >= floor) d[nk] = d[i], vi[nk] = vi[i], vo[nk] = vo[i], io[nk] = io[i], ++nk;
        res = mcFit(d, vi, vo, io, nk, fsw, 0);
    }
    // restore as the task does: ramp down, wait for the disable, back to MPPT
    mppt.setManualTarget(0);
    for (double dl = tNow() + 10.0; !converter.disabled() && tNow() < dl;) run(0.01, 0);
    const bool off = converter.disabled();
    if (off && mppt.limits.reverse_current_paranoia) mppt.bflow.enable(false);
    if (!simPsuWait(off ? mppt.requestPsuOff() : mppt.requestPsuSweep())) printf("  measure: MPPT restore timed out\n");
    secs = tNow() - t0;
    return res;
}

// `measure-coil l0` on a boost (sweepL0()'s boost path): BF on, the Iin zero, a low duty lifts Vout above Vin, then the
// band at the 80 V trip, stopped by the live Vout/Iin; trips (task: Vout 80 V or Iin 8 A, med3; RT: ovset 80 V) stop
// the PWM at once. Rows map onto the buck relations through coilfit::asBuck(). Fills g_mcIpk, g_mcVmax, g_mcImax.
static float g_preImax = 0, g_preIewm = 0; // precharge: plant input current, its ewm reading
static float g_mcVmax = 0, g_mcImax = 0, g_mcIpkPlant = 0, g_mcIpkMean = 0; // g_mcIpkMean: of the fitted rows
static coilfit::Result simMeasureL0Boost(int steps, uint32_t dwellMs, double &secs, std::string &stop) {
    const double t0 = tNow();
    coilfit::Result res;
    const float ov0 = mppt.getExplicitOvLimit(), hvTrip = std::min(80.f, 0.95f * mppt.limits.Vout_max),
                iTrip = std::min(8.f, mppt.limits.Iin_max);
    mppt.setExplicitOvLimit(fminf(ov0, hvTrip));
    bool tripped = false;
    g_mcIpk = g_mcVmax = g_mcImax = g_mcIpkPlant = 0;
    auto wait = [&](double sec) { // run in 10 ms steps with the task's trip check
        for (const double dl = tNow() + sec; !tripped;) {
            g_mcVmax = std::max(g_mcVmax, g_vconv.takeVoutMax()), g_mcImax = std::max(g_mcImax, g_vconv.getIinAvg());
            if (sensors.Vout->med3.get() > hvTrip || sensors.Iin->med3.get() > iTrip) {
                tripped = true;
                (void) mppt.requestPsuManual(0, -1, false, true);
                printf("  measure: TRIP Vout %.2f Iin %.2f\n", sensors.Vout->med3.get(), sensors.Iin->med3.get());
                break;
            }
            if (tNow() >= dl) return true;
            run(0.01, 0);
        }
        return false;
    };
    if (!simPsuWait(mppt.requestPsuManual(0, -1, false, true))) printf("  measure: RT transition timed out\n");
    for (const double dl = tNow() + 10; tNow() < dl;) {
        if (converter.disabled() && fabsf(sensors.Iin->ewm.avg.get()) < 1.f) break;
        run(0.01, 0);
    }
    const float fsw = (float) converter.realizedFreqHz();
    const uint16_t pwmMax = converter.pwmMaxDriver();
    const float vin0 = sensors.Vin->ewm.avg.get(), vout0 = sensors.Vout->ewm.avg.get();
    int lo, hi, step;
    if (!(vin0 > 5 && vin0 < 0.8f * hvTrip && vout0 < hvTrip - 2) ||
        !coilfit::band(hvTrip, hvTrip - vin0, pwmMax, converter.pwmCtrlMax, steps, lo, hi, step)) {
        stop = "refused";
        mppt.setExplicitOvLimit(ov0);
        return res;
    }
    printf("  measure: boost fsw %.0f Hz pwmMax %u band %d..%d step %d Vin %.2f Vout %.2f\n", fsw, pwmMax, lo, hi, step,
           vin0, vout0);
    mppt.bflow.enable(true);
    const auto hPre = (uint16_t) std::max(2, std::min((int) ceilf(2e-6f * fsw * pwmMax), converter.pwmCtrlMax / 8));
    auto up = [] { return sensors.Vout->ewm.avg.get() > sensors.Vin->ewm.avg.get() - 3; };
    if (!up()) mppt.setManualTarget(hPre);
    const double tPre = tNow();
    g_vconv.takeIinMax(), g_preImax = g_preIewm = 0, g_tickHook = [] {
        g_preImax = std::max(g_preImax, g_vconv.takeIinMax()), g_preIewm = std::max(g_preIewm, sensors.Iin->ewm.avg.get());
    };
    for (const double dl = tNow() + 3; !up() && tNow() < dl && wait(0.02);) {}
    g_tickHook = nullptr;
    printf("  measure: precharge at duty %u: Vout %.2f after %.3f s, plant Iin max %.2f A, Iin ewm max %.2f A\n", hPre,
           sensors.Vout->ewm.avg.get(), tNow() - tPre, g_preImax, g_preIewm);
    if (!up() || tripped) {
        printf("  measure: Vout %.2f did not follow Vin %.2f\n", sensors.Vout->ewm.avg.get(), sensors.Vin->ewm.avg.get());
        stop = "Vout did not follow";
        mppt.setManualTarget(0);
        for (const double dl = tNow() + 10; !converter.disabled() && tNow() < dl;) run(0.01, 0);
        mppt.setExplicitOvLimit(ov0);
        secs = tNow() - t0;
        return res;
    }
    float d[coilfit::kMaxPts], vi[coilfit::kMaxPts], vo[coilfit::kMaxPts], io[coilfit::kMaxPts];
    uint8_t row[coilfit::kMaxPts];
    int n = 0, rows = 0;
    auto zero = [&] {
        mppt.setManualTarget(0);
        for (const double dl = tNow() + 10; !converter.disabled() && tNow() < dl;) run(0.01, 0);
        run(std::max<uint32_t>(dwellMs, 2000) * 1e-3, 0);
        float c[5];
        for (float &x: c) x = sensors.Iin->ewm.avg.get(), run(0.05, 0);
        return med5(c);
    };
    const float z0 = zero();
    mppt.setManualTarget((uint16_t) std::max(2, lo / 3));
    for (const double dl = tNow() + 5; !(sensors.Vout->ewm.avg.get() > sensors.Vin->ewm.avg.get() + 2) && tNow() < dl &&
                                       wait(0.02);) {}
    wait(std::max<uint32_t>(dwellMs, 2000) * 1e-3);
    if (!(sensors.Vout->ewm.avg.get() > sensors.Vin->ewm.avg.get() + 2)) hi = -1, stop = "Vout not above Vin";
    for (int H = lo; H <= hi && n < coilfit::kMaxPts; H += step) {
        const float vin = sensors.Vin->ewm.avg.get(), vout = sensors.Vout->ewm.avg.get();
        if (!(vout > vin + 2)) { stop = "Vout fell to Vin"; break; }
        if ((float) H / pwmMax > 0.9f * (1 - vin / vout)) { stop = "near CCM"; break; }
        mppt.setManualTarget((uint16_t) H);
        for (const double dl = tNow() + std::max<uint32_t>(dwellMs, 3000) * 1e-3 + H * 4e-3;
             converter.getCtrlOnPwmCnt() != H && tNow() < dl && wait(0.02);) {}
        wait(dwellMs * 1e-3);
        float a[5], b[5], c[5];
        for (int k = 0; k < 5; ++k)
            a[k] = sensors.Vin->ewm.avg.get(), b[k] = sensors.Vout->ewm.avg.get(), c[k] = sensors.Iin->ewm.avg.get(), wait(0.05);
        if (tripped || converter.disabled()) { stop = tripped ? "TRIP" : "converter disabled"; break; }
        const float D = (float) converter.getCtrlOnPwmCnt() / converter.periodCounts();
        float bi = med5(a), bo = med5(b);
        const float vout1 = bo, iin = med5(c);
        coilfit::asBuck(bi, bo);
        const bool dcm = converter.inDCM(), ok = iin > 0.02f && bi > bo && bo > 0;
        if (dcm && ok) d[n] = D, vi[n] = bi, vo[n] = bo, io[n] = iin, row[n] = (uint8_t) rows, ++n;
        ++rows;
        const float ipk = coilfit::ipkDcm(D, bi, bo, iin);
        if (dcm) g_mcIpk = std::max(g_mcIpk, ipk), g_mcIpkPlant = std::max(g_mcIpkPlant, g_vconv.getVin() * D / (fsw * g_vconv.getL()));
        if (getenv("SIM_V")) printf("  row H %d D %.3f Vin %.2f Vout %.2f Iin %.3f Ipk %.2f %s\n", H, D, med5(a), vout1, iin, ipk, dcm ? "DCM" : "CCM");
        if (iin > std::min(5.f, 0.75f * iTrip)) { stop = "I"; break; }
        if (!(ipk <= mppt.limits.Iin_max)) { stop = "Ipk"; break; }
        if (!dcm) { stop = "CCM"; break; }
        if (vout1 > hvTrip - 2) { stop = "Vout near the trip"; break; }
    }
    if (stop.empty()) stop = "band end";
    const float z1 = zero();
    const float zsig = coilfit::zeroCorrect(io, n, z0, z1, row, rows);
    float iMaxSeen = 0;
    for (int i = 0; i < n; ++i) iMaxSeen = std::max(iMaxSeen, io[i]);
    const float floor = std::max(0.15f, 0.2f * iMaxSeen);
    int nk = 0;
    for (int i = 0; i < n; ++i)
        if (io[i] >= floor) d[nk] = d[i], vi[nk] = vi[i], vo[nk] = vo[i], io[nk] = io[i], ++nk;
    g_mcIpkMean = 0;
    for (int i = 0; i < nk; ++i) g_mcIpkMean += coilfit::ipkDcm(d[i], vi[i], vo[i], io[i]) / (float) nk;
    res = mcFit(d, vi, vo, io, nk, fsw, zsig);
    if (std::fabs(z1 - z0) > coilfit::kZeroTol) printf("  measure: zero DISTURBED %+.3f/%+.3f\n", z0, z1);
    mppt.setExplicitOvLimit(ov0);
    if (mppt.limits.reverse_current_paranoia) mppt.bflow.enable(false);
    secs = tNow() - t0;
    return res;
}

// mc-boost-dark: `measure-coil l0` at night on fry in boost (battery 26.6 V on the LV input, a dark PV string as a diode
//   on the HV output, HV 6.85 V before the start); -lowf: at 21.5 kHz (the `f=21500` pass, booted at that
//   frequency: no runtime change without MCPWM). SIM_MC_DIODE=i0,vt: the string (5 substrings, knee ~70 V: 7.3e-9,3.7),
//   SIM_MC_L=50e-6 with SIM_CONF=coil.conf:L0=50e-6: that board's coil, SIM_MC_VINOFS=a,b: Vin reads
//   a + b Iin [V, V/A] low. SIM_MC_K: plant DC bias
//   L = L0 (1 - k Ipk) (1/A, default 0.004: a sendust core, inside the on-time fit's slope tolerance; at 0.01 the fit
//   takes the slope for an on-time offset of about -130 ns and L comes out ~4 % low), SIM_MC_L: plant L0 (default 80e-6).
//   -ton: plant LS on-time +100 ns (t_on_ofs), k 0: the estimate vs the plant's offset, corrected vs uncorrected L. Per pass: L vs the plant's L at the fitted
//   rows' mean Ipk within 3 %, the slope vs -k (printed), Vout below 80 V, Iin below 8 A, no protection trip.
// mc-boost-open: the output unloaded (string disconnected): the trip stops it at 80 V.
// All start from the night state: C_in and the string at 6.85 V, the battery behind its open backflow switch (in_bf), whose
// gate pump runs only while the converter switches. mc-boost-bfopen: that switch stuck open, the start must abort.
static float g_mcVinOfs[2] = {0, 0};
static int runMcBoost(const std::string &name) {
    int f = 0;
    auto chk = [&](bool c, const char *w) {
        printf("== %s %s: %s\n", name.c_str(), c ? "ok" : "FAIL", w);
        f += !c;
    };
    const float k = getenv("SIM_MC_K") ? (float) atof(getenv("SIM_MC_K")) : 0.004f;
    g_vconv.setLBias(k);
    const bool open = name == "mc-boost-open";
    if (open) g_vconv.setOutDiode(0, 0);
    if (const char *e = getenv("SIM_MC_DIODE")) {
        float i0 = 0, vt = 0;
        sscanf(e, "%f,%f", &i0, &vt);
        g_vconv.setOutDiode(i0, vt);
    }
    if (const char *e = getenv("SIM_MC_VINOFS")) {
        sscanf(e, "%f,%f", &g_mcVinOfs[0], &g_mcVinOfs[1]);
        g_simSenseHook = [](uint8_t ch, float x) { return ch ? x : x - g_mcVinOfs[0] - g_mcVinOfs[1] * g_vconv.getIinAvg(); };
    }
    g_vconv.setVin(6.85f), g_vconv.setVout(6.85f); // night: C_in and the string at 6.85 V, the backflow switch open
    run(5, 0);
    printf("== %s: plant L0 %.1f uH, k %.1f %%/A, Vin %.2f Vout %.2f, fsw %.0f Hz, %s\n", name.c_str(), g_vconv.getL() * 1e6f,
           k * 100, g_vconv.getVin(), g_vconv.getVout(), converter.realizedFreqHz(), modeStr());
    const size_t trips0 = g_trips.size();
    double secs = 0;
    std::string stop;
    const auto r = simMeasureL0Boost(48, 1000, secs, stop);
    for (size_t i = trips0; i < g_trips.size(); ++i) printf("==   t=%.3f [%s] %us\n", g_trips[i].t, g_trips[i].who.c_str(), g_trips[i].sec);
    // the fit's L is at its rows' mean Ipk (L-free, ipkDcm()), where the plant has L0 (1 - k Ipk)
    const float lPlant = g_vconv.getL() * (1 - k * g_mcIpkMean);
    printf("== %s: L %.2f uH vs plant %.2f uH at Ipk %.2f A (%+.2f %%), slope %+.2f +-%.2f %%/A vs %+.2f, %d DCM pts, ring %s,"
           " Ipk up to %.2f A (plant %.2f), Vout max %.2f V, Iin max %.2f A, stop: %s, %.0f s\n", name.c_str(), r.L * 1e6f,
           lPlant * 1e6f, g_mcIpkMean, (r.L / lPlant - 1) * 100, r.k * 100, r.kse * 100, -k * 100, r.n,
           r.f > 0 ? "fit" : "none", g_mcIpk, g_mcIpkPlant, g_mcVmax, g_mcImax, stop.c_str(), secs);
    if (name == "mc-boost-bfopen") { // the backflow switch never conducts: the start aborts, nothing reaches the output
        chk(stop == "Vout did not follow", "aborted at the start");
        chk(g_mcVmax < 26.6f && g_mcImax < 2.f, "Vout below Vin, Iin < 2 A");
        chk(g_trips.size() == trips0, "no protection trip");
        chk(secs < 20, "within 20 s");
    } else if (open) { // nothing loads the output: it climbs at the probe duty until the 80 V trip
        chk(g_trips.size() > trips0 || stop == "TRIP", "tripped");
        chk(g_mcVmax < 80.5f, "Vout < 80.5 V");
    } else {
        chk(std::isfinite(r.L) && r.n >= coilfit::kMinFitPts, "the sweep found its DCM points");
        chk(std::fabs(r.L / lPlant - 1) < 0.03f, "L within 3 % of the plant");
        chk(g_mcVmax < 80.f && g_mcImax < 8.f, "Vout < 80 V, Iin < 8 A");
        chk(g_trips.size() == trips0 && stop != "TRIP", "no trip");
        mcTonCheck(name, lPlant, chk);
        if (name == "mc-boost-dark-f20")
            chk(g_mcIpkPlant <= mppt.limits.Iin_max && g_mcIpk <= mppt.limits.Iin_max, "peak coil current within the limit");
    }
    chk(converter.disabled(), "off after the run");
    printf("== SUMMARY %s: %s (%d failures)\n", name.c_str(), f ? "FAILED" : "OK", f);
    return f != 0;
}

// mc-flat: `measure-coil l0` takes over from MPPT at ~150 W in CCM with paranoia on (bf-daily-flat's plant):
//   no protection trip at the take-over (a duty ramp-down at full power trips highD-bflowOpen, whose backoff then
//   holds the converter off under the sweep), the sweep finds its DCM points with the BF closed (no highD-bflowOpen
//   near 2 A, L within 3 %). SIM_MC_RAMP=1 enters with the old ramp-down, SIM_MC_BFOPEN=1 sweeps with the BF open.
// mc-flat-ramp (SIM_MC_RAMP=1): before pwmPerturb() counted the net decrease over the filtered current's lag, that ramp
//   kept the CCM rect window below D = M, the coil current reversed and highD-bflowOpen tripped (field flat 2026-11-24).
static int runMcFlat(const std::string &sc) {
    int f = 0;
    auto chk = [&](bool c, const char *w) {
        printf("== %s %s: %s\n", sc.c_str(), c ? "ok" : "FAIL", w);
        f += !c;
    };
    settle(30);
    for (double dl = tNow() + 120; tNow() < dl && !(g_vconv.getVin() * g_vconv.pvCurrent(g_vconv.getVin()) > 140.f);)
        run(0.5, 0);
    const float v = g_vconv.getVin(), p0 = v * g_vconv.pvCurrent(v);
    printf("== %s: PV %.0f W, %s, %s, paranoia %d\n", sc.c_str(), p0, converter.inDCM() ? "DCM" : "CCM", modeStr(),
           (int) mppt.limits.reverse_current_paranoia);
    // at 22 kHz flat's coil is still in DCM at full power
    const bool lowf = sc == "mc-flat-lowf" || sc == "mc-flat-f20";
    chk(p0 > 130.f && (lowf || !converter.inDCM()) && !converter.disabled(), lowf ? "MPPT at full power" : "MPPT at full power in CCM");
    const size_t trips0 = g_trips.size();
    const double tEnter = tNow();
    double secs = 0;
    g_ioMin = g_ioMin0 = 0, g_ioTrack = true, g_tickHook = ioMinTick;
    const auto r = simMeasureL0(48, 1000, secs);
    g_tickHook = nullptr;
    int entryTrips = 0;
    for (size_t i = trips0; i < g_trips.size(); ++i) entryTrips += g_trips[i].t < tEnter + 2;
    for (size_t i = trips0; i < g_trips.size(); ++i) printf("==   t=%.3f [%s] %us\n", g_trips[i].t, g_trips[i].who.c_str(), g_trips[i].sec);
    printf("== %s: L %.2f uH vs plant %.2f uH, %d DCM pts, %.0f s, fsw %.0f Hz, Ipk up to %.2f A\n", sc.c_str(), r.L * 1e6f,
           g_vconv.getL() * 1e6f, r.n, secs, converter.realizedFreqHz(), g_mcIpk);
    printf("== %s: until the first converter off: min plant Iout %.3f A with an LS window (HS %u, LS %u), %.3f A any\n",
           sc.c_str(), g_ioMin, g_ioMinD, g_ioMinLs, g_ioMin0);
    chk(!entryTrips, "no protection trip at the take-over");
    chk(g_ioMin > -0.05f, "no reverse output current under an LS window at the take-over");
    chk(g_trips.size() == trips0, "no protection trip during the sweep");
    chk(fabsf(r.L / g_vconv.getL() - 1) < 0.03f, "L within 3 % of the plant");
    chk(r.n >= 3 && std::isfinite(r.L), "the sweep found its DCM points");
    mcTonCheck(sc, g_vconv.getL(), chk);
    if (sc == "mc-flat-f20") {
        printf("== %s: plant Ipk up to %.2f A (rows %.2f A), limit %.1f A\n", sc.c_str(), g_mcIpkPl, g_mcIpk, mppt.limits.Iout_max);
        chk(g_mcIpkPl <= mppt.limits.Iout_max && g_mcIpk <= mppt.limits.Iout_max, "peak coil current within the limit");
    }
    run(40, 5.0);
    chk(!converter.disabled() && g_app.opMode.load() == OpMode::Mppt, "MPPT running again");
    printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
    return f != 0;
}

// dcd-*: duty decreases against pwmPerturb()'s decrease rule from MPPT in CCM, paranoia on, flat's (51 uH) or fry's
//   (76 uH) plant and sensor filters.
//   dcd-stop-*: manual ramp (pwmMax/512 per sample) to 30 and 60 counts below the zero-current duty Vout/Voc, then hold:
//     no reverse output current, no protection trip, the duty held. dcd-slow-*: the same steps 3 samples apart.
//   dcd-ccm-*: ~350 W deep CCM: 300 s of P&O, then a thermal derate that retreats the power limit from 110 % to 40 % of
//     the PV power over 120 s: the rule never drops the LS to its minimum, no trip.
static int runDcd(const std::string &sc) {
    int f = 0;
    auto chk = [&](bool c, const char *w) {
        printf("== %s %s: %s\n", sc.c_str(), c ? "ok" : "FAIL", w);
        f += !c;
    };
    const bool ccm = sc.rfind("dcd-ccm", 0) == 0, slow = sc.rfind("dcd-slow", 0) == 0;
    auto pv = [] { const float v = g_vconv.getVin(); return v * g_vconv.pvCurrent(v); };
    auto atPower = [&](float pMin) {
        for (double dl = tNow() + 150; tNow() < dl && !(pv() > pMin && !converter.inDCM() && !mppt.isSweeping() &&
                                                        g_app.opMode.load() == OpMode::Mppt && !converter.disabled());)
            run(0.5, 0);
        run(5, 0);
        printf("== %s: PV %.0f W, Iout %.2f A, D %u, %s, %s\n", sc.c_str(), pv(), g_vconv.getIoutAvg(),
               converter.getCtrlOnPwmCnt(), converter.inDCM() ? "DCM" : "CCM", modeStr());
        chk(pv() > pMin && !converter.inDCM() && !converter.disabled(), "MPPT at power in CCM");
    };
    settle(30);
    g_tickHook = ioMinTick;
    if (!ccm) {
        for (int off: {30, 60}) {
            atPower(130);
            const uint16_t pwmMax = converter.pwmMaxDriver(), d0 = converter.getCtrlOnPwmCnt();
            const int dz = (int) std::lround(g_vconv.getVout() / g_vconv.getVoc() * pwmMax);
            const auto target = (uint16_t) (dz - off);
            const size_t trips0 = g_trips.size();
            const int ls0 = g_lsMinN;
            g_ioMin = g_ioMin0 = 0, g_ioTrack = true;
            const double t0 = tNow();
            if (slow) { // the same steps 3 samples apart: below the rule's rate (pwmMax/50 per filter lag)
                if (!simPsuWait(mppt.requestPsuManual(d0, -1))) printf("  manual: RT transition timed out\n");
                const double ts = 3.0 / atof(getenv("SIM_ADC_HZ"));
                for (int d = d0; d > target && !converter.disabled() && tNow() < t0 + 5;) {
                    d = std::max<int>(target, d - converter.pwmMaxDriver() / 512);
                    mppt.setManualTarget((uint16_t) d);
                    run(ts, 0);
                }
            } else if (!simPsuWait(mppt.requestPsuManual(target, -1))) printf("  manual: RT transition timed out\n");
            while (converter.getCtrlOnPwmCnt() != target && !converter.disabled() && tNow() < t0 + 5) run(0.001, 0);
            const double tRamp = tNow() - t0;
            run(5, 0);
            printf("== %s: D %u -> %u (zero-current duty %d %+d) in %.0f ms, LS to min %d x, min plant Iout %.3f A with an LS"
                   " window (HS %u, LS %u), %.3f A any, now Vin %.1f V Iout %.2f A %s, %zu trips\n", sc.c_str(), d0, target, dz,
                   -off, tRamp * 1e3, g_lsMinN - ls0, g_ioMin, g_ioMinD, g_ioMinLs, g_ioMin0, g_vconv.getVin(), g_vconv.getIoutAvg(), converter.inDCM() ? "DCM" : "CCM",
                   g_trips.size() - trips0);
            for (size_t i = trips0; i < g_trips.size(); ++i)
                printf("==   t=%.3f [%s] %us\n", g_trips[i].t, g_trips[i].who.c_str(), g_trips[i].sec);
            chk(g_trips.size() == trips0, off == 30 ? "-30: no protection trip" : "-60: no protection trip");
            chk(g_ioMin > -0.05f, off == 30 ? "-30: no reverse output current under an LS window"
                                            : "-60: no reverse output current under an LS window");
            chk(!converter.disabled() && converter.getCtrlOnPwmCnt() == target,
                off == 30 ? "-30: the duty held" : "-60: the duty held");
            g_ioTrack = false;
            if (!simPsuWait(mppt.requestPsuSweep())) printf("  MPPT restore timed out\n");
        }
    } else {
        atPower(250);
        const size_t trips0 = g_trips.size();
        const int ls0 = g_lsMinN;
        const float p0 = pv(), span = mppt.limits.Temp_max - mppt.limits.Temp_derate;
        auto tempFor = [&](float p) { return mppt.limits.Temp_max - span * p / mppt.limits.P_max; };
        g_simChipTemp = tempFor(1.3f * p0); // the chip sensor's EWMA (40 reads, 3 s) settles while P&O runs
        run(300, 0);
        const int lsA = g_lsMinN - ls0;
        const uint16_t dA = converter.getCtrlOnPwmCnt();
        for (int i = 0; i <= 1200; ++i) {
            g_simChipTemp = tempFor(p0 * (1.1f - 0.7f * (float) i / 1200));
            run(0.1, 0);
        }
        run(150, 0);
        const int lsB = g_lsMinN - ls0 - lsA;
        printf("== %s: P&O 300 s: LS to min %d x; limiter retreat %.0f -> %.0f W (chip %.1f -> %.1f C in 120 s), D %u -> %u,"
               " PV %.0f W, LS to min %d x; %zu trips\n", sc.c_str(), lsA, 1.1f * p0, 0.4f * p0, tempFor(1.1f * p0),
               tempFor(0.4f * p0), dA, converter.getCtrlOnPwmCnt(), pv(), lsB, g_trips.size() - trips0);
        chk(lsA == 0, "P&O never drops the LS to its minimum");
        chk(lsB == 0, "the limiter retreat never drops the LS to its minimum");
        chk(pv() < 0.55f * p0, "the limiter retreated");
        chk(g_trips.size() == trips0, "no protection trip");
        g_simChipTemp = 40;
    }
    g_tickHook = nullptr;
    printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
    return f != 0;
}

// dcd-hv-*: a duty cut at a high step-down ratio, Vin ~80 V (95 V Voc) into 26.8 V, from manual duty held in CCM
//   just above the CCM/DCM boundary (Iout < 1.6 x half the coil ripple), paranoia on, fry's or flat's plant and
//   filters. The cut goes 700 counts down (D ~0.34 -> ~0.17, below Vout/Voc), past the filtered current's lag:
//   dcd-hv-step-*: in one pwmPerturb() step (> pwmMax/50: the large-decrease rule)
//   dcd-hv-train-*: by the manual ramp, pwmMax/512 per sample (each step below pwmMax/50, the sum within the lag
//     crosses it: the cumulative rule)
//   Checks: the LS dropped to its minimum, no protection trip, the coil current at LS turn-off stays above -0.2 A
//   (~0.6 us of LS past the zero crossing; without the drop the CCM window pulls -0.5 .. -1.9 A), the cycle-mean
//   output current above -0.05 A, the LS back at its maximum within 2 s, the duty held.
static int runDcdHv(const std::string &sc) {
    int f = 0;
    auto chk = [&](bool c, const char *w) {
        printf("== %s %s: %s\n", sc.c_str(), c ? "ok" : "FAIL", w);
        f += !c;
    };
    const bool train = sc.rfind("dcd-hv-train", 0) == 0;
    settle(30);
    for (double dl = tNow() + 150; tNow() < dl && !(!converter.inDCM() && !mppt.isSweeping() &&
                                                    g_app.opMode.load() == OpMode::Mppt && !converter.disabled());)
        run(0.5, 0);
    run(5, 0);
    const uint16_t pwmMax = converter.pwmMaxDriver(), d0 = converter.getCtrlOnPwmCnt();
    const float vo = g_vconv.getVout(), vi = g_vconv.getVin(),
                iBnd = vo * (1 - vo / vi) / (2 * g_vconv.getL() * converter.realizedFreqHz());
    printf("== %s: Vin %.1f V, Vout %.2f V, Iout %.2f A (CCM boundary %.2f A), D %u, %s, %s\n", sc.c_str(), vi, vo,
           g_vconv.getIoutAvg(), iBnd, d0, converter.inDCM() ? "DCM" : "CCM", modeStr());
    if (!simPsuWait(mppt.requestPsuManual(d0, -1))) printf("  manual: RT transition timed out\n");
    for (double dl = tNow() + 3; tNow() < dl && converter.getRectOnPwmCnt() + 2 < converter.getRectOnPwmMax();) run(0.01, 0);
    run(0.5, 0);
    chk(!converter.inDCM() && !converter.disabled() && g_vconv.getIoutAvg() < 1.6f * iBnd &&
        converter.getRectOnPwmCnt() + 2 >= pwmMax - converter.getCtrlOnPwmCnt() - 1,
        "manual duty in CCM near the boundary, LS at the CCM window");
    const auto d1 = (uint16_t) (converter.getCtrlOnPwmCnt() - 700);
    const size_t trips0 = g_trips.size();
    const int ls0 = g_lsMinN;
    float ilMin, ioMin;
    g_vconv.takeIMin(ilMin, ioMin);
    const double t0 = tNow();
    mppt.setManualTarget(d1);
    if (!train) converter.pwmPerturb((int16_t) (d1 - converter.getCtrlOnPwmCnt()));
    double tFull = -1;
    while (tNow() < t0 + 5) {
        run(0.001, 0);
        if (tFull < 0 && converter.getCtrlOnPwmCnt() == d1 &&
            converter.getRectOnPwmCnt() + 2 >= converter.getRectOnPwmMax())
            tFull = tNow() - t0;
    }
    g_vconv.takeIMin(ilMin, ioMin);
    char tFullS[16] = "never";
    if (tFull >= 0) snprintf(tFullS, sizeof tFullS, "%.0f ms", tFull * 1e3);
    printf("== %s: D %u -> %u (%s; zero-current duty %d at Voc), LS to min %d x, back at its max after %s; coil current"
           " at LS off >= %.2f A, cycle-mean Iout >= %.3f A; now Vin %.1f V Iout %.2f A LS %u/%u %s, %zu trips\n", sc.c_str(),
           d0, d1, train ? "ramp" : "one step", (int) std::lround(vo / g_vconv.getVoc() * pwmMax), g_lsMinN - ls0,
           tFullS, ilMin, ioMin, g_vconv.getVin(), g_vconv.getIoutAvg(), converter.getRectOnPwmCnt(),
           converter.getRectOnPwmMax(), converter.inDCM() ? "DCM" : "CCM", g_trips.size() - trips0);
    for (size_t i = trips0; i < g_trips.size(); ++i)
        printf("==   t=%.3f [%s] %us\n", g_trips[i].t, g_trips[i].who.c_str(), g_trips[i].sec);
    chk(g_lsMinN > ls0, "the LS dropped to its minimum");
    chk(g_trips.size() == trips0, "no protection trip");
    chk(ilMin > -0.2f, "coil current at LS turn-off above -0.2 A");
    chk(ioMin > -0.05f, "no reverse output current");
    chk(tFull >= 0 && tFull < 2, "the LS back at its maximum within 2 s");
    chk(!converter.disabled() && converter.getCtrlOnPwmCnt() == d1, "the duty held");
    printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
    return f != 0;
}

static int runRingCoil(const std::string &name, bool weak) {
    Wx wx;
    const bool fry = !strcmp(g_ring->name, "fry");
    wx.voc = fry ? 78 : 70, wx.k = 0.86f, wx.vbat = fry ? 26.8f : 26.3f, wx.pClear = weak ? 40 : 150;
    wx.init();
    g_verbose = getenv("SIM_V") != nullptr;
    settle(45);
    const float Lt = g_vconv.getL();
    const char *tags[2] = {"old (10 steps, 3 s, median)", "new (48 steps, 1 s, ring fit)"};
    for (int m = 0; m < 2; ++m) {
        double secs = 0;
        const auto r = simMeasureL0(m ? 48 : 10, m ? 1000 : 3000, secs);
        const float L = m ? r.L : r.Lmed;
        printf("== RING coil %s: L %.2f uH = %+.1f %% vs plant %.2f uH (%d DCM pts, %.0f s); fit: ring %.0f kHz"
               " (plant %.0f), ripple +-%.0f %%, %.1f periods, median of points %.2f uH (%+.1f %%)\n", tags[m], L * 1e6f,
               100 * (L / Lt - 1), Lt * 1e6f, r.n, secs, r.f * 1e-3f, g_vconv.getRingF() * 1e-3f, 100 * r.amp, r.periods,
               r.Lmed * 1e6f, 100 * (r.Lmed / Lt - 1));
        // does MPPT come back on its own?
        const double tr = tNow();
        double tOn = -1;
        while (tNow() - tr < 40) {
            run(0.05, 0);
            if (tOn < 0 && !converter.disabled()) tOn = tNow() - tr;
        }
        const float v = g_vconv.getVin();
        printf("== RING coil restore: converter %s (first on after %.1f s), %s, PV %.1f W\n",
               converter.disabled() ? "OFF" : "running", tOn, modeStr(), v * g_vconv.pvCurrent(v));
    }
    printf("== SUMMARY %s: see RING coil lines\n", name.c_str());
    return 0;
}

// ---- self-tests (vconv.conf bflow_model=1: battery-side BF, Vout/Iout sensed at the battery) ----
static float g_iOfs = 0; // Iout sensor offset [A]
static float ofsHook(uint8_t ch, float x) { return ch == 2 ? x + g_iOfs : x; }
static int g_vinGlitch = 0;
static float glitchHook(uint8_t ch, float x) { if (ch == 0 && g_vinGlitch > 0) { --g_vinGlitch; return 1.f; } return x; }

static int selftestCheck(const std::string &sc, bool c, const char *what) {
    printf("== %s %s: %s\n", sc.c_str(), c ? "ok" : "FAIL", what);
    return !c;
}

// hs-short: HS shorted while MPPT runs, then stopped: latch (gates off, no clamp without hs_clamp), start block,
//   clear, re-latch, recovery.
// hs-short-clamp: hs_clamp=1 and a recent blocks pass, by day: gates off, the clamp pending.
// hs-short-unidir: hs_clamp=1 with a one-direction Iout sensor: ignored at boot, latches, the clamp never arms.
// hs-short-bfclosed: hs_clamp=1, BF stuck closed: at night battery current flows before the ramp, so it never starts.
// hs-night: hs_clamp=1, latched by day, then night: after 60 s below 9 V the LS ramps for 20 ms and is held; the
//   250 ms tick re-applies the hold; a BF stuck closed under it releases it (clamp unsafe, persisted).
// hs-night-abort: as hs-night, a disable() 5 ms into the ramp stops it with all gates off.
// hs-night-dsp: as hs-night with despike=8: the quiet night Vin steps to Voc in one sample, which the despike passes
//   to the EWM ahead of the median; the start must wait for the median (the Vin limiter's view), no no-MPP backoff.
// hs-cal: short present at boot, a 1 A panel (inside the 5 % zero bound): the boot zero must not hide it.
// hs-dawn: 0.5 A Iout offset, night then Voc rising through Vbat: no latch. hs-dawn-nozero: no night first, so it
//   latches (known limit), but never clamps.
// hs-volt: battery disconnected, HS shorted: Vout follows Vin above the OV limit at ~0 A, latched by voltage.
// hs-run: HS shorted while switching: another protection stops it, the idle check latches.
// hs-unplug: the panel shorted 3 times in 6 min, healthy HS: never latched, running. hs-glitch: 3 single 1 V Vin samples: no latch, running.
// hs-run-lsdead: the LS never turns on (no shoot-through).
// hs-run-nodecl: hs-run without bflow_bat: no idle check and no latch; Vin-Vout blocks the restart.
static int runHsShort(const std::string &sc) {
    int f = 0;
    auto chk = [&](bool c, const char *w) { f += selftestCheck(sc, c, w); };
    auto st = [] { return mppt.hsFaultState(); };
    auto runFor = [&](double sec, double *tLatch) {
        const double t0 = tNow();
        while (tNow() - t0 < sec) {
            run(0.01, 0);
            if (tLatch && *tLatch < 0 && st()) *tLatch = tNow() - t0;
        }
    };
    const bool clampConf = sc == "hs-short-clamp" || sc == "hs-short-bfclosed" || sc == "hs-dawn-nozero" || sc == "hs-short-unidir" ||
                           sc == "hs-volt-clamp" || sc.rfind("hs-night", 0) == 0;
    if (clampConf) mppt.bfRecent = true; // as after a passed `selftest bflow`
    chk(mppt.hsIdleCheck() == (sc.find("-nodecl") == std::string::npos), "idle check per board.conf");
    const bool unidir = sc == "hs-short-unidir";
    if (unidir) {
        chk(!mppt.batIBipolar() && g_clampIgnoredLog, "unidirectional sensor: hs_clamp ignored at boot");
        chk(!mppt.hsClampAllowed(true) && !mppt.hsClampAllowed(false), "clamp never allowed");
    }
    if (sc == "hs-short" || unidir || sc == "hs-short-clamp" || sc == "hs-short-bfclosed" || sc.rfind("hs-night", 0) == 0) {
        const bool night = sc.rfind("hs-night", 0) == 0;
        const float isc = g_vconv.getIsc(), voc = g_vconv.getVoc(), k = g_vconv.getPvK();
        if (clampConf) { // a night first: the clamp needs a learnt zero
            g_vconv.setPv(isc * .02f, .3f * g_vconv.getVbat(), k);
            runFor(5, nullptr);
            g_vconv.setPv(isc, voc, k);
            printf("== %s: zero %.3f A after the night\n", sc.c_str(), mppt.hsZero());
        }
        settle(20);
        chk(!converter.disabled() && !st(), "running, no fault");
        g_vconv.hsShort = true;
        if (sc == "hs-short-bfclosed") g_vconv.bfStuck = 2;
        printf("%9.4f >>> HS short%s, converter stopped\n", tNow(), g_vconv.bfStuck ? " + BF stuck closed" : "");
        stopAndBackoff(5);
        const uint16_t en0 = converter.enableCount();
        double tl = -1;
        runFor(8, &tl);
        traceLine("fault ");
        printf("== %s: latched (%c) after %.2f s, Iout zero %.3f A, plant Vin %.1f V, LS count %u/%u\n", sc.c_str(),
               mppt.hsFaultWhy() ?: '-', tl, mppt.hsZero(), g_vconv.getVin(), g_vconv.getPwm().pwmRect, g_vconv.getPwm().pwmMax);
        chk(tl > 0.5 && tl < 5, "latched 0.5 s off + 3 s hold after the stop");
        chk(converter.disabled() && converter.startInhibited(SynchronousConverter::InhibitHs), "start blocked");
        chk(!mppt.bflow.state(), "BF commanded off");
        // by day the clamp waits for the night (pending), gates off either way
        const uint8_t want = sc == "hs-short" || unidir ? MpptController::HsOff : MpptController::HsPending;
        chk(st() == want && !converter.lsClamped() && g_vconv.getPwm().pwmRect == 0 && g_vconv.getVin() > 20.f,
            want == MpptController::HsOff ? "gates off, no clamp" : "clamp pending: gates off, Vin up");
        if (unidir) {
            g_vconv.setPv(isc * .01f, 6.f, k);
            printf("%9.4f >>> night\n", tNow());
            bool ls = false;
            for (int n = 0; n < 9000; ++n) {
                run(0.01, 0);
                ls |= g_vconv.getPwm().pwmRect != 0 || st() == MpptController::HsRamp || st() == MpptController::HsClamped;
            }
            traceLine("night ");
            chk(!ls && st() == MpptController::HsOff && converter.enableCount() == en0, "90 s of night: no LS ramp or clamp");
        } else if (sc == "hs-short" || sc == "hs-short-clamp") {
            runFor(20, nullptr);
            chk(converter.enableCount() == en0 && st() == want, "no start, still latched after 20 s");
            mppt.hsClear();
            runFor(0.5, nullptr);
            chk(!st() && !converter.lsClamped() && converter.startInhibited(SynchronousConverter::InhibitHs),
                "cleared: clamp off, start still blocked");
            tl = -1;
            runFor(6, &tl);
            chk(st() == want, "re-latched, the short is still there");
            if (want == MpptController::HsPending) {
                mppt.bfRecent = false; // the blocks pass expires
                runFor(.5, nullptr);
                chk(st() == MpptController::HsOff && !converter.lsClamped(), "pending dropped to gates off when no longer armed");
            }
            g_vconv.hsShort = false;
            mppt.hsClear();
            printf("%9.4f >>> short removed, cleared\n", tNow());
            run(30, 5.0);
            chk(!st() && !converter.disabled(), "running again");
        } else if (sc == "hs-short-bfclosed") {
            // night: the plant's panel sinks at its 6 V Voc, so the battery discharges through the closed BF and the
            // shorted HS into it (a real dark panel would rather hold Vin at the battery: pending for good)
            g_vconv.setPv(isc * .01f, 6.f, k);
            printf("%9.4f >>> night\n", tNow());
            bool lsOn = false, ramp = false;
            for (int n = 0; n < 12000; ++n) {
                run(0.01, 0);
                lsOn |= g_vconv.getPwm().pwmRect != 0;
                ramp |= st() == MpptController::HsRamp || st() == MpptController::HsClamped;
            }
            traceLine("night ");
            chk(!lsOn && !ramp, "LS never on, no ramp");
            chk(st() == MpptController::HsOff && mppt.hsClampUnsafe.load() && simStoredUnsafe(),
                "battery current at the arming: gates off, clamp unsafe (persisted)");
            chk(converter.enableCount() == en0, "no start");
        } else {
            runFor(30, nullptr);
            chk(st() == MpptController::HsPending, "still pending by day");
            g_vconv.setPv(isc * .01f, 6.f, k);
            printf("%9.4f >>> night\n", tNow());
            double tLow = -1, tRamp = -1, tHold = -1;
            uint16_t ls0 = 0, lsPrev = 0;
            bool mono = true, hsLow = true, everHeld = false;
            int rampSteps = 0;
            // -ov*: an OV alert lands between hsShortRt and the alert clear (pend: by night before the ramp, ov: 5 ms into
            // the ramp, ovhold: in the hold); -race*: a core-0 disable() inside a ramp step's driver writes (race0: the
            // first pulse, race: 5 ms in)
            static bool s_raceArm, s_raceHit;
            static int s_lsAfterRace;
            s_raceArm = sc == "hs-night-race0", s_raceHit = false, s_lsAfterRace = -1;
            g_simPwmStopHook = [](int) {
                if (!s_raceArm || mppt.hsFaultState() != MpptController::HsRamp) return;
                s_raceArm = false, s_raceHit = true;
                printf("%9.4f >>> core 0 disable() inside a ramp step\n", tNow());
                converter.disable();
            };
            g_tickHook = [] {
                if (s_raceHit && s_lsAfterRace < 0) s_lsAfterRace = g_vconv.getPwm().pwmRect;
            };
            auto ovAlert = [] {
                printf("%9.4f >>> OV alert after hsShortRt (%u)\n", tNow(), mppt.hsFaultState());
                g_afterHsHook = [] { g_hwOvTrip = true, g_afterHsHook = nullptr; };
            };
            const bool stopLike = sc == "hs-night-abort" || sc == "hs-night-ov" || sc == "hs-night-ovpend" ||
                                  sc == "hs-night-race" || sc == "hs-night-race0";
            const double t0 = tNow();
            while (tNow() - t0 < 100 && st() != MpptController::HsClamped && st() != MpptController::HsOff) {
                run(0.0002, 0);
                const auto &pw = g_vconv.getPwm();
                if (tLow < 0 && sensors.Vin->ewm.avg.get() < 9.f) tLow = tNow();
                if (sc == "hs-night-ovpend" && tLow > 0 && tNow() - tLow > 30 && st() == MpptController::HsPending &&
                    !g_hwOvTrip && !g_afterHsHook && tRamp < 0)
                    ovAlert(), tRamp = 0;
                if (st() == MpptController::HsRamp) {
                    if (tRamp <= 0) tRamp = tNow(), ls0 = pw.pwmRect;
                    if (pw.pwmRect != lsPrev) ++rampSteps;
                    mono &= pw.pwmRect >= lsPrev;
                    hsLow &= pw.pwmCtrl == 0;
                    lsPrev = pw.pwmRect;
                    if (sc == "hs-night-abort" && tNow() - tRamp > .005) {
                        printf("%9.4f >>> a protection disables the converter mid-ramp\n", tNow());
                        mppt.shutdownDcdc("test", 0);
                        run(0.002, 0);
                        break;
                    }
                    if (sc == "hs-night-ov" && tNow() - tRamp > .005 && !g_afterHsHook && !g_hwOvTrip) ovAlert();
                    if (sc == "hs-night-race" && tNow() - tRamp > .005 && !s_raceHit) s_raceArm = true;
                }
                if (tHold < 0 && st() == MpptController::HsClamped) tHold = tNow();
            }
            everHeld = tHold >= 0;
            if (sc == "hs-night-ovhold" && st() == MpptController::HsClamped) {
                run(0.1, 0);
                ovAlert();
                bool reheld = false;
                for (int n = 0; n < 100; ++n) run(0.01, 0), reheld |= n > 1 && (converter.lsClamped() || g_vconv.getPwm().pwmRect);
                traceLine("ov ");
                chk(st() == MpptController::HsOff && !converter.lsClamped() && g_vconv.getPwm().pwmRect == 0 && !reheld,
                    "OV alert in the hold: released, all gates off, not re-applied");
                g_simPwmStopHook = nullptr, g_tickHook = nullptr, g_afterHsHook = nullptr;
                printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
                return f != 0;
            }
            if (sc == "hs-night-ovpend" || sc == "hs-night-race0") {
                run(1, 0);
                traceLine("night ");
                printf("== %s: Vin<9 V at %.1f s, state %u, held %d, LS after the race %d\n", sc.c_str(), tLow - t0, st(),
                       (int) everHeld, s_lsAfterRace);
                if (sc == "hs-night-race0") chk(s_raceHit && s_lsAfterRace == 0, "the first pulse undone at once");
                chk(st() == MpptController::HsOff && !everHeld && !converter.lsClamped() && g_vconv.getPwm().pwmRect == 0,
                    sc == "hs-night-ovpend" ? "OV alert while pending: gates off, never clamped"
                                            : "disable() during the first pulse: gates off, never clamped");
                g_simPwmStopHook = nullptr, g_tickHook = nullptr, g_afterHsHook = nullptr;
                printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
                return f != 0;
            }
            g_simPwmStopHook = nullptr, g_tickHook = nullptr, g_afterHsHook = nullptr;
            traceLine("night ");
            printf("== %s: Vin<9 V at %.1f s, ramp at %.3f s (%d steps, LS %u -> %u of %u), hold at %.3f s\n", sc.c_str(),
                   tLow - t0, tRamp - t0, rampSteps, ls0, lsPrev, g_vconv.getPwm().pwmMax, tHold - t0);
            chk(tLow > 0 && tRamp - tLow >= 60 && tRamp - tLow < 61, "ramp starts after 60 s below 9 V");
            chk(ls0 > 0 && ls0 < g_vconv.getPwm().pwmMax / 50, "ramp starts with a short LS pulse");
            chk(mono && hsLow, "LS width rises, HS held low");
            if (stopLike) {
                if (sc == "hs-night-race") chk(s_raceHit && s_lsAfterRace == 0, "the step after the disable undone at once");
                chk(st() == MpptController::HsOff && !everHeld && !converter.lsClamped() && g_vconv.getPwm().pwmRect == 0 &&
                    !mppt.hsClampUnsafe.load(), sc == "hs-night-ov" ? "ramp stopped by the OV alert: gates off"
                                                                    : "ramp stopped by the disable: gates off");
                runFor(5, nullptr);
                chk(st() == MpptController::HsOff && g_vconv.getPwm().pwmRect == 0, "stays off");
            } else {
                chk(tHold - tRamp >= .019 && tHold - tRamp < .03 && rampSteps >= 10, "hold after a 20 ms ramp");
                chk(st() == MpptController::HsClamped && converter.lsClamped() &&
                    g_vconv.getPwm().pwmRect == g_vconv.getPwm().pwmMax && g_vconv.getVin() < 1.f, "LS clamped, Vin collapsed");
                bool ramped = false;
                for (int n = 0; n < 300; ++n) {
                    run(0.01, 0);
                    if (n == 100) converter.disable(); // a force-off elsewhere: the 250 ms tick re-applies the hold
                    ramped |= st() != MpptController::HsClamped;
                }
                chk(!ramped && converter.lsClamped() && g_vconv.getPwm().pwmRect == g_vconv.getPwm().pwmMax,
                    "3 s: the hold is re-applied, no new ramp");
                g_vconv.bfStuck = 2;
                printf("%9.4f >>> BF stuck closed under the hold\n", tNow());
                runFor(2, nullptr);
                traceLine("bf ");
                chk(st() == MpptController::HsOff && !converter.lsClamped() && mppt.hsClampUnsafe.load(),
                    "clamp released on battery current, all gates off, clamp unsafe");
                chk(converter.enableCount() == en0, "no start");
                // reboot right after: the urgent write persisted the state, RAM does not survive
                chk(simStoredUnsafe(), "clamp-unsafe written at once");
                mppt.hsClampUnsafe = false;
                selftestLoad();
                mppt.bfRecent = true; // as if a pass were recent: only the unsafe flag may disarm the clamp
                printf("%9.4f >>> reboot (state reloaded)\n", tNow());
                chk(st() == MpptController::HsOff && mppt.hsFaultWhy() == 'P' && mppt.hsClampUnsafe.load() &&
                    !mppt.hsClampAllowed(true), "latch and clamp-unsafe restored, clamp not allowed");
            }
        }
    } else if (sc == "hs-cal") {
        runFor(15, nullptr);
        traceLine("boot ");
        printf("== %s: Iout zero taken %.3f A (calibrated %d), plant Iout %.2f A\n", sc.c_str(), sensors.Iout->calibrationAvg,
               (int) sensors.Iout->calibrationComplete, g_vconv.getIoutSense());
        chk(st() != 0, "latched");
        chk(converter.enableCount() == 0, "never started");
    } else if (sc == "hs-dawn" || sc == "hs-dawn-nozero") {
        g_simSenseHook = ofsHook;
        g_iOfs = .5f;
        const float isc = g_vconv.getIsc(), k = g_vconv.getPvK(), vb = g_vconv.getVbat();
        const bool night = sc == "hs-dawn";
        const float v0 = night ? .3f * vb : vb - .5f, v1 = 1.3f * vb;
        g_vconv.setPv(isc * .05f, v0, k);
        runFor(night ? 30 : 5, nullptr);
        printf("== %s: zero %.3f A after %s\n", sc.c_str(), mppt.hsZero(), night ? "night" : "5 s");
        double tl = -1;
        for (int i = 0; i <= 240; ++i) {
            const float x = std::min(1.f, i / 240.f);
            g_vconv.setPv(isc * (.05f + .3f * x), v0 + x * (v1 - v0), k);
            runFor(.5, &tl);
            if (i % 40 == 0) traceLine("dawn ");
        }
        printf("== %s: latched %s (after %.1f s)\n", sc.c_str(), st() ? "YES" : "no", tl);
        if (night) chk(!st() && !converter.disabled(), "no latch, running");
        else chk(st() == 2 && !converter.lsClamped(), "latched (known limit), no clamp without a learnt zero");
    } else if (sc == "hs-volt" || sc == "hs-volt-clamp") {
        const bool cl = sc == "hs-volt-clamp";
        settle(20);
        g_vconv.setBat(0, 1e9f);
        g_vconv.hsShort = true;
        printf("%9.4f >>> battery disconnected, HS short\n", tNow());
        stopAndBackoff(5);
        double tl = -1;
        runFor(70, &tl);
        traceLine("fault ");
        chk(tl > 60 && tl < 62, "voltage alone latches after 0.5 s off + 60 s");
        printf("== %s: latched (%c) after %.2f s, Vin %.1f Vout %.1f OV %.1f\n", sc.c_str(), mppt.hsFaultWhy() ?: '-', tl,
               sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(), mppt.computeOvThreshold());
        chk(st() == (cl ? MpptController::HsPending : MpptController::HsOff) && mppt.hsFaultWhy() == 'V' &&
            g_vconv.getPwm().pwmRect == 0, cl ? "latched by voltage, gates off, clamp pending" : "latched by voltage");
    } else if (sc == "hs-unplug") {
        // the panel shorted three times (connector pulled into a sinking source): Vout>2Vin each time, healthy HS
        settle(20);
        const float isc = g_vconv.getIsc(), voc = g_vconv.getVoc(), k = g_vconv.getPvK();
        const size_t t0 = g_trips.size();
        for (int n = 0; n < 3; ++n) {
            printf("%9.4f >>> panel unplugged at %.1f W\n", tNow(), sensors.Vout->ewm.avg.get() * sensors.Iout->ewm.avg.get());
            g_vconv.setPv(.001f, .5f, k);
            runFor(1, nullptr);
            g_vconv.setPv(isc, voc, k);
            runFor(120, nullptr);
        }
        printf("== %s trips:", sc.c_str());
        for (size_t q = t0; q < g_trips.size(); ++q) printf(" %s", g_trips[q].who.c_str());
        printf("\n== %s: hs fault %d why %c, start %s\n", sc.c_str(), st(), mppt.hsFaultWhy() ?: '-', mppt.startBlockReason() ?: "free");
        chk(!st(), "healthy HS not latched by 3 panel disconnects");
        chk(!converter.disabled(), "running");
    } else if (sc == "hs-glitch") {
        settle(20);
        g_simSenseHook = glitchHook;
        const size_t t0 = g_trips.size();
        for (int n = 0; n < 3; ++n) {
            printf("%9.4f >>> one Vin sample reads 1 V\n", tNow());
            g_vinGlitch = 1;
            runFor(120, nullptr);
        }
        printf("== %s trips:", sc.c_str());
        for (size_t q = t0; q < g_trips.size(); ++q) printf(" %s", g_trips[q].who.c_str());
        printf("\n== %s: hs fault %d why %c, start %s\n", sc.c_str(), st(), mppt.hsFaultWhy() ?: '-', mppt.startBlockReason() ?: "free");
        chk(!st() && !converter.disabled(), "healthy HS not latched by 3 single-sample Vin glitches, running");
    } else if (sc == "hs-run" || sc == "hs-run-nodecl" || sc == "hs-run-lsdead") {
        settle(20);
        const size_t t0 = g_trips.size();
        const uint16_t en0 = converter.enableCount();
        g_vconv.hsShort = sc == "hs-run-lsdead" ? 2 : 1;
        printf("%9.4f >>> HS short while switching\n", tNow());
        double tOff = -1, tl = -1;
        const double ts = tNow();
        while (tNow() - ts < (sc == "hs-run" ? 10 : 60)) {
            run(0.001, 0);
            if (tOff < 0 && converter.disabled()) tOff = tNow() - ts;
            if (tl < 0 && st()) tl = tNow() - ts;
        }
        traceLine("fault ");
        printf("== %s: off after %.1f ms, latched (%c) after %.2f s, %u starts since, trips:", sc.c_str(), tOff * 1e3,
               mppt.hsFaultWhy() ?: '-', tl, (unsigned) (converter.enableCount() - en0));
        for (size_t k = t0; k < g_trips.size(); ++k) printf(" %s", g_trips[k].who.c_str());
        printf("\n");
        chk(tOff >= 0 && tOff < .2, "stopped within 200 ms");
        // the plant dumps C_in into the battery at the short: an over-current or Vout>2Vin trip comes first
        chk(g_trips.size() > t0, "a protection trip");
        if (sc != "hs-run-nodecl") chk(st() == 2 && mppt.hsFaultWhy() == 'I' && converter.enableCount() == en0, "idle check latched, no restart");
        else chk(!st() && converter.enableCount() == en0 && !strcmp(mppt.startBlockReason() ?: "", "Vin-Vout"),
                 "no idle check: no latch, the shorted HS holds Vin at Vout, so no restart (Vin-Vout)");
    } else {
        printf("unknown scenario %s\n", sc.c_str());
        return 1;
    }
    printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
    return f != 0;
}

// bf-ok, bf-open (stuck open), bf-closed (stuck closed): `selftest bflow` against the plant's BF
// bf-nodecl: refused without board.conf bflow_bat=1
// bf-daily: the daily run starts by itself, passes and is not repeated; bf-daily-fail: BF stuck open, bf-fail
// bf-daily-flat: the daily run takes over from MPPT at ~160 W in CCM (flat 2026-11-21: stiff source, filt_len 20,
//   flat's plant); a duty ramp-down at full power tripped highD-bflowOpen there
//   latched, clear releases it
// bf-uni-ok/-open/-closed: Iout clips at 0 (one-direction amp): the toggle variant runs; -abort: the sun goes
//   mid-toggle (SIM_DROP s after engaging): aborted, reverse current through the on switch stopped within the floor's hold
//   (SIM_G: the PV current factor, Voc kept; SIM_VB: a battery voltage step instead)
// bf-uni-offdrop: BF stuck closed, the sun goes in the first off dwell: the RT y floor stops it, aborted
// bf-uni-cloud: the PV current to 30 % 0.2 s after engaging (a cloud): aborted, never FAIL
// bf-uni-dcm: a 0.5 A panel stays in DCM up to the duty cap: aborted before forced PWM, no reverse current
// bf-uni-slow: lv_i_filt_len=300: the toggle's dwell exceeds 1 s: refused
// bf-uni-soft-ok/-closed: a 1.6 A panel at 100 kHz: the current partly stays while off, Vin rises by Vf / D instead
// bf-uni-stale: no sample for 300 ms (SIM_MUTE s) from the first off command: no fresh reading in time, aborted
// bf-uni-soft-vstep: the Vout reading 0.6 V up (SIM_DV, a sense error, the plant unchanged) in the first off window
//   (from SIM_VAT s): the step in D Vin - Vout is gone, the current stays: aborted
// bf-nobat-ok/-closed: battery removed (divider load only): toggle variant judged on D Vin - Vout per duty bump
// bf-nobat-reconnect: a one-direction sensor, the pack (BMS) reconnects mid-toggle: RT stop, aborted
// bf-nobat-packsense: a pack is there but Vout reads 0 (open divider): the output does not follow, aborted
// bf-nobat-bms: Vout reads 0 but BMS cell data is fresh: refused
// bf-nobat-sawbat: the battery was seen (Vout > 9 V, converter off 2 min) before it went: refused until a reboot
// env: SIM_BFVF the body-diode drop, SIM_BFSTUCK 1/2 the switch stuck open/closed, SIM_UNIDIR a one-direction sensor
// bf-ok-dcm: a 0.5 A panel, bipolar sensor: blocks passes, conducts aborts in DCM at the duty cap
// bf-rec: NVS record decode: a version 2 record converts (tog 0, pass kept), version 3 reads, version 1 drops the pass
// bf-daily-uni: the daily run on a one-direction sensor: toggle pass, verified, clamp not armed
// bf-daily-then-closed: daily pass, then BF stuck closed: blocks FAIL revokes the pass, clamp disarmed
// bf-abort: the daily run loses its sun mid-test: aborted, retried after 30 min, no spin
// bf-flat-ccm: on request from MPPT at ~100 W on flat's plant (flat 2026-11-24): the live point holds CCM by the stay
//   ratio, the conducts ramp leaves DCM only past the live duty. -open: BF stuck open, FAIL after that climb;
//   -grow: the current reading grows 8 A/s during the climb, the RT stops at 5 A, aborted; -vinfall: the PV current to
//   15 % during the climb, aborted on the Vin headroom
static int runBfTest(const std::string &sc) {
    int f = 0;
    auto chk = [&](bool c, const char *w) { f += selftestCheck(sc, c, w); };
    if (sc == "bf-rec") {
        const uint32_t t = (uint32_t) time(nullptr) - 3600;
        uint8_t b[16] = {2, 0, 0, 0, 'P', 'P', 0xa5, 0x5a};
        memcpy(b + 8, &t, 4), memcpy(b + 12, &t, 4);
        simLoadBlob(b, 16);
        selftestLoad();
        chk(simRecTog() == 0 && bfPassAgeS() < 7200 && !mppt.bfFail.load(), "v2: tog 0, pass kept");
        selftestLf((uint32_t) (esp_timer_get_time() / 1000), false);
        chk(simStoredVer() == 3 && simStoredPass() == t, "v2: rewritten as v3");
        b[0] = 3, b[6] = 1;
        simLoadBlob(b, 16);
        selftestLoad();
        chk(simRecTog() == 1 && bfPassAgeS() < 7200, "v3: tog read");
        b[0] = 1, b[2] = 1;
        simLoadBlob(b, 16);
        selftestLoad();
        chk(mppt.bfFail.load() && bfPassAgeS() == UINT32_MAX && simRecTog() == 0, "v1: bf kept, pass dropped");
        mppt.bfFailSet(false);
        const bool bat0 = mppt.bfBat;
        mppt.bfBat = false; // board.conf bflow_bat=0: the test cannot run to clear a stored failure
        simLoadBlob(b, 16);
        selftestLoad();
        selftestLf((uint32_t) (esp_timer_get_time() / 1000), false);
        chk(!mppt.bfFail.load() && !converter.startInhibited(SynchronousConverter::InhibitBf) && !simStoredBf(),
            "bflow_bat=0: stored bf dropped, start free, dropped in the store");
        mppt.bfBat = bat0;
        printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
        return f != 0;
    }
    settle(30);
    // no battery since boot: nothing may have seen the pack yet (the firmware's LF tick would)
    const bool nb = sc.rfind("bf-nobat-", 0) == 0 && sc != "bf-nobat-sawbat";
    if (sc == "bf-nobat-sawbat") { // the pack holds Vout with the converter off
        simPsuWait(mppt.requestPsuManual(0, -1, true, true));
        run(125, 0);
    }
    const char *r = nb ? nullptr : bfTestRefusal();
    printf("== %s: refusal %s\n", sc.c_str(), r ? r : "none");
    if (sc == "bf-nodecl") {
        chk(r && strstr(r, "bflow_bat"), "refused without bflow_bat");
        chk(!bfTestStart(), "not armed");
        printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
        return f != 0;
    }
    auto trace = [] {
        if (getenv("SIM_V") && !converter.disabled())
            printf("%9.4f bf: HS %u LS %u/%u (plant %u/%u) bf %d I %.2f/%.2f Vin %.2f Vout %.2f dcm %d\n", tNow(),
                   converter.getCtrlOnPwmCnt(), converter.getRectOnPwmCnt(), converter.getRectOnPwmMax(),
                   g_vconv.getPwm().pwmRect, g_vconv.getPwm().pwmCtrl, (int) mppt.bflow.state(), sensors.Iout->med3.get(),
                   sensors.Iout->ewm.avg.get(), sensors.Vin->ewm.avg.get(), sensors.Vout->ewm.avg.get(), (int) converter.inDCM());
    };
    static float iMaxBlocks;
    iMaxBlocks = 0;
    auto watch = [] {
        if (mppt.bfTest.mode.load() & MpptController::BfBlocks)
            iMaxBlocks = std::max(iMaxBlocks, sensors.Iout->ewm.avg.get());
    };
    if (sc == "bf-daily-then-closed") {
        static uint32_t lfMs2;
        lfMs2 = 0;
        g_tickHook = [] {
            const uint32_t ms = (uint32_t) (esp_timer_get_time() / 1000);
            if (ms - lfMs2 >= 1000) lfMs2 = ms, selftestLf(ms, false);
        };
        run(200, 20);
        printf("== %s: first run verdicts %c %c recent %d\n", sc.c_str(), bfTestVerdict(0) ?: '-', bfTestVerdict(1) ?: '-',
               (int) mppt.bfRecent.load());
        chk(bfTestVerdict(0) == 'P' && bfTestVerdict(1) == 'P' && mppt.bfRecent.load() && mppt.hsClampAllowed(true),
            "daily pass: clamp armed");
        g_vconv.bfStuck = 2;
        printf("%9.4f >>> BF stuck closed; manual selftest bflow, the next 3 NVS writes fail\n", tNow());
        chk(simStoredPass() && !simStoredBf(), "pass persisted");
        simWriteFail = 3; // blocks F, selftestUrgent, runEnd
        g_tickHook = nullptr; // no selftestLf: only urgent writes
        chk(bfTestStart(), "armed");
        run(120, 20);
        chk(!simWriteFail && !simStoredBf() && simStoredPass(), "the urgent writes tried and failed");
        chk(mppt.bfFail.load() && !mppt.hsClampAllowed(true), "write failed: bf-fail and the disarmed clamp hold in RAM");
        g_tickHook = [] {
            const uint32_t ms = (uint32_t) (esp_timer_get_time() / 1000);
            if (ms - lfMs2 >= 1000) lfMs2 = ms, selftestLf(ms, false);
        };
        run(60, 20);
        chk(simStoredBf() && !simStoredPass(), "retried: bf-fail and the revoked pass persisted");
        printf("== %s: verdicts %c %c, bf-fail %d, recent %d, pass age %u s, clampAllowed(V) %d\n", sc.c_str(),
               bfTestVerdict(0) ?: '-', bfTestVerdict(1) ?: '-', (int) mppt.bfFail.load(), (int) mppt.bfRecent.load(),
               (unsigned) bfPassAgeS(), (int) mppt.hsClampAllowed(true));
        chk(bfTestVerdict(0) == 'F' && !bfTestVerdict(1), "blocks FAIL, conducts phase skipped");
        chk(!mppt.bfRecent.load() && bfPassAgeS() == UINT32_MAX, "a blocks FAIL revokes the verified state");
        mppt.bfRecent = true; // even a stale verified flag must not arm the clamp with bf-fail latched
        chk(!mppt.hsClampAllowed(true), "clamp not allowed");
        g_tickHook = nullptr;
        printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
        return f != 0;
    }
    if (sc.rfind("bf-daily", 0) == 0 || sc == "bf-abort") {
        static int runs;
        static uint32_t lfMs;
        runs = 0, lfMs = 0;
        g_tickHook = [] {
            const uint32_t ms = (uint32_t) (esp_timer_get_time() / 1000);
            const bool b = bfTestBusy();
            if (ms - lfMs >= 1000) lfMs = ms, selftestLf(ms, false);
            if (!b && bfTestBusy()) ++runs;
            if (mppt.bfTest.mode.load() & MpptController::BfBlocks)
                iMaxBlocks = std::max(iMaxBlocks, sensors.Iout->ewm.avg.get());
        };
        if (sc == "bf-abort") {
            const float isc = g_vconv.getIsc(), voc = g_vconv.getVoc();
            // drop the sun once the test has engaged
            while (tNow() < 400 && !(mppt.bfTest.mode.load() & MpptController::BfEngage)) run(0.01, 0);
            g_vconv.setPv(isc * .02f, .9f * g_vconv.getVbat(), g_vconv.getPvK());
            printf("%9.4f >>> sun gone mid-test\n", tNow());
            run(20, 0);
            printf("== %s: verdicts %c %c, runs %d\n", sc.c_str(), bfTestVerdict(0) ?: '-', bfTestVerdict(1) ?: '-', runs);
            chk(runs == 1 && !bfTestBusy() && !mppt.bfFail.load() && (bfTestVerdict(0) == 'A' || bfTestVerdict(1) == 'A'),
                "aborted once, no fail latch");
            g_vconv.setPv(isc, voc, g_vconv.getPvK());
            run(25 * 60, 300);
            chk(runs == 1, "no retry within 25 min");
            run(10 * 60, 300);
            chk(runs == 2, "retried after 30 min");
            chk(bfTestVerdict(0) == 'P' && bfTestVerdict(1) == 'P', "retry passed");
        } else {
            const size_t trips0 = g_trips.size();
            run(sc == "bf-daily" ? 200 : 150, 20);
            printf("== %s: runs %d, verdicts %c %c, bf-fail %d, recent %d, pass age %u s\n", sc.c_str(), runs,
                   bfTestVerdict(0) ?: '-', bfTestVerdict(1) ?: '-', (int) mppt.bfFail.load(), (int) mppt.bfRecent.load(),
                   (unsigned) bfPassAgeS());
            chk(runs == 1 && !bfTestBusy(), "one daily run");
            if (sc == "bf-daily-uni") {
                chk(bfTestVerdict(0) == 'P' && !bfTestVerdict(1) && bfPassAgeS() < 600 && !mppt.bfRecent.load(),
                    "toggle passed, verified, clamp not armed");
                chk(!converter.disabled() && g_app.opMode.load() == OpMode::Mppt, "MPPT running again");
                chk(g_trips.size() == trips0, "no protection trip");
                run(30 * 60, 300);
                chk(runs == 1, "not repeated within 30 min");
            } else if (sc == "bf-daily" || sc == "bf-daily-flat") {
                chk(bfTestVerdict(0) == 'P' && bfTestVerdict(1) == 'P' && mppt.bfRecent.load(), "passed, verified");
                chk(!converter.disabled() && g_app.opMode.load() == OpMode::Mppt, "MPPT running again");
                for (size_t i = trips0; i < g_trips.size(); ++i) printf("==   t=%.3f [%s] %us\n", g_trips[i].t, g_trips[i].who.c_str(), g_trips[i].sec);
                chk(g_trips.size() == trips0, "no protection trip");
                if (sc == "bf-daily") run(30 * 60, 300);
                chk(runs == 1, "not repeated within 30 min");
            } else {
                chk(bfTestVerdict(1) == 'F' && mppt.bfFail.load(), "conducts failed, bf-fail latched");
                const uint16_t en = converter.enableCount();
                run(60, 20);
                chk(converter.disabled() && converter.enableCount() == en &&
                    !strcmp(mppt.startBlockReason() ?: "", "bf-fail"), "start blocked: bf-fail");
                selftestClearBf();
                g_vconv.bfStuck = 0;
                run(30, 10);
                chk(!converter.disabled(), "running after clear");
            }
        }
        g_tickHook = nullptr;
        printf("== %s: blocks phase peak ewm I %.2f A\n", sc.c_str(), iMaxBlocks);
        printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
        return f != 0;
    }
    if (sc.rfind("bf-flat-ccm", 0) == 0) {
        const bool open = sc == "bf-flat-ccm-open", grow = sc == "bf-flat-ccm-grow", vinFall = sc == "bf-flat-ccm-vinfall";
        run(60, 20);
        static uint16_t dLive, dMax;
        static float iSenseMax, iPlantMax, vinMin;
        static bool fell;
        dLive = converter.getCtrlOnPwmCnt(), dMax = 0, iSenseMax = iPlantMax = 0, vinMin = INFINITY, fell = false;
        printf("== %s: live D %u Vin %.2f Vout %.2f Iout %.2f %s\n", sc.c_str(), dLive, sensors.Vin->ewm.avg.get(),
               sensors.Vout->ewm.avg.get(), sensors.Iout->ewm.avg.get(), converter.inDCM() ? "DCM" : "CCM");
        chk(!converter.inDCM() && g_app.opMode.load() == OpMode::Mppt, "live point in CCM");
        g_iOfs = 0;
        if (grow) g_simSenseHook = ofsHook;
        g_tickHook = [] {
            const uint8_t m = mppt.bfTest.mode.load();
            if (!(m & MpptController::BfConducts) || converter.disabled()) return;
            dMax = std::max(dMax, converter.getCtrlOnPwmCnt());
            iSenseMax = std::max(iSenseMax, sensors.Iout->med3.get());
            iPlantMax = std::max(iPlantMax, g_vconv.getIoutSense());
            vinMin = std::min(vinMin, g_vconv.getVin());
        };
        const float isc = g_vconv.getIsc(), voc = g_vconv.getVoc();
        chk(bfTestStart(), "armed");
        const double t0 = tNow();
        double tOver = -1;
        while (bfTestBusy() && tNow() - t0 < 180) {
            run(0.01, 0);
            const bool over = (mppt.bfTest.mode.load() & MpptController::BfConducts) && converter.getCtrlOnPwmCnt() > dLive;
            if (over && tOver < 0) tOver = tNow(), printf("%9.4f >>> conducts search above the live duty %u\n", tNow(), dLive);
            if (grow && tOver >= 0 && !converter.disabled()) g_iOfs += .01f * 8.f; // 8 A/s
            if (vinFall && tOver >= 0 && !fell) fell = true, g_vconv.setPv(isc * .15f, voc, g_vconv.getPvK());
            if (getenv("SIM_V") && (mppt.bfTest.mode.load() & MpptController::BfConducts) && !converter.disabled())
                printf("%9.4f bf: HS %u I %.2f/%.2f Vin %.2f Vout %.2f dcm %d\n", tNow(), converter.getCtrlOnPwmCnt(),
                       sensors.Iout->med3.get(), sensors.Iout->ewm.avg.get(), sensors.Vin->ewm.avg.get(),
                       sensors.Vout->ewm.avg.get(), (int) converter.inDCM());
        }
        g_tickHook = nullptr;
        if (grow) g_simSenseHook = nullptr, g_iOfs = 0;
        if (vinFall) g_vconv.setPv(isc, voc, g_vconv.getPvK());
        run(1.1, 0);
        printf("== %s: done after %.1f s, verdicts %c %c, conducts max D %u (live %u, +%.1f %%), sensed I max %.2f A, plant I max "
               "%.2f A, plant Vin min %.2f V, trips %zu\n", sc.c_str(), tNow() - t0, bfTestVerdict(0) ?: '-',
               bfTestVerdict(1) ?: '-', dMax, dLive, 100.f * ((float) dMax / dLive - 1.f), iSenseMax, iPlantMax, vinMin,
               g_trips.size());
        chk(!bfTestBusy() && !mppt.bfTest.mode.load() && !mppt.bflow.lockOff.load(), "finished, RT state released");
        chk(bfTestVerdict(0) == 'P', "blocks PASS");
        const char want1 = open ? 'F' : grow || vinFall ? 'A' : 'P';
        chk(bfTestVerdict(1) == want1, open ? "conducts FAIL" : want1 == 'A' ? "conducts aborted" : "conducts PASS");
        chk(iPlantMax < 5.5f, "plant forward current below the 6 A trip with margin");
        if (grow) chk(mppt.bfTest.why == 5 && iSenseMax < 5.6f, "the RT stopped the search at the 5 A envelope");
        run(20, 5.0);
        chk(mppt.bfFail.load() == open, open ? "bf-fail latched" : "no bf-fail");
        chk(g_app.opMode.load() == OpMode::Mppt && converter.disabled() == open,
            open ? "MPPT restored, start blocked" : "MPPT restored and running");
        printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
        return f != 0;
    }
    if (sc == "bf-abort-cmd") { // `selftest abort` while the blocks phase is engaged
        chk(bfTestStart(), "armed");
        double t0 = tNow();
        while (tNow() - t0 < 60 && !(mppt.bfTest.mode.load() & MpptController::BfEngage)) run(0.01, 0);
        chk(mppt.bfTest.mode.load() & MpptController::BfEngage, "engaged");
        bfTestAbort();
        printf("%9.4f >>> selftest abort\n", tNow());
        t0 = tNow();
        while (bfTestBusy() && tNow() - t0 < 10) run(0.01, 0);
        printf("== %s: stopped after %.2f s, verdicts %c %c\n", sc.c_str(), tNow() - t0, bfTestVerdict(0) ?: '-',
               bfTestVerdict(1) ?: '-');
        chk(!bfTestBusy() && tNow() - t0 < 5, "stopped within 5 s");
        chk(bfTestVerdict(0) == 'A' && !bfTestVerdict(1) && !mppt.bfFail.load(), "aborted, no verdict, no latch");
        chk(!mppt.bfTest.mode.load() && !mppt.bflow.lockOff.load(), "RT state released");
        g_tickHook = nullptr;
        run(20, 5.0);
        chk(g_app.opMode.load() == OpMode::Mppt && !converter.disabled(), "MPPT restored and running");
        printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
        return f != 0;
    }
    if (sc == "bf-uni-slow") {
        chk(r && strstr(r, "too slow") && !bfTestStart(), "refused: filters too slow");
        printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
        return f != 0;
    }
    if (sc.rfind("bf-uni-", 0) == 0 || sc.rfind("bf-nobat-", 0) == 0) {
        const bool noBat = sc[3] == 'n', ok = sc.size() > 3 && !sc.compare(sc.size() - 3, 3, "-ok"),
                   offDrop = sc == "bf-uni-offdrop", sunDrop = sc == "bf-uni-abort" || sc == "bf-uni-cloud" || offDrop, reconnect = sc == "bf-nobat-reconnect",
                   packSense = sc == "bf-nobat-packsense", stale = sc == "bf-uni-stale", vstep = sc == "bf-uni-soft-vstep",
                   aborts = sunDrop || reconnect || packSense || sc == "bf-uni-dcm" || stale || vstep,
                   uni = !noBat || reconnect || getenv("SIM_UNIDIR");
        chk(mppt.batIBipolar() == !uni, uni ? "sensor unidirectional" : "sensor bipolar");
        static float (*prevHook)(uint8_t, float);
        prevHook = g_simSenseHook;
        if (packSense) { // the pack stays, the Vout divider is open
            chk(simPsuWait(mppt.requestPsuManual(0, -1, true, true)), "stopped");
            g_simSenseHook = [](uint8_t ch, float x) { return ch == 1 ? 0.f : prevHook ? prevHook(ch, x) : x; };
            run(3, 0);
            r = bfTestRefusal();
            printf("== %s: pack %.1f V, Vout reads %.2f V, refusal %s\n", sc.c_str(), g_vconv.getVbat(),
                   sensors.Vout->ewm.avg.get(), r ? r : "none");
        } else if (noBat) { // battery off, both output nodes bled down by the divider
            chk(simPsuWait(mppt.requestPsuManual(0, -1, true, true)), "stopped");
            g_vconv.bfVf = .5f; // body diode at the divider's 0.2 mA
            g_vconv.setBat(0.f, 94e3f);
            run(1, 0);
            g_vconv.setVout(0.f);
            run(3, 0);
            r = bfTestRefusal();
            printf("== %s: no battery, Vout %.2f V, refusal %s\n", sc.c_str(), sensors.Vout->ewm.avg.get(), r ? r : "none");
            if (sc == "bf-nobat-sawbat") {
                printf("== %s: battery seen before it went, refusal %s\n", sc.c_str(), r ? r : "none");
                chk(r && strstr(r, "seen since boot") && !bfTestStart(), "refused: a battery was seen this boot");
                printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
                return f != 0;
            }
            if (sc == "bf-nobat-bms") {
                mppt.charger.batSt.setVcellHigh(3.3f);
                r = bfTestRefusal();
                printf("== %s: BMS data fresh, refusal %s\n", sc.c_str(), r ? r : "none");
                chk(r && strstr(r, "BMS") && !bfTestStart(), "refused: BMS data fresh");
                printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
                return f != 0;
            }
        }
        static float iRevBfOn, vOutMax;
        static double revT0, revS; // reverse current with the switch on: first time, total time
        iRevBfOn = 0, vOutMax = 0, revT0 = -1, revS = 0;
        // the first off window: 1 no samples from the command on, 2 a Vout reading offset after vAt, switch off
        static int dist, offs;
        static bool wasOff;
        static float dVb, vOfs, muteS, vAt;
        static double muteT;
        dist = stale ? 1 : vstep ? 2 : 0, offs = 0, wasOff = false, muteT = 0;
        dVb = getenv("SIM_DV") ? (float) atof(getenv("SIM_DV")) : .6f, vOfs = 0;
        vAt = getenv("SIM_VAT") ? (float) atof(getenv("SIM_VAT")) : 0.f;
        muteS = getenv("SIM_MUTE") ? (float) atof(getenv("SIM_MUTE")) : .3f;
        if (vstep) g_simSenseHook = [](uint8_t ch, float x) { return (prevHook ? prevHook(ch, x) : x) + (ch == 1) * vOfs; };
        g_tickHook = [] {
            static double tPrev = 0;
            const double t = tNow();
            if (dist) {
                const uint8_t m = mppt.bfTest.mode.load();
                const bool eng = (m & MpptController::BfToggle) && (m & MpptController::BfEngage),
                           off = eng && (dist == 1 ? !(m & MpptController::BfOn) : !mppt.bflow.state());
                if (off != wasOff) {
                    if (off && !offs++) muteT = t, g_simAdcMute = dist == 1;
                    if (!off && vOfs != 0) vOfs = 0, printf("%9.4f >>> Vout reading back\n", t);
                    wasOff = off;
                }
                if (dist == 2 && off && offs == 1 && t - muteT >= vAt && vOfs == 0)
                    vOfs = dVb, printf("%9.4f >>> Vout reading %+.2f V\n", t, dVb);
                if (g_simAdcMute && t - muteT >= muteS) g_simAdcMute = false, printf("%9.4f >>> ADC unmute\n", t);
            }
            if (mppt.bfTest.mode.load() && g_vconv.bfConducts()) {
                iRevBfOn = std::min(iRevBfOn, g_vconv.getIoutSense());
                if (g_vconv.getIoutSense() < -.5f) revS += t - tPrev, revT0 = revT0 < 0 ? t : revT0;
                if (getenv("SIM_REV") && g_vconv.getIoutSense() < -.5f)
                    printf("%9.5f rev I %.2f Vs %.2f D %.4f Vin %.2f med3 Vin %.2f Vout %.2f I %.2f dis %d mode %u\n", t,
                           g_vconv.getIoutSense(), g_vconv.getVoutSense(), converter.dutyCtrlEff(), g_vconv.getVin(),
                           sensors.Vin->med3.get(), sensors.Vout->med3.get(), sensors.Iout->med3.get(),
                           (int) converter.disabled(), (unsigned) mppt.bfTest.mode.load());
            }
            tPrev = t;
            vOutMax = std::max(vOutMax, g_vconv.getVout());
        };
        if (getenv("SIM_BFVF")) g_vconv.bfVf = (float) atof(getenv("SIM_BFVF"));
        chk(bfTestStart(), "armed");
        const double t0 = tNow();
        const float isc = g_vconv.getIsc(), voc = g_vconv.getVoc();
        const float vb = g_vconv.getVbat(), rb = g_vconv.getRbat();
        if (sunDrop) { // the sun goes while the switch is toggled
            while (tNow() - t0 < 60 && !(mppt.bfTest.mode.load() & MpptController::BfEngage)) run(0.01, 0);
            const bool cloud = sc == "bf-uni-cloud";
            if (offDrop) // the first off dwell
                while (tNow() - t0 < 60 && (mppt.bfTest.mode.load() & MpptController::BfOn)) run(0.001, 0);
            run(getenv("SIM_DROP") ? atof(getenv("SIM_DROP")) : offDrop ? .02 : cloud ? .2 : .25, 0);
            const char *g = getenv("SIM_G");
            if (getenv("SIM_VB")) g_vconv.setBat(vb + (float) atof(getenv("SIM_VB")), rb);
            else if (g || cloud) g_vconv.setPv(isc * (g ? (float) atof(g) : .3f), voc, g_vconv.getPvK());
            else g_vconv.setPv(isc * .02f, .9f * g_vconv.getVbat(), g_vconv.getPvK());
            printf("%9.4f >>> source change mid-test\n", tNow());
        }
        if (reconnect) { // the pack comes back while the switch is toggled
            while (tNow() - t0 < 60 && bfTestBusy() && !(mppt.bfTest.mode.load() & MpptController::BfOn)) run(0.01, 0);
            run(.3, 0);
            g_vconv.setBat(27.f, .05f);
            printf("%9.4f >>> pack reconnected, Vout %.2f V\n", tNow(), g_vconv.getVout());
        }
        while (bfTestBusy() && tNow() - t0 < 180) {
            run(0.05, 0);
            trace();
        }
        if (sunDrop) g_vconv.setPv(isc, voc, g_vconv.getPvK()), g_vconv.setBat(vb, rb);
        g_tickHook = nullptr, g_simAdcMute = false;
        g_simSenseHook = prevHook;
        run(1.1, 0); // past the UART rate limit
        bfTestPrint();
        printf("== %s: done after %.1f s, verdicts %c %c, trips %zu, plant I min with the BF on %.3f A, Vout max %.2f V\n",
               sc.c_str(), tNow() - t0, bfTestVerdict(0) ?: '-', bfTestVerdict(1) ?: '-', g_trips.size(), iRevBfOn, vOutMax);
        chk(!bfTestBusy(), "finished");
        chk(bfTestVerdict(0) == (aborts ? 'A' : ok ? 'P' : 'F') && !bfTestVerdict(1),
            aborts ? "toggle aborted" : ok ? "toggle PASS" : "toggle FAIL");
        chk(!mppt.bfTest.mode.load() && !mppt.bflow.lockOff.load(), "RT state released");
        printf("== %s: reverse current below -0.5 A with the switch on for %.1f ms (from %.4f)\n", sc.c_str(), revS * 1e3, revT0);
        // a sun loss within one sample reverses at once; the floor must stop it within its hold
        chk(sunDrop || reconnect ? revS < .03 : iRevBfOn > -.3f, "no reverse current through the switch while on");
        const bool good = ok || aborts;
        chk(mppt.bfFail.load() == !good && simStoredBf() == !good, good ? "no bf-fail" : "bf-fail latched and persisted");
        selftestLf((uint32_t) (esp_timer_get_time() / 1000), false);
        chk(ok == (bfPassAgeS() != UINT32_MAX) && !mppt.bfRecent.load(), ok ? "verified, clamp not armed" : "no pass");
        if (!noBat) {
            run(20, 5.0);
            chk(g_app.opMode.load() == OpMode::Mppt && converter.disabled() != good,
                ok ? "MPPT restored and running" : "MPPT restored, start blocked");
        } else chk(g_app.opMode.load() == OpMode::Manual && converter.disabled(), "manual 0 restored");
        printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
        return f != 0;
    }
    chk(bfTestStart(), "armed");
    static auto s_trace = trace;
    static auto s_watch = watch;
    g_tickHook = [] { s_watch(); };
    const double t0 = tNow();
    while (bfTestBusy() && tNow() - t0 < 180) {
        run(0.05, 0);
        s_trace();
    }
    g_tickHook = nullptr;
    printf("== %s: done after %.1f s, verdicts %c %c, trips %zu, blocks phase peak ewm I %.2f A\n", sc.c_str(),
           tNow() - t0, bfTestVerdict(0) ?: '-', bfTestVerdict(1) ?: '-', g_trips.size(), iMaxBlocks);
    // a blocks FAIL skips the conducts phase
    const char want0 = sc == "bf-closed" ? 'F' : 'P',
               want1 = sc == "bf-open" ? 'F' : sc == "bf-closed" ? 0 : sc == "bf-ok-dcm" ? 'A' : 'P';
    chk(!bfTestBusy(), "finished");
    chk(bfTestVerdict(0) == want0, "blocks verdict");
    chk(bfTestVerdict(1) == want1, "conducts verdict");
    chk(!mppt.bfTest.mode.load() && !mppt.bflow.lockOff.load(), "RT state released");
    chk(iMaxBlocks < 5.f, "blocks phase stays clear of the 6 A highI-bflowOpen trip");
    run(20, 5.0);
    const bool ok = sc == "bf-ok" || sc == "bf-ok-dcm";
    chk(mppt.bfFail.load() == !ok, ok ? "no bf-fail" : "bf-fail latched");
    chk(g_app.opMode.load() == OpMode::Mppt && converter.disabled() != ok, ok ? "MPPT restored and running" : "MPPT restored, start blocked");
    printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
    return f != 0;
}

// vout_max=auto: no switching while the pack is unknown; a 26 V pack is detected as 8 cells (cv_eoc 3.6: 28.8 V)
// pack swap while powered: 8 cells charging, pack removed (divider load) at 40 s, a 13.2 V 4-cell pack at 80 s.
// bat-auto-swap: the band exit clears the 8-cell count, no switching until 4 cells are detected, then the
// output never exceeds 4 x 3.6 V x 1.03; bat-swap-explicit (vout_max=29): the count is never touched
static int runBatSwap(const std::string &sc) {
    const bool autoMode = sc == "bat-auto-swap";
    static bool cleared, onCleared;
    static float voutMax4;
    cleared = onCleared = false, voutMax4 = 0;
    g_tickHook = [] {
        if (mppt.charger.params.vbatAuto && !mppt.charger.params.haveVbatMax()) cleared = true;
        if (cleared && !mppt.charger.params.haveVbatMax() && !converter.disabled()) onCleared = true;
        if (mppt.charger.params.n_cells == 4) voutMax4 = std::max(voutMax4, g_vconv.getVout());
    };
    g_vconv.setBat(26.f, 0.05f);
    run(40, 10.0);
    const auto &p = mppt.charger.params;
    int f = 0;
    auto chk = [&](bool ok, const char *what) { printf("== %s %s\n", ok ? "PASS" : "FAIL", what); f += !ok; };
    chk(p.n_cells == 8 && !converter.disabled(), "8 cells, charging");
    cleared = false;
    g_vconv.setBat(0.f, 94e3f);
    printf("%9.4f >>> pack removed\n", tNow());
    run(40, 5.0);
    g_vconv.setBat(13.2f, 0.05f);
    printf("%9.4f >>> 4-cell pack 13.2 V\n", tNow());
    run(60, 5.0);
    if (autoMode) {
        chk(cleared, "8-cell count cleared after the removal");
        chk(!onCleared, "no switching while cleared");
        chk(p.n_cells == 4 && fabsf(p.Vbat_max - 14.4f) < 1e-3f, "re-detected as 4 cells (14.4 V)");
        chk(voutMax4 > 0 && voutMax4 < 4 * 3.6f * 1.03f, "4-cell pack never above 14.83 V");
        printf("== 4-cell plant Vout max %.2f V\n", voutMax4);
    } else {
        chk(!p.vbatAuto && p.n_cells == 8 && p.Vbat_max == 29.f && !cleared, "explicit vout_max untouched");
    }
    traceLine("final ");
    printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
    return f != 0;
}

static int runBatAuto(const std::string &sc) {
    static double tDet, tOnEarly;
    tDet = tOnEarly = -1;
    g_tickHook = [] {
        if (tDet < 0 && mppt.charger.params.haveVbatMax()) tDet = tNow();
        if (tDet < 0 && tOnEarly < 0 && !converter.disabled()) tOnEarly = tNow();
    };
    if (sc == "bat-auto-nobat") g_vconv.setBat(0.f, 94e3f); // output open: the Vout divider (47k + 47k) bleeds C_out
    else if (sc == "bat-auto-gap") g_vconv.setBat(17.f, 0.05f);
    else g_vconv.setBat(26.f, 0.05f); // below the no-BMS float pin 8 x 3.37 V
    run(60, 10.0);
    if (sc != "bat-auto") { // `dc` while pending: manual PWM holds off too
        g_app.opMode = OpMode::Manual;
        mppt.setManualTarget((uint16_t) (0.3f * (float) converter.pwmMaxDriver()));
    }
    run(30, 10.0);
    int f = 0;
    auto chk = [&](bool ok, const char *what) { printf("== %s %s\n", ok ? "PASS" : "FAIL", what); f += !ok; };
    chk(tOnEarly < 0, sc == "bat-auto" ? "no switching before detection" : "no switching, also with a manual duty");
    if (sc == "bat-auto") {
        const auto &p = mppt.charger.params;
        printf("== detected at %.1f s: %u cells, Vbat_max %.2f V, fallback %.2f V\n", tDet, (unsigned) p.n_cells, p.Vbat_max,
               p.Vbat_fallback);
        chk(tDet > 0 && tDet < 30, "detected within 30 s");
        chk(p.n_cells == 8 && fabsf(p.Vbat_max - 28.8f) < 1e-3f && fabsf(p.Vbat_fallback - 26.96f) < 1e-3f, "8 cells");
        chk(!converter.disabled() && sensors.Iout->ewm.avg.get() > 1.f, "MPPT charging after detection");
    } else {
        chk(tDet < 0 && mppt.charger.params.awaitingDetect(), "never detected");
        chk(g_nStarts == 0, "never started");
    }
    traceLine("final ");
    printf("== SUMMARY %s: %s (%d failures)\n", sc.c_str(), f ? "FAILED" : "OK", f);
    return f != 0;
}

// Start refused only by the idle profile's InhibitIdle while Vin rises, with a sweep armed (st=START,1: fry
// 2026-11-18, update() returned before any wake) and without (START,0: the start branch): the RT loop must raise
// the wake, not start, and start once the bit is released.
static int runIdleWake() {
    g_vconv.setPvBlocking(true); // the night's panel would sink the battery through the HS body diode
    settle(20);
    const float isc = g_vconv.getIsc(), voc = g_vconv.getVoc(), k = g_vconv.getPvK();
    int fails = 0;
    auto check = [&](bool c, const char *what, int armed) {
        printf("== idle-wake START,%d %s: %s\n", armed, c ? "ok" : "FAIL", what);
        fails += !c;
    };
    for (int armed = 1; armed >= 0; --armed) {
        stopAndBackoff(1);
        g_vconv.setPv(isc, 0.3f * g_vconv.getVbat(), k);
        g_vconv.setVin(0.3f * g_vconv.getVbat()); // a blocking source keeps C_in charged
        run(10, 0);
        if (armed) mppt.startSweep();
        run(10, 0);
        const char *r = mppt.startBlockReason();
        traceLine("night ");
        check(converter.disabled() && mppt.isSweeping() == (bool) armed && r && !strcmp(r, "Vin-Vout"),
              "disabled, Vin-Vout", armed);
        converter.inhibitStart(true, SynchronousConverter::InhibitIdle);
        converter.takeStartWanted();
        g_vconv.setPv(isc, voc, k);
        run(3, 0);
        traceLine("dawn  ");
        check(mppt.startBlockReason() == MpptController::kIdleBlock, "only the idle bit blocks", armed);
        check(converter.disabled(), "no start while inhibited", armed);
        check(converter.takeStartWanted(), "wake raised", armed);
        const uint16_t en = converter.enableCount();
        converter.inhibitStart(false, SynchronousConverter::InhibitIdle);
        run(6, 0);
        traceLine("start ");
        check(converter.enableCount() != en, "started after release", armed);
    }
    printf("== SUMMARY idle-wake: %s (%d failures)\n", fails ? "FAILED" : "OK", fails);
    return fails != 0;
}

// psu-boost-dark: fry driven backwards at night. topo=boost, the battery (26.6 V, ~1.5 mOhm source) on the LV input,
// the HV output loaded only by its caps (1.2 mF), the Vout divider (207.5 kOhm) and a dark PV string as a diode,
// I = I0 (exp(V / Vt) - 1) with I0 120 uA, Vt = n Ns kT/q = 8.3 V (Ns 120 cells, n ~2.7 lumping the series R):
// 5.5 mA at 32 V, 0.3 A at 65 V, 1.0 A at 75 V, 1.8 A at 80 V. Boots in manual duty 0 (target_duty_cycle=0), then
// `psu` 32, 40, 50, 60, 70, 80 V, SIM_DWELL s each (default 90). Per step: plant and sensed Vout maximum against the
// OV threshold, counted PSU trips, and over the second half of the dwell the mean Vout, battery and string current.
// PASS: no trip, plant Vout within 1 % of the setpoint (a third of the 1.03x OV margin), mean within 1 %.
static int runPsuBoostDark(const std::string &name) {
    g_vconv.setVin(26.6f), g_vconv.setVout(26.0f);
    run(5, 0);
    traceLine("manual ");
    const double dwell = getenv("SIM_DWELL") ? atof(getenv("SIM_DWELL")) : 90;
    int fails = 0;
    for (const float v: {32.f, 40.f, 50.f, 60.f, 70.f, 80.f}) {
        const int trips0 = g_psuTrips;
        if (!simPsuWait(mppt.queuePsuSetpoint(v)) || !(mppt.psuTarget == v)) {
            printf("== STEP %.0f V: request refused\n", v);
            ++fails;
            continue;
        }
        const float ovTh = mppt.computeOvThreshold();
        float vMaxP = 0, vMaxS = 0;
        double sv = 0, si = 0, sd = 0, n = 0;
        uint32_t offTicks = 0;
        g_vconv.takeVoutMax();
        for (const double t0 = tNow(), tEnd = t0 + dwell; tNow() < tEnd;) {
            run(0.002, 0);
            vMaxP = std::max(vMaxP, g_vconv.takeVoutMax());
            vMaxS = std::max(vMaxS, sensors.Vout->last);
            offTicks += converter.disabled();
            if (tNow() - t0 >= dwell / 2)
                sv += g_vconv.getVout(), si += g_vconv.getIinAvg(), sd += g_vconv.outDiodeI(g_vconv.getVout()), ++n;
            if (tNow() - t0 > dwell - 1e-9) break;
        }
        const int trips = g_psuTrips - trips0;
        const float vm = (float) (sv / n);
        const bool ok = trips == 0 && !mppt.isPsuLatched() && vMaxP < 1.01f * v && std::fabs(vm - v) < 0.01f * v &&
                        !converter.disabled();
        printf("== STEP %.0f V: Vout max plant %.2f sensed %.2f (OV %.2f), mean %.2f V, Ibat %.3f A, string %.3f A,"
               " D %u, %d PSU trips%s, off %.1f %% : %s\n", v, vMaxP, vMaxS, ovTh, vm, si / n, sd / n,
               converter.getCtrlOnPwmCnt(), trips, mppt.isPsuLatched() ? " LATCHED" : "", 100.0 * offTicks / (dwell / 0.002),
               ok ? "ok" : "FAIL");
        fails += !ok;
    }
    printf("== SUMMARY %s: %s (%d of 6 steps failed)\n", name.c_str(), fails ? "FAILED" : "OK", fails);
    return fails != 0;
}

// psu-boost-cold: `psu 32` from the night state. The backflow switch between the battery and C_in is open (in_bf),
// C_in and the string sit at 6.85 V, Vin senses the battery. Pass: the ramp never starts below Vin + headroom,
// the output settles at 32 V within 20 s, no PSU trip, no latch
static int runPsuBoostCold(const std::string &name) {
    g_vconv.setVin(6.85f), g_vconv.setVout(6.85f);
    run(5, 0);
    traceLine("manual ");
    const float vset = 32.f, vin = sensors.Vin->ewm.avg.get();
    if (!simPsuWait(mppt.queuePsuSetpoint(vset)) || !(mppt.psuTarget == vset)) {
        printf("== SUMMARY %s: FAILED (request refused)\n", name.c_str());
        return 1;
    }
    float rampMin = INFINITY, vMax = 0, iMax = 0;
    unsigned dMax = 0;
    double tReach = -1, sv = 0, n = 0;
    g_vconv.takeVoutMax();
    for (const double t0 = tNow(); tNow() - t0 < 30;) {
        run(0.002, 0);
        if (!converter.disabled()) rampMin = std::min(rampMin, mppt.getPsuSetpoint()), dMax = std::max(dMax, (unsigned) converter.getCtrlOnPwmCnt());
        vMax = std::max(vMax, g_vconv.takeVoutMax()), iMax = std::max(iMax, g_vconv.getIinAvg());
        if (tReach < 0 && g_vconv.getVout() > 0.99f * vset) tReach = tNow() - t0;
        if (tNow() - t0 >= 20) sv += g_vconv.getVout(), ++n;
    }
    const float vm = (float) (sv / n);
    const bool ok = rampMin >= vin + MpptController::BoostHeadroomV - 0.05f && g_psuTrips == 0 && !mppt.isPsuLatched() &&
                    tReach >= 0 && tReach < 20 && std::fabs(vm - vset) < 0.01f * vset;
    printf("== COLD Vin %.2f V: ramp start %.2f V, D max %u, Vout max %.2f, %.2f V at 32 V after %.2f s, mean %.2f V,"
           " Iin max %.1f A, %d PSU trips%s\n", vin, rampMin, dMax, vMax, 0.99f * vset, tReach, vm, iMax, g_psuTrips,
           mppt.isPsuLatched() ? " LATCHED" : "");
    printf("== SUMMARY %s: %s\n", name.c_str(), ok ? "OK" : "FAILED");
    return !ok;
}

// ---- top of charge on a shared bus (cv-hold-*) ----
// Vout sensor error [V]: idle, both units converting, converting alone; linear in 0..2 A own and 0..5 A sibling
// current, so no step at a threshold
struct VoutErr {
    float idle = 0, both = 0, alone = 0;
    [[nodiscard]] float at(float own, float other) const {
        const float s = std::clamp(other / 5.f, 0.f, 1.f), w = std::clamp(own / 2.f, 0.f, 1.f);
        return idle + (alone + (both - alone) * s - idle) * w;
    }
};

static struct TopBus {
    LfpPack pack;
    BmsFeed bms;
    VoutErr err[2];
    std::optional<BatteryCharger> u2;
    ChargerRtc rtc2{};
    float rBus = 0.02f, pv = 560, tau2 = 0.3f; // cable [Ohm], PV power per unit [W], unit 2 output lag [s]
    float i2 = 0, iload = 0, err1 = 0, vcell = 0, ibat = 0, base = 2; // base: house load [A]
    double ah = 0, wh = 0, tTop = 0; // charge and energy into the pack, time with the cell at the top
    float load[4] = {0, 30, 30, 0}; // W, on s, off s, from t
    float loadW = -1; // >= 0: load [W] set per step instead of load[]
    bool bal = true; // balance run from the start
    double gap[4] = {-1, 0, -1, 0}; // t, len (x2)
    double boot2 = 0, sweep2 = 0, lf2 = 0, last = 0;
    unsigned boots2 = 0;
    enum { DOWN, WAIT, SWEEP, ON } st2 = DOWN;
} *g_top;

static float (*g_topPrevSense)(uint8_t, float) = nullptr;

static float topSense(uint8_t ch, float x) {
    if (g_topPrevSense) x = g_topPrevSense(ch, x);
    return ch == 1 ? x + g_top->err1 : x;
}

template<class C>
static bool stalePaused(const C &c) {
    if constexpr (requires { c.termCond.stalePaused(); }) return c.termCond.stalePaused();
    else return false;
}

// per RT tick: pack, load, BMS frames to both chargers, unit 2, then the battery the plant sees (Thevenin of the pack,
// unit 2 and the load)
static void topTick() {
    auto &T = *g_top;
    const double t = tNow();
    const float dt = (float) (t - T.last);
    if (dt <= 0) return;
    T.last = t;
    const float R = T.rBus + LfpPack::N * T.pack.r0, vbus = g_vconv.getVout(), i1 = g_vconv.getIoutAvg();
    T.ibat = (vbus - T.pack.emf()) / R;
    T.ah += T.ibat * dt / 3600.f, T.wh += T.ibat * (T.pack.emf() + T.ibat * LfpPack::N * T.pack.r0) * dt / 3600.f;
    T.pack.step(T.ibat, dt);
    T.vcell = T.pack.vcellHigh(T.ibat);
    if (T.vcell >= mppt.charger.params.cv_eoc - mppt.charger.params.cv_hold_band) T.tTop += dt;
    const float *L = T.load;
    T.iload = T.base + (T.loadW >= 0 ? T.loadW / vbus
              : L[0] > 0 && t >= L[3] && std::fmod(t - L[3], (double) (L[1] + L[2])) < L[1] ? L[0] / vbus : 0.f);
    bool gap = false;
    for (int k = 0; k < 4; k += 2) gap |= T.gap[k] >= 0 && t >= T.gap[k] && t < T.gap[k] + T.gap[k + 1];
    T.bms.cells = T.bms.ibatOn = !gap;
    T.bms.poll(t, T.ibat, T.vcell);
    T.bms.deliver(mppt.charger);
    // unit 2: up BOOT_S after a reset, BMS frames from MQTT_S, starts as loopRTNewData does (pack not full, BMS data
    // decided or the 12 s wait over), ramps to its PV current over SWEEP_S (the sweep), then converts
    constexpr double BOOT_S = 5, MQTT_S = 10, SWEEP_S = 5, BMS_WAIT_S = 12;
    auto &c2 = T.u2;
    if (T.st2 == TopBus::DOWN && t >= T.boot2 + BOOT_S) {
        c2.emplace();
        c2->begin(ConfFile{"/littlefs/conf/charger.conf"});
        c2->attachRtc(&T.rtc2);
        c2->beginMqtt(ConfFile{{"cell_voltages_max_topic", "bms"}});
        if (!T.boots2++ && T.bal) c2->balanceNow();
        T.st2 = TopBus::WAIT, T.lf2 = t;
    }
    if (c2 && t >= T.boot2 + MQTT_S) T.bms.deliver(*c2);
    T.err1 = T.err[0].at(i1, T.i2);
    const float v2 = vbus + T.err[1].at(T.i2, i1);
    if (T.st2 == TopBus::WAIT) {
        const bool fresh = c2->batSt.haveValidCellVoltage(), full = bool(c2->termCond) && fresh;
        const bool waiting = c2->hasBmsCellSource() && (!c2->terminationDecided() || !fresh);
        if (!full && (!waiting || t >= T.boot2 + BOOT_S + BMS_WAIT_S)) T.st2 = TopBus::SWEEP, T.sweep2 = t;
    }
    if (T.st2 == TopBus::SWEEP && t >= T.sweep2 + SWEEP_S) T.st2 = TopBus::ON;
    float tgt = 0;
    if (T.st2 >= TopBus::SWEEP) {
        float cap = T.pv / vbus;
        if (T.st2 == TopBus::SWEEP) cap *= (float) ((t - T.sweep2) / SWEEP_S);
        tgt = std::clamp(T.i2 + (c2->Vout_max() - v2) / R, 0.f, std::min(cap, c2->Iout_max()));
    }
    T.i2 += (tgt - T.i2) * (1.f - expf(-dt / T.tau2));
    if (c2 && t >= T.lf2) {
        T.lf2 = t + lfPeriod * 1e-6;
        chargerUpdate(*c2, v2, T.i2, T.st2 >= TopBus::SWEEP && (T.i2 > 0.5f || c2->chargeHold()), false,
                      T.st2 == TopBus::SWEEP);
    }
    g_vconv.setBat(T.pack.emf() + R * (T.i2 - T.iload), R);
}

static const char *topState(const BatteryCharger *c) {
    if (!c) return "down";
    return stalePaused(*c) ? "paused" : c->balanceRunning() ? "bal" : c->termCond.cvHold() ? "hold" : (bool) c->termCond
           ? "term" : "bulk";
}

static int runCvHold(const std::string &name, const std::string &sc) {
    if (sc != "cv-hold-pair" && sc != "cv-hold-offset" && sc != "cv-hold-gap" && sc != "cv-hold-load" &&
        sc != "cv-hold-reboot" && sc != "cv-hold-mask" && sc != "cv-hold-day") {
        printf("unknown scenario %s\n", sc.c_str());
        return 1;
    }
    static TopBus top;
    g_top = &top;
    auto &T = top;
    topOfCharge(T.pack);
    if (const char *e = getenv("SIM_BASE_A")) T.base = atof(e);
    const float wBase = T.base * 27.f; // the base load at a nominal bus [W]: event loads below keep their ibat
    T.bms.phaseI = 1.3f, T.bms.phaseC = 5.1f, T.bms.qV = 0.001f;
    double tEnd = 3600, rebootAt = -1;
    if (sc == "cv-hold-offset") tEnd = 5400, T.err[1] = {0, -0.15f, -0.9f};
    if (sc == "cv-hold-gap") T.gap[0] = 1800, T.gap[1] = 200;
    if (sc == "cv-hold-load") T.load[0] = 900, T.load[3] = 1200;
    if (sc == "cv-hold-reboot") rebootAt = 1200;
    // a load takes the PV current (ibat ~1 A): the line crosses low, far below cv_eoc, but no hold starts; it ends
    if (sc == "cv-hold-mask") tEnd = 2400, T.load[0] = std::max(1080 - wBase, 0.f), T.load[1] = 900, T.load[2] = 1e5f;
    const bool day = sc == "cv-hold-day";
    if (day) { // 70 Ah below the fit's ah 0 (2026-11-23 09:00, ~80 % SoC by the BMS)
        topOfCharge(T.pack, -70.f);
        T.bal = false, T.loadW = 0, tEnd = 16000;
    }
    if (const char *e = getenv("SIM_T")) tEnd = atof(e);
    if (const char *e = getenv("SIM_PV_W")) T.pv = atof(e);
    if (const char *e = getenv("SIM_LOAD")) sscanf(e, "%f,%f,%f,%f", T.load, T.load + 1, T.load + 2, T.load + 3);
    if (const char *e = getenv("SIM_BMS_GAP")) sscanf(e, "%lf,%lf,%lf,%lf", T.gap, T.gap + 1, T.gap + 2, T.gap + 3);
    if (const char *e = getenv("SIM_REBOOT_AT")) rebootAt = atof(e);
    if (const char *e = getenv("SIM_BMS")) {
        int lag = 0;
        sscanf(e, "%f,%f,%d", &T.bms.phaseI, &T.bms.phaseC, &lag);
        T.bms.lagI = lag;
    }
    for (const char *e = getenv("SIM_VOUT_ERR"); e && *e;) { // u:idle,both,alone[;u:...]
        int u = 0;
        VoutErr v;
        if (sscanf(e, "%d:%f,%f,%f", &u, &v.idle, &v.both, &v.alone) == 4 && (u == 1 || u == 2)) T.err[u - 1] = v;
        e = strchr(e, ';');
        if (e) ++e;
    }
    const float R = T.rBus + LfpPack::N * T.pack.r0;
    Wx wx; // unit 1's PV: fry's string
    wx.voc = 80, wx.k = 0.86f, wx.pClear = T.pv, wx.vbat = T.pack.emf(), wx.rbat = R;
    wx.init();
    wx.apply(0);
    mppt.charger.beginMqtt(ConfFile{{"cell_voltages_max_topic", "bms"}});
    if (T.bal) mppt.charger.balanceNow();
    g_topPrevSense = g_simSenseHook, g_simSenseHook = topSense;
    T.last = tNow(), g_tickHook = topTick;
    g_verbose = getenv("SIM_V") != nullptr;
    const double trEvery = getenv("SIM_TR") ? atof(getenv("SIM_TR")) : 60;
    const auto &P = mppt.charger.params;
    const float eoc = P.cv_eoc, ceil = P.cv_ceiling;
    printf("# %s: %u cells, unit PV %.0f W, R %.0f mOhm, cv_eoc %.3f, ceiling %.3f, %.0f s; Vout error unit 1 %.2f/%.2f/%.2f,"
           " unit 2 %.2f/%.2f/%.2f V; base load %.1f A, load %.0f W %.0f/%.0f s from %.0f s; BMS gaps %.0f+%.0f %.0f+%.0f s;"
           " reboot at %.0f s\n", name.c_str(), (unsigned) P.n_cells, T.pv, R * 1e3f, eoc, ceil, tEnd, T.err[0].idle,
           T.err[0].both, T.err[0].alone, T.err[1].idle, T.err[1].both, T.err[1].alone, T.base, T.load[0], T.load[1],
           T.load[2], T.load[3], T.gap[0], T.gap[1], T.gap[2], T.gap[3], rebootAt);
    double holdAt = -1, gapEnd = T.gap[0] >= 0 ? T.gap[0] + T.gap[1] : -1, resumeAt = -1;
    float peak = 0, entry = 0, lo = 9, hi = 0;
    bool wasTerm[2] = {}, paused[2] = {};
    std::vector<std::string> latches;
    unsigned nCeil = 0;
    // cv-hold-day: the day's clouds (start, s, light) and loads (start, s, W) until the cell reaches the band
    struct Ev { double t, len; float v; };
    static constexpr Ev clouds[] = {{500, 120, .15f}, {1300, 300, .10f}, {2100, 60, .20f}, {2900, 180, .12f},
                                    {3700, 240, .18f}, {4600, 90, .10f}, {5400, 150, .15f}, {6300, 270, .12f}};
    static constexpr Ev loads[] = {{800, 120, 900}, {1800, 60, 900}, {2400, 300, 900}, {3300, 90, 900},
                                   {4200, 180, 900}, {5000, 60, 900}, {5900, 240, 900}};
    const float pv0 = T.pv, band = eoc - P.cv_hold_band;
    double tGap = -1, tE1 = -1, tE2 = -1, endAt = -1, firstHold = -1;
    bool wasHold[2] = {}, early = false;
    float peakHold = 0;
    std::vector<std::string> holds;
    if (day) wx.cloud = {{0.f, 1.f}};
    for (double nextTr = 0; tNow() < tEnd && (endAt < 0 || tNow() < endAt);) {
        const double tb = tNow();
        if (day) {
            const float aHi = (float) T.pack.ah[0];
            if (tGap < 0 && aHi >= -62.f) {
                tGap = tb, T.gap[2] = tb, T.gap[3] = 200;
                printf("%9.2f >>> BMS gap 200 s (cell %.3f, ah %.1f)\n", tb, T.vcell, aHi);
            }
            if (tE1 < 0 && aHi >= -56.f) { // the field case: cloud + load (ibat ~1 A), unit 2 restarts in it
                tE1 = tb, rebootAt = tb + 30;
                printf("%9.2f >>> cloud 20 %% + 180 W for 300 s, unit 2 restarts at +30 s (cell %.3f, ah %.1f)\n", tb,
                       T.vcell, aHi);
            }
            if (tE1 >= 0 && tE2 < 0 && tb >= tE1 + 300 && T.bms.vcell >= eoc - 0.025f) { // in the approach band
                tE2 = tb, rebootAt = tb;
                printf("%9.2f >>> BMS cell %.3f in the approach band: unit 2 restarts, clear sun from here\n", tb,
                       T.bms.vcell);
            }
            float g = 1, w = 0;
            auto edge = [&](const Ev &e) { // 15 s ramps
                return (float) std::clamp(std::min(tb - e.t, e.t + e.len - tb) / 15.0, 0.0, 1.0);
            };
            if (tE2 < 0) {
                for (auto &c: clouds) g = std::min(g, 1 - (1 - c.v) * edge(c));
                for (auto &l: loads) if (tb >= l.t && tb < l.t + l.len) w = l.v;
            }
            if (tE1 >= 0 && tb < tE1 + 300) g = 1 - 0.8f * edge({tE1, 300, 0}), w = std::max(180 - wBase, 0.f);
            wx.cloud[0].second = g, T.pv = pv0 * g, T.loadW = w;
        }
        if (rebootAt >= 0 && tb >= rebootAt) {
            printf("%9.2f >>> unit 2 reboots (cell %.3f)\n", tb, T.vcell);
            T.u2.reset(), T.st2 = TopBus::DOWN, T.boot2 = tb, T.i2 = 0, wasTerm[1] = wasHold[1] = false, rebootAt = -1;
        }
        wx.apply(tb);
        run(0.02, 0);
        const BatteryCharger *u[2] = {&mppt.charger, T.u2 ? &*T.u2 : nullptr};
        for (int k = 0; k < 2; ++k) {
            if (!u[k]) continue;
            const bool hold = u[k]->termCond.cvHold(), term = (bool) u[k]->termCond;
            const float vh = u[k]->batSt.vcell_high;
            if (hold && !wasHold[k]) {
                const bool low = vh < band;
                early |= low;
                if (firstHold < 0) firstHold = tb;
                char b[96];
                snprintf(b, sizeof b, "unit %d at %.0f s, BMS cell %.3f, ibat %.1f%s", k + 1, tb, vh, T.ibat,
                         low ? " (below the band)" : "");
                holds.emplace_back(b);
                printf("%9.2f >>> hold %s\n", tb, b);
            }
            wasHold[k] = hold;
            if (term && !wasTerm[k] && vh < band && vh < ceil) early = true;
            if (term && !wasTerm[k]) {
                const bool c = u[k]->batSt.vcell_high >= ceil;
                nCeil += c;
                char b[96];
                snprintf(b, sizeof b, "unit %d at %.0f s, BMS cell %.3f%s", k + 1, tb, u[k]->batSt.vcell_high,
                         c ? " (ceiling)" : "");
                latches.emplace_back(b);
                printf("%9.2f >>> latch %s\n", tb, b);
            }
            wasTerm[k] = term;
            if (gapEnd >= 0 && tb >= T.gap[0] && tb < gapEnd + 30) paused[k] |= stalePaused(*u[k]);
        }
        if (holdAt < 0 && u[1] && u[0]->termCond.cvHold() && u[1]->termCond.cvHold()) holdAt = tb;
        if (firstHold >= 0) peakHold = std::max(peakHold, T.vcell);
        if (day && endAt < 0 && u[1] && (bool) u[0]->termCond && (bool) u[1]->termCond) endAt = tb + 300;
        if (gapEnd >= 0 && resumeAt < 0 && tb >= gapEnd && u[1] && u[0]->balanceRunning() && u[1]->balanceRunning())
            resumeAt = tb;
        peak = std::max(peak, T.vcell);
        if (holdAt < 0 || tb < holdAt + 300) entry = peak;
        else lo = std::min(lo, T.vcell), hi = std::max(hi, T.vcell);
        if (tb >= nextTr) {
            printf("%8.0f cell %.3f ibat %6.1f bus %6.3f load %4.1f | 1: %5.1f A lim %5.1f Vmax %6.3f %-6s %-11s | 2: %5.1f A"
                   " lim %5.1f Vmax %6.3f %s\n", tb, T.vcell, T.ibat, g_vconv.getVout(), T.iload, g_vconv.getIoutAvg(),
                   u[0]->Iout_max(), u[0]->Vout_max(), topState(u[0]), modeStr(), T.i2, u[1] ? u[1]->Iout_max() : 0.f,
                   u[1] ? u[1]->Vout_max() : 0.f, topState(u[1]));
            nextTr += trEvery;
        }
    }
    g_tickHook = nullptr, g_simSenseHook = g_topPrevSense;
    std::string ls;
    for (auto &l: latches) ls += (ls.empty() ? "" : "; ") + l;
    printf("\n== CVHOLD %s: both in the hold at %.0f s, cell peak %.3f to hold + 300 s, then %.3f..%.3f, run peak %.3f"
           " (cv_eoc %.3f, ceiling %.3f); %zu latches, %u at the ceiling [%s]\n", name.c_str(), holdAt, entry, lo, hi,
           peak, eoc, ceil, latches.size(), nCeil, ls.c_str());
    bool ok = true;
    std::string why;
    auto need = [&](bool c, const char *w) { if (!c) ok = false, why += why.empty() ? w : std::string(", ") + w; };
    need(nCeil == 0, "ceiling latch");
    need(peak < ceil, "cell at the ceiling");
    if (sc == "cv-hold-pair") {
        need(holdAt >= 0, "no hold");
        need(entry <= eoc + 0.015f, "entry peak > cv_eoc + 15 mV");
        need(lo >= eoc - 0.010f && hi <= eoc + 0.010f, "not within +-10 mV");
        need(latches.empty(), "latched");
    }
    if (sc == "cv-hold-load" || sc == "cv-hold-mask") need(holdAt >= 0 && peak <= eoc + 0.015f, "peak > cv_eoc + 15 mV");
    if (sc == "cv-hold-mask") need(holdAt >= T.load[1], "hold entered below the band");
    if (day) {
        std::string hs;
        for (auto &h: holds) hs += (hs.empty() ? "" : "; ") + h;
        const BatteryCharger *u[2] = {&mppt.charger, T.u2 ? &*T.u2 : nullptr};
        bool fin = u[1] != nullptr;
        for (auto *c: u) fin = fin && ((bool) c->termCond || (c->termCond.cvHold() && c->termCond.cvHoldS() <= P.cv_hold_max_s));
        printf("== CVDAY %s: events gap %.0f s, cloud+load %.0f s, band restart %.0f s; first hold %.0f s, cell peak %.4f"
               " after it (run %.4f); %.1f Ah / %.0f Wh into the pack, %.0f s with the cell >= %.3f, end %s/%s at %.0f s;"
               " holds [%s]\n", name.c_str(), tGap, tE1, tE2, firstHold, peakHold, peak, T.ah, T.wh, T.tTop, band,
               topState(u[0]), topState(u[1]), tNow(), hs.c_str());
        need(!early, "hold or latch below cv_eoc - cv_hold_band");
        need(firstHold >= 0, "no hold");
        need(peakHold <= eoc + 0.015f, "peak > cv_eoc + 15 mV after the hold");
        need(fin, "no latch or hold within cv_hold_max_s at the end");
        printf("== CVDAY %s: %s%s%s\n", name.c_str(), ok ? "PASS" : "FAIL", ok ? "" : ": ", why.c_str());
        return ok ? 0 : 1;
    }
    if (gapEnd >= 0) {
        printf("== CVHOLD %s gap %.0f..%.0f s: paused unit 1 %d, unit 2 %d; both running again at %.0f s\n", name.c_str(),
               T.gap[0], gapEnd, (int) paused[0], (int) paused[1], resumeAt);
        need(paused[0] && paused[1], "no stale pause");
        need(resumeAt >= 0 && resumeAt < gapEnd + 60, "no resume within 60 s");
    }
    printf("== CVHOLD %s: %s%s%s\n", name.c_str(), ok ? "PASS" : "FAIL", ok ? "" : ": ", why.c_str());
    return ok ? 0 : 1;
}

// scan-static: probe placement of PeakScan::update() against the exact plant's maxima, no closed loop. Grid: n = 4..6
// substrings (string Voc 80 V, SIM_STATIC_N: one n), last k at s (k2 before them at (1 + s) / 2), module Vmpp/Voc kp,
// Voc now vs at the sweep (heating); SIM_STATIC_VARS=a,b: only these variants; "land": the maximum moved to / the global;
// every reachable maximum as the operating point; share variants (weight w on vs0, wn if m0 == n, whi if Vin/m0 >
// vs0, probe factor lo). One STATIC line per case and variant: per target maximum the best probe in its basin
// (dV, read / true power), the decision (move to basin / stay) against the truth.
static int scanStatic() {
    struct Var {
        const char *nm;
        float w, wn, whi, lo, hi = -1;
        bool hiVs = false;
        float up = -1; // m > m0 probes at up x m x share
    };
    const Var vars[] = {{"base", .5f, -1, -1, .97f}, {"A", .5f, 1, -1, .97f}, {"A2", .5f, -1, 1, .97f},
                        {"B94", .5f, -1, -1, .94f}, {"B92", .5f, -1, -1, .92f}, {"C75", .75f, -1, -1, .97f},
                        {"A+B94", .5f, 1, -1, .94f}, {"r0", 0, -1, -1, .97f}, {"vs", 1, -1, -1, .97f},
                        {"D1", .5f, -1, -1, .97f, 1, true}, {"D2", .5f, -1, -1, .97f, 1, false},
                        {"D3", .5f, -1, -1, .97f, 1.03f, true}, {"D2+A", .5f, 1, -1, .97f, 1, false},
                        {"D4", .5f, -1, -1, .97f, 1.03f, false}, {"D5", .5f, -1, -1, .97f, 1.06f, false},
                        {"D6", .5f, -1, -1, .97f, -1, false, 1.03f}, {"D7", .5f, -1, -1, .97f, -1, false, 1.0f},
                        {"u0.980", .5f, -1, -1, .97f, -1, false, 0.98f}, {"u0.990", .5f, -1, -1, .97f, -1, false, 0.99f},
                        {"u1.005", .5f, -1, -1, .97f, -1, false, 1.005f}, {"u1.010", .5f, -1, -1, .97f, -1, false, 1.01f}, {"u1.015", .5f, -1, -1, .97f, -1, false, 1.015f},
                        {"u1.020", .5f, -1, -1, .97f, -1, false, 1.02f}, {"u1.040", .5f, -1, -1, .97f, -1, false, 1.04f},
                        {"u1.050", .5f, -1, -1, .97f, -1, false, 1.05f}};
    // SIM_STATIC_VARS=base,D6,...: only these variants (all by default)
    const char *only = getenv("SIM_STATIC_VARS");
    auto wanted = [&](const char *nm) {
        if (!only) return true;
        const size_t l = strlen(nm);
        for (const char *q = only; (q = strstr(q, nm)); q += l)
            if ((q == only || q[-1] == ',') && (q[l] == ',' || !q[l])) return true;
        return false;
    };
    const float vocS = 80, vMin = 1.05f * 1.18f * 26.6f, margin = 0.03f;
    const bool learned = !getenv("SIM_STATIC_KDEF"); // kVmp from an unshaded sweep, else the 0.82 default
    // n substrings in a string of Voc 80 V; SIM_STATIC_N=6: fry6 only
    const int nOnly = getenv("SIM_STATIC_N") ? atoi(getenv("SIM_STATIC_N")) : 0;
    for (int n = 4; n <= 6; ++n)
    for (float kp: {0.80f, 0.86f})
        for (float drift: {0.02f, 0.f, -0.04f, -0.08f, -0.12f})
            for (int k = 1; k <= std::min(4, n - 1); ++k)
                for (int k2 = 0; k2 <= (k <= 3 ? 1 : 0); ++k2)
                    for (float sh = 0.3f; sh < 0.96f; sh += 0.05f) {
                        float g[6] = {1, 1, 1, 1, 1, 1};
                        if (nOnly && n != nOnly) continue;
                        for (int i = 0; i < n; ++i) g[i] = i >= n - k ? sh : i >= n - k - k2 ? 0.5f * (1 + sh) : 1;
                        // the sweep: unshaded, cold (kVmp learned there)
                        const float ones[6] = {1, 1, 1, 1, 1, 1};
                        g_vconv.setPvExact(true);
                        g_vconv.setPv(6.f, vocS, kp), g_vconv.setPvString(n, ones, nullptr, 0.45f);
                        Peak pk[6];
                        float valley[6];
                        int np = pvPeaks(pk, valley);
                        float kVmp = 0.82f;
                        if (learned && np >= 1 && pk[np - 1].v / vocS > 0.74f && pk[np - 1].v / vocS < 0.86f) kVmp = pk[np - 1].v / vocS;
                        g_vconv.setPv(6.f, vocS * (1 + drift), kp), g_vconv.setPvString(n, g, nullptr, 0.45f);
                        np = pvPeaks(pk, valley);
                        auto basin = [&](float v) {
                            int b = 0;
                            while (b + 1 < np && v > valley[b]) ++b;
                            return b;
                        };
                        auto P = [](float v) { return v * g_vconv.pvCurrent(v); };
                        const float iEnv = g_vconv.pvCurrent(vMin);
                        int iG = -1;
                        for (int i = 0; i < np; ++i) if (pk[i].v >= vMin && (iG < 0 || pk[i].p > pk[iG].p)) iG = i;
                        for (int op = 0; op < np; ++op) {
                            if (pk[op].v < vMin) continue;
                            const float vin = pk[op].v, pw = pk[op].p, vs0 = kVmp * vocS / n;
                            const int m0 = std::clamp((int) std::lround(vin / vs0), 1, n);
                            for (auto &V: vars) {
                                if (!wanted(V.nm)) continue;
                                const float x = m0 == n && V.wn >= 0 ? V.wn : vin / m0 > vs0 && V.whi >= 0 ? V.whi : V.w;
                                const float shr = (1 - x) * vin / (float) m0 + x * vs0;
                                float best = 0, bestV = 0;
                                char buf[300];
                                int o = 0;
                                float dvB[6], rdB[6];
                                for (int i = 0; i < 6; ++i) dvB[i] = NAN, rdB[i] = 0;
                                for (int m = 1; m <= n; ++m) {
                                    const float v = m == n && V.hi > 0 ? V.hi * (float) n * (V.hiVs ? vs0 : shr)
                                                    : m > m0 && V.up > 0 ? V.up * (float) m * shr
                                                                         : V.lo * (float) m * shr;
                                    if (m == m0 || v < vMin || v > 0.95f * vocS ||
                                        (m < m0 && (float) m * shr * iEnv < pw * (1 + margin)))
                                        continue;
                                    const int b = basin(v);
                                    const float rd = P(v) / pk[b].p;
                                    if (b != op && rd > rdB[b]) rdB[b] = rd, dvB[b] = v / pk[b].v - 1;
                                    if (P(v) > best) best = P(v), bestV = v;
                                }
                                const float g0 = best / pw;
                                const int to = g0 > 1 + margin ? basin(bestV) : op;
                                const float gT = pk[iG].p / pw;
                                const char *cls = to == op ? (gT > 1 + margin ? (rdB[iG] > 0 ? "MISS" : "MISS-unprobed") : "ok")
                                                           : to == iG ? "HIT" : pk[to].p > pw ? "PARTIAL" : "WRONG";
                                for (int b = 0; b < np; ++b)
                                    if (b != op && pk[b].v >= vMin)
                                        o += snprintf(buf + o, sizeof buf - o, " %.1fV/%.0fW:%s", pk[b].v, pk[b].p,
                                                      std::isnan(dvB[b]) ? "none" : "");
                                (void) buf;
                                printf("STATIC n %d kp %.2f drift %+.2f k %d k2 %d s %.2f op %d/%d %.1fV m0 %d gT %.3f var %-6s"
                                       " sh %.2f | G dV %+.1f%% read %.3f | %s land %.1fV x%.4f", n, kp, drift, k, k2, sh, op,
                                       np, vin, m0, gT, V.nm, shr, op == iG ? 0.f : 100 * dvB[iG], op == iG ? 1.f : rdB[iG],
                                       cls, pk[to].v, pk[to].p / pk[iG].p);
                                for (int b = 0; b < np; ++b)
                                    if (b != op && pk[b].v >= vMin)
                                        printf(" [%.1fV %.0fW %s%+.1f%% x%.3f]", pk[b].v, pk[b].p, b == iG ? "G " : "",
                                               100 * dvB[b], rdB[b]);
                                printf("\n");
                            }
                        }
                    }
    return 0;
}

static int simMain(int argc, char **argv) {
    std::string sc = argc > 1 ? argv[1] : "sanity";
    g_simLogHook = logHook;
    if (sc == "scan-static") return scanStatic();
    if (const char *e = getenv("SIM_SCAN_LOW")) PeakScan::probeLow = (float) atof(e);
    if (const char *e = getenv("SIM_SCAN_UP")) PeakScan::probeHigh = (float) atof(e);
    if (const char *e = getenv("SIM_SCAN_HEAT")) PeakScan::heatCut = (float) atof(e);
    if (const char *f = getenv("SIM_FINE")) sscanf(f, "%lf,%lf", &g_fineT0, &g_fineT1);
    if (const char *f = getenv("SIM_TTRACE")) sscanf(f, "%lf,%lf", &g_trkT0, &g_trkT1);
    // suffix -p0: limits.conf reverse_current_paranoia=0 (Vout-OV threshold 1.5x instead of 1.03x Vbat_max)
    const bool p0 = sc.size() > 3 && sc.compare(sc.size() - 3, 3, "-p0") == 0;
    const std::string name = sc;
    if (p0) sc.resize(sc.size() - 3);
    if (sc == "rcal-flat") setenv("SIM_RING", "flat", 0), setenv("SIM_ADC_HZ", "1470", 0);
    if (sc == "mc-flat-ramp") setenv("SIM_MC_RAMP", "1", 1);
    // -ton: a plant on-time offset (buck HS -100 ns, boost LS +100 ns) on a coil without DC bias
    if (sc == "mc-flat-ton") setenv("SIM_MC_TON", "-100e-9", 0);
    if (sc == "mc-boost-dark-ton") setenv("SIM_MC_TON", "100e-9", 0), setenv("SIM_MC_K", "0", 0);
    // -f20: the sweep of `measure-coil l0 f=20000` on a 48.6 uH coil (fsw L0 0.95 = 0.92 ohm, under pwm-freq's 1 ohm).
    // Booted at 20 kHz (no runtime change without MCPWM), so coil.conf L0 is 52.7 uH, the least the boot check takes;
    // the rows' peak-current stop is L-free.
    if (sc == "mc-flat-f20" || sc == "mc-boost-dark-f20") {
        setenv("SIM_MC_L", "48.6e-6", 0), setenv("SIM_PLANT", "flat:L=48.6e-6", 0);
        std::string c = "coil.conf:L0=52.7e-6,board.conf:pwm_freq=20000";
        if (const char *e = getenv("SIM_CONF")) c = c + "," + e;
        setenv("SIM_CONF", c.c_str(), 1);
    }
    const bool dcd = sc.rfind("dcd-", 0) == 0, dcdFry = dcd && sc.size() > 4 && !sc.compare(sc.size() - 4, 4, "-fry");
    if (dcdFry) setenv("SIM_RING", "fry-off", 0), setenv("SIM_ADC_HZ", "513", 0);
    const bool flatTake = sc == "bf-daily-flat" || sc.rfind("mc-flat", 0) == 0 || sc.rfind("bf-flat-ccm", 0) == 0 ||
                          (dcd && !dcdFry);
    if (flatTake) setenv("SIM_RING", "flat-off", 0), setenv("SIM_ADC_HZ", "1469", 0);
    if (sc == "rcal-pv") setenv("SIM_RING", "fry", 0);
    if (sc.rfind("wx-", 0) == 0 && sc.size() > 4 && sc.compare(sc.size() - 4, 4, "-fry") == 0)
        setenv("SIM_RING", "fry-off", 0), setenv("SIM_ADC_HZ", "513", 0);
    if (sc.rfind("wx-", 0) == 0 && sc.size() > 5 && sc.compare(sc.size() - 5, 5, "-flat") == 0)
        setenv("SIM_RING", "flat-off", 0), setenv("SIM_ADC_HZ", "1469", 0);
    const bool psuBoost = sc == "psu-boost-dark" || sc == "psu-boost-cold", mcBoost = sc.rfind("mc-boost-", 0) == 0;
    if (psuBoost || mcBoost) { // fry in boost: MCPWM timing, its ADC rate and filters, INA226 (Vin, Iin) on the LV side, no HV shunt
        std::string c = "converter.conf:topo=boost,tracker.conf:target_duty_cycle=0,coil.conf:L0=80e-6,"
                        "sensor.conf:lv_v_ch=0,sensor.conf:hv_v_ch=1,sensor.conf:lv_i_ch=3,sensor.conf:lv_i_factor=-1,"
                        "sensor.conf:hv_v_filt_len=20,sensor.conf:lv_v_filt_len=20,sensor.conf:lv_i_filt_len=20,"
                        "limits.conf:lv_i_max=32,board.conf:bflow_bat=1";
        if (sc == "mc-boost-dark-lowf") c += ",board.conf:pwm_freq=21500";
        if (const char *e = getenv("SIM_CONF")) c = c + "," + e;
        setenv("SIM_CONF", c.c_str(), 1);
        setenv("SIM_ADC_HZ", "513", 0);
        if (!getenv("SIM_DRV")) setenv("SIM_DRV", "mcpwm", 1);
    }
    const bool cvHold = sc.rfind("cv-hold-", 0) == 0;
    if (sc.rfind("wx-", 0) == 0 || cvHold) setenv("SIM_RING", "fry", 0); // tracker scenarios: a field board's plant
    if (cvHold) { // fry/flat's charger, a balance run
        std::string c = "charger.conf:vout_max=29,charger.conf:vout_max_fallback=26.8,charger.conf:cv_eoc=3.5,"
                        "charger.conf:cv_float=3.35,charger.conf:bat_c=280,charger.conf:ibat_max=40,"
                        "charger.conf:cv_hold_max_s=5400,charger.conf:balance_interval=30,charger.conf:balance_hold_s=7200";
        if (const char *e = getenv("SIM_CONF")) c = c + "," + e;
        setenv("SIM_CONF", c.c_str(), 1);
    }
    if (const char *n = getenv("SIM_NOISE"); n && *n) noiseLoad(n);
    if (const char *f = getenv("SIM_PDUMP")) g_pDump = ::fopen(f, "wb");
    g_liveOffset = sc.rfind("rcal", 0) != 0;
    sc = ringBoardFrom(sc);
    std::vector<std::pair<std::string, std::string>> ov;
    if (sc == "b") ov = {{"pv_isc", "1.6"}};
    if (sc == "c") ov = {{"v_bat", "28.7"}};
    if (sc == "d-noise") ov = {{"noise_vout", "0.03"}, {"vbat_ac_amp", "0.26"}, {"vbat_ac_freq", "100"}};
    if (sc.rfind("shade", 0) == 0) {
        ov = {{"pv_isc", "6.5"}, {"pv_voc", "78"}};
        // shade-cv: battery near the Vout cap with inverter ripple, so the Vout limiter touches now and then
        if (sc == "shade-cv") {
            ov.insert(ov.end(), {{"noise_vout", "0.03"}, {"vbat_ac_amp", "0.26"}, {"vbat_ac_freq", "100"}});
        }
    }
    Wx wx;
    const bool isWx = sc.rfind("wx-", 0) == 0 || sc == "ring-sweep" || sc == "ring-weak";
    if (isWx && !wxScenario(sc, wx)) {
        printf("unknown scenario %s\n", sc.c_str());
        return 1;
    }
    if (sc == "wx-low" || sc == "wx-mid" || sc == "wx-low-slow")
        ov = {{"noise_iout", sc != "wx-mid" ? "0.02" : "0.04"}, {"noise_vout", "0.03"}};
    if (sc == "wx-calfail")
        ov = {{"noise_iout", "0.04"}, {"noise_vout", "0.03"}, {"vbat_ac_amp", "1.3"}, {"vbat_ac_freq", "100"}};
    if (sc == "wx-low-slow") { // filter length 60 on the power sensors (buck_bench, boost_*)
        std::string c = "sensor.conf:lv_i_filt_len=60,sensor.conf:lv_v_filt_len=60";
        if (const char *e = getenv("SIM_CONF")) c = c + "," + e;
        setenv("SIM_CONF", c.c_str(), 1);
    }
    const bool selftest = sc.rfind("hs-", 0) == 0 || sc.rfind("bf-", 0) == 0 || sc.rfind("mc-flat", 0) == 0 || dcd;
    if (selftest) {
        std::string c = sc.size() > 7 && !sc.compare(sc.size() - 7, 7, "-nodecl") ? "" : "board.conf:bflow_bat=1";
        if (sc == "hs-short-clamp" || sc == "hs-short-bfclosed" || sc == "hs-dawn-nozero" || sc == "bf-daily-then-closed" || sc == "hs-short-unidir" ||
            sc == "hs-volt-clamp" || sc.rfind("hs-night", 0) == 0)
            c += ",board.conf:hs_clamp=1";
        if (sc == "hs-night-dsp") c += ",sensor.conf:despike=8";
        if (sc.rfind("dcd-hv-", 0) == 0) c += ",limits.conf:hv_max=110";
        if (flatTake || dcd) // flat's (and fry's) sensor filters
            c += ",sensor.conf:hv_v_filt_len=20,sensor.conf:lv_v_filt_len=20,sensor.conf:lv_i_filt_len=20";
        // the state `measure-coil l0 f=` runs in, near flat's floor (fsw*L0*0.95 > 1 ohm with L0 50 uH)
        if (sc == "mc-flat-lowf") c += ",board.conf:pwm_freq=22000";
        if (const char *e = getenv("SIM_CONF")) c = c + "," + e;
        if (!c.empty()) setenv("SIM_CONF", c.c_str(), 1);
        ov = {{"bflow_model", "1"}};
        if (sc == "bf-open" || sc == "bf-daily-fail" || sc == "bf-uni-open" || sc == "bf-flat-ccm-open") ov.push_back({"bflow_stuck", "1"});
        if (sc == "bf-closed" || sc == "bf-uni-closed" || sc == "bf-uni-offdrop" || sc == "bf-uni-soft-closed" || sc == "bf-nobat-closed")
            ov.push_back({"bflow_stuck", "2"});
        if (sc.rfind("bf-uni-soft", 0) == 0) ov.push_back({"pv_isc", "1.6"}); // the PV, not the duty, sets the current
        if (sc.rfind("bf-uni-", 0) == 0 || sc == "hs-short-unidir" || sc == "bf-daily-uni" || sc == "bf-nobat-reconnect")
            ov.push_back({"i_unidir", "1"});
        if (sc == "bf-uni-dcm" || sc == "bf-ok-dcm") ov.push_back({"pv_isc", "0.5"});
        if (const char *e = getenv("SIM_BFSTUCK")) ov.push_back({"bflow_stuck", e});
        if (getenv("SIM_UNIDIR")) ov.push_back({"i_unidir", "1"});
        if (sc == "bf-uni-slow") c += ",sensor.conf:lv_i_filt_len=300", setenv("SIM_CONF", c.c_str(), 1);
        // 100 kHz: CCM from ~1 A, so the 1.6 A panel engages (DCM aborts)
        if (sc.rfind("bf-uni-soft", 0) == 0) c += ",board.conf:pwm_freq=100000", setenv("SIM_CONF", c.c_str(), 1);
        // the switch's body diode takes 0.7 V of the drive: a bit more sun for the same live point
        if (sc.rfind("bf-flat-ccm", 0) == 0)
            ov.insert(ov.end(), {{"pv_isc", sc == "bf-flat-ccm-open" ? "1.9" : "1.83"}, {"pv_voc", "69"}, {"pv_k", "0.86"}, {"v_bat", "26.2"}});
        else if (sc.rfind("dcd-hv-", 0) == 0) // ~1.5 x the CCM boundary current at Vin ~80 V
            ov.insert(ov.end(), {{"pv_isc", dcdFry ? "1.7" : "2.1"}, {"pv_voc", "95"}, {"pv_k", "0.84"}, {"v_bat", "26.8"}});
        else if (dcd)
            ov.insert(ov.end(), {{"pv_isc", sc.rfind("dcd-ccm", 0) == 0 ? "6" : "2.8"}, {"pv_voc", dcdFry ? "78" : "62"},
                                 {"pv_k", dcdFry ? "0.86" : "0.95"}, {"v_bat", dcdFry ? "26.8" : "25.6"}});
        else if (flatTake) ov.insert(ov.end(), {{"pv_isc", "2.8"}, {"pv_voc", "62"}, {"pv_k", "0.95"}, {"v_bat", "25.6"}});
        if (sc == "hs-cal") ov.insert(ov.end(), {{"hs_short", "1"}, {"pv_isc", "1.0"}});
        if (sc.rfind("hs-dawn", 0) == 0) ov.push_back({"noise_iout", "0.04"});
    }
    if (sc.rfind("wx-bms-", 0) == 0) {
        std::string c = "charger.conf:vout_max_fallback=26.8";
        if (const char *e = getenv("SIM_CONF")) c = c + "," + e;
        setenv("SIM_CONF", c.c_str(), 1);
    }
    if (wx.ioutMax > 0) {
        std::string c = "limits.conf:lv_i_max=" + std::to_string(wx.ioutMax);
        if (const char *e = getenv("SIM_CONF")) c = c + "," + e;
        setenv("SIM_CONF", c.c_str(), 1);
    }
    if (sc == "wx-cloud-cv-ripple") ov = {{"noise_vout", "0.03"}, {"vbat_ac_amp", "0.26"}, {"vbat_ac_freq", "100"}};
    const bool rcal = sc.rfind("rcal", 0) == 0;
    if (rcal) ov = {{"noise_iout", "0.01"}, {"noise_vout", "0.005"}, {"noise_vin", "0.05"}};
    if (sc == "rcal-flat") ov.insert(ov.end(), {{"pv_isc", "7.5"}, {"pv_voc", "66"}, {"v_bat", "26.7"}});
    if (sc == "rcal-ov") setenv("SIM_CONF", "limits.conf:hv_max=77.5", 1);
    if (sc == "rcal-sun") setenv("SIM_CONF", "coil.conf:L0=80e-6", 1);
    const bool n1 = sc.size() > 3 && sc.compare(sc.size() - 3, 3, "-n1") == 0;
    if (sc == "wx-sweep-learn-off" || sc == "wx-sweep-scan" || n1) {
        std::string c = sc == "wx-sweep-scan" ? "tracker.conf:pv_substrings=6"
                        : n1                  ? "tracker.conf:pv_substrings=1"
                                              : "tracker.conf:sweep_verify=0";
        if (const char *e = getenv("SIM_CONF")) c = c + "," + e;
        setenv("SIM_CONF", c.c_str(), 1);
    }
    if (const char *n = getenv("SIM_NOISE_IOUT"); rcal && n) ov[0].second = n;
    if (psuBoost)
        ov = {{"pv_isc", "200"}, {"pv_voc", "26.6"}, {"pv_k", "0.95"}, {"v_bat", "0"}, {"r_bat", "207.5e3"}, {"plant_c_in", "470e-6"},
              {"plant_c_out", "1.2e-3"}, {"out_d_i0", "1.2e-4"}, {"out_d_vt", "8.3"}};
    if (sc == "psu-boost-cold") {
        ov.push_back({"in_bf", "1"});
        if (const char *e = getenv("SIM_BFSTUCK"); e && *e) ov.push_back({"bflow_stuck", e});
    }
    if (mcBoost) // battery 26.6 V as a stiff source, Vout divider 207.5 kOhm, dark string I = 23 nA (exp(V / 4.45 V) - 1): fit to
        // the 6-substring string measured at night, 0.18 / 0.39 / 0.97 / 1.5 A at 70 / 74 / 78 / 80 V
        ov = {{"pv_isc", "200"}, {"pv_voc", "26.6"}, {"pv_k", "0.95"}, {"v_bat", "0"}, {"r_bat", "207.5e3"}, {"plant_c_in", "470e-6"},
              {"plant_c_out", "1.2e-3"}, {"out_d_i0", "2.3e-8"}, {"out_d_vt", "4.45"}, {"noise_iin", "0.02"}, {"noise_vin", "0.02"},
              {"noise_vout", "0.05"}, {"in_bf", "1"}, {"bflow_stuck", sc == "mc-boost-bfopen" ? "1" : "0"}};
    g_batAuto = sc.rfind("bat-auto", 0) == 0;
    if (const char *e = getenv("SIM_MC_TON")) ov.push_back({"t_on_ofs", e});
    boot(ov, p0);
    if (mcBoost && getenv("SIM_MC_L")) g_vconv.setPassives(g_vconv.getCin(), g_vconv.getCout(), (float) atof(getenv("SIM_MC_L")));
    if (rcal) g_vconv.setDiodeVf(0.7f);
    if (sc == "rcal-sun" && !g_ring) g_vconv.setRing(5e-9f, 40.f);
    if (sc == "rcal-sun") g_vconv.setPv(5.6f, 79.6f, 0.78f);
    if (sc == "rcal-pv") {
        float isc = 5.6f, voc = 79.6f, k = 0.78f;
        if (const char *e = getenv("SIM_PV")) sscanf(e, "%f,%f,%f", &isc, &voc, &k);
        g_vconv.setPv(isc, voc, k);
    }
    if (g_ring) {
        g_vconv.setPassives(g_vconv.getCin(), g_vconv.getCout(), g_ring->lPlant);
        printf("# board plant %s: coil.conf L0 %.0f uH, plant L %.0f uH, ring %s (%.0f kHz at %.0f V, Q %.1f, node C %.2f nF),"
               " Vf %.2f V, dead %.0f ns, R %.0f/%.0f mOhm, t_sw %.0f ns\n", g_ring->name, g_ring->l0Conf * 1e6f,
               g_ring->lPlant * 1e6f, g_ringOn ? (g_ringDamped ? "damped" : "on") : "OFF", g_ring->ringF * 1e-3f, g_ring->ringV,
               g_ring->ringQ, g_ring->ringC() * 1e9f, g_ring->vf, g_ring->tDead * 1e9f, g_ring->rHs * 1e3f, g_ring->rLs * 1e3f,
               g_ring->tSw * 1e9f);
    }
    if (sc == "ring-coil" || sc == "ring-coil-weak") return runRingCoil(name, sc == "ring-coil-weak");
    if (isWx) return runWx(name, wx);
    if (psuBoost) return sc == "psu-boost-cold" ? runPsuBoostCold(name) : runPsuBoostDark(name);
    if (mcBoost) return runMcBoost(name);
    if (cvHold) return runCvHold(name, sc);
    if (sc.rfind("shade", 0) == 0) g_vconv.setPvShade(0.68f);

    if (sc == "sanity") {
        Probe p = probeStart();
        g_eventT = -1;
        run(90, 1.0, &p);
        printf("\n== sanity: %zu trips in 90 s\n", g_trips.size());
        for (auto &t: g_trips) printf("==   t=%.3f [%s] %us\n", t.t, t.who.c_str(), t.sec);
        traceLine("final ");
        return 0;
    }

    if (sc == "idle-wake") return runIdleWake();
    if (sc == "bat-auto-swap" || sc == "bat-swap-explicit") return runBatSwap(sc);
    if (sc.rfind("bat-auto", 0) == 0) return runBatAuto(sc);
    if (sc.rfind("hs-", 0) == 0) return runHsShort(sc);
    if (sc.rfind("bf-", 0) == 0) return runBfTest(sc);
    if (sc.rfind("mc-flat", 0) == 0) return runMcFlat(sc);
    if (sc.rfind("dcd-hv-", 0) == 0) return runDcdHv(sc);
    if (dcd) return runDcd(sc);
    settle(sc == "d-noise" ? 60 : 60);
    size_t settleTrips = g_trips.size();
    printf("# trips during settle: %zu\n", settleTrips);

    if (sc == "a" || sc == "b" || sc == "c" || sc == "e" || sc == "e-bleed") {
        const bool e = sc[0] == 'e';
        Probe p = probeStart();
        // e-bleed: open battery but a 1 kOhm residual output load (hypothetical divider/aux drain)
        g_vconv.setBat(0, sc == "e-bleed" ? 1e3f : 1e9f);
        printf("%9.4f >>> vconv bat open (r=%g)\n", tNow(), g_vconv.getRbat());
        if (const char *ph = getenv("SIM_PHASE_US")) g_vconv.stepSeconds(atof(ph) * 1e-6f, 39000);
        run(e ? 120 : 6, e ? 2.0 : 0.5, &p, 25);
        report(name.c_str(), p);
        if (e) {
            Probe q = probeStart();
            g_vconv.setBat(27.f, 0.05f);
            printf("%9.4f >>> vconv bat reconnect 27 V / 0.05 ohm\n", tNow());
            run(60, 2.0, &q, 0);
            report("e-reconnect", q);
            // the battery's return ends the no-MPP backoff: back on within seconds, no trip
            auto on = std::upper_bound(g_onT.begin(), g_onT.end(), q.t0);
            const double dOn = on == g_onT.end() ? 1e9 : *on - q.t0;
            const bool ok = dOn <= 5 && q.nOn > 0 && g_trips.size() == q.tripIdx0;
            printf("== RECONNECT %s: on %.1f s after the reconnect (want <= 5), %u on-ticks, %zu trips: %s\n", name.c_str(),
                   dOn, q.nOn, g_trips.size() - q.tripIdx0, ok ? "PASS" : "FAIL");
            if (!ok) return 1;
            traceLine("final ");
        }
    } else if (sc == "d-cloud" || sc == "d-cloud-ramp") {
        Probe p = probeStart();
        if (sc == "d-cloud") {
            g_vconv.setPv(3.f, g_vconv.getVoc(), g_vconv.getPvK());
            printf("%9.4f >>> Isc 13 -> 3 A step\n", tNow());
        } else {
            g_rampT0 = tNow(), g_rampSec = 1.0, g_rampFrom = g_vconv.getIsc(), g_rampTo = 3.0;
            printf("%9.4f >>> Isc 13 -> 3 A linear over 1 s\n", tNow());
        }
        run(60, 1.0, &p, 20);
        report(name.c_str(), p);
    } else if (sc.rfind("d-bus", 0) == 0) {
        float dv = std::stof(sc.substr(5)) / 10.f;
        Probe p = probeStart();
        g_vconv.setBat(g_vconv.getVbat() + dv, g_vconv.getRbat());
        printf("%9.4f >>> Vbat +%.1f V -> %.2f V\n", tNow(), dv, g_vconv.getVbat());
        run(30, 1.0, &p, 25);
        report(name.c_str(), p);
    } else if (sc.rfind("shade", 0) == 0) {
        // two-peak curve; report where the tracker sits and the power against the higher peak
        float pk[2] = {0, 0}, vk[2] = {0, 0};
        for (float v = 5; v < g_vconv.getVoc(); v += 0.1f) {
            int k = v > g_vconv.getVoc() * 0.6f;
            if (v * g_vconv.pvCurrent(v) > pk[k]) pk[k] = v * g_vconv.pvCurrent(v), vk[k] = v;
        }
        printf("== peaks: %.0f W at %.1f V, %.0f W at %.1f V\n", pk[0], vk[0], pk[1], vk[1]);
        const double T = getenv("SIM_T") ? atof(getenv("SIM_T")) : 600;
        double eW = 0, tLow = 0, t0 = tNow(), tLeft = -1;
        uint32_t nLim = 0, n = 0;
        g_verbose = getenv("SIM_V") != nullptr;
        for (double tEnd = tNow() + T, nextTr = tNow(); tNow() < tEnd;) {
            double tb = tNow();
            run(0.02, 0);
            double dt = tNow() - tb;
            eW += g_vconv.getVin() * g_vconv.pvCurrent(g_vconv.getVin()) * dt;
            if (g_vconv.getVin() < g_vconv.getVoc() * 0.6f) { tLow += dt; if (tLeft < 0) tLeft = tNow() - t0; }
            ++n; if (mppt.teleSnap().limIdx != 15) ++nLim;
            if (tNow() >= nextTr) { traceLine(); nextTr += T / 40; }
        }
        float best = std::max(pk[0], pk[1]);
        printf("\n== %s: mean PV power %.1f W = %.1f %% of the higher peak; %.0f s of %.0f s on the low-voltage side"
               " (first at +%.0f s); limiter active in %.1f %% of 20 ms slices; %zu trips\n", name.c_str(), eW / T,
               100 * eW / T / best, tLow, T, tLeft, 100.0 * nLim / n, g_trips.size());
        return 0;
    } else if (sc == "d-noise") {
        Probe p = probeStart();
        run(300, 10.0, &p);
        report(name.c_str(), p);
    } else if (rcal) {
        return runRectCal(name, sc);
    } else {
        printf("unknown scenario %s\n", sc.c_str());
        return 1;
    }

    printf("\n== log after event (first %zu lines):\n", g_logSince.size());
    for (size_t i = 0; i < g_logSince.size() && i < 60; ++i) printf("   %s\n", g_logSince[i].c_str());
    return 0;
}

int main(int argc, char **argv) {
    g_simRtAlloc.now = tNow;
    if (const char *e = getenv("SIM_NOALLOC")) g_noAllocAfter = atof(e);
    int rc = simMain(argc, argv);
    auto &a = g_simRtAlloc;
    printf("== rt-alloc: %llu allocs, %llu frees in the RT iteration (%s)", (unsigned long long) a.allocs,
           (unsigned long long) a.frees, a.mallocHooked ? "new/delete + malloc zone" : "new/delete only");
    if (g_noAllocAfter >= 0)
        printf("; after %.0f s: %llu allocs, %llu frees, %d sites", g_noAllocAfter, (unsigned long long) a.armedAllocs,
               (unsigned long long) a.armedFrees, a.nSites);
    printf("\n");
    fflush(stdout);
    if (g_noAllocAfter >= 0 && (a.armedAllocs || a.armedFrees)) return rc ? rc : 3;
    return rc;
}
