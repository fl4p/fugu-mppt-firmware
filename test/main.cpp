#include <unity.h>
//#include <task.h>
#include <Arduino.h>

#include "storage/key-value.h"
#include "util.h"

//#warning "Test main"

// src/main.cpp defines this in the normal build; with RUN_TESTS=1 the entry point
// is swapped to this file, so we provide our own definition. wallClockUs() reads it,
// but nothing in tests writes — that's fine, time-based tests pass explicit timestamps.
time_us loopWallClockUs_ = 0;

// Same swap: src/main.cpp owns the real nvs; telemetry.cpp (linked into the test
// build) references it, so define it here too.
KeyValueStorage nvs{};

// telemetry.cpp references the global `mppt`; mqtt.cpp references handleCommand() (cmd_input).
// Both live in src/main.cpp / src/cli.cpp in the normal build, which RUN_TESTS excludes — so
// stub them here to link. The ctor args use internal linkage to avoid clashing with any test
// TU's own globals; construction mirrors the real static-init (null sensors, empty LCD).
#include "adc/sampling.h"
#include "buck.h"
#include "viz/lcd.h"
#include "mppt.h"
static ADC_Sampler s_adcSampler{};
static VIinVout<const Sensor *> s_sensors{nullptr, nullptr, nullptr, nullptr};
static SynchronousConverter s_converter{};
static LCD s_lcd{};
MpptController mppt{s_adcSampler, s_sensors, s_converter, s_lcd};

// RT/ADC path (temperature.h, mppt) reaches the scope streamer through this pointer; null = off.
Scope *scope = nullptr;

// More main.cpp-owned globals the netw objects reference: g_app (mode flags, via mppt.telemetry())
// and lastTimeOutUs (telnet onConnect). test_security.cpp pulls telnet_service.cpp into the link,
// so define them here too (same swap as nvs/mppt above).
#include "app_state.h"
AppState g_app{};
time_us lastTimeOutUs = 0;

// Instrumented stub: test_security.cpp asserts the telnet command path DEFERS handleCommand (via
// enqueue_task) instead of running it inline (the UAF fix). Record calls so the test can observe
// that the command runs only after process_queued_tasks(), and with the expected (trimmed) text.
int g_handleCommandCalls = 0;
String g_lastHandleCommand;

bool handleCommand(const String &s) {
    ++g_handleCommandCalls;
    g_lastHandleCommand = s;
    return false;
}

// Mirror of src/main.cpp's shim: arduino-esp32 WiFiGeneric.cpp calls esp_netif_create_default_wifi_ap()
// unconditionally, but esp_wifi only defines it under CONFIG_ESP_WIFI_SOFTAP_SUPPORT=y (we keep it off).
// RUN_TESTS swaps main.cpp out, so re-provide the stub here.
extern "C" void *esp_netif_create_default_wifi_ap(void) { return nullptr; }

// adc/adc_esp32_cont.h + INA226 interrupt-path liveness (test_adc_hw.cpp) — on-target, no converter
void test_internal_adc_continuous_samples();
void test_ina226_alert_interrupt();

// test_adc_min.cpp — GPIO-ISR core-affinity (the invariant pinGpioIsrToRtCore() relies on).
// On-target, self-triggers TEST_ISR_GPIO; bench-only (default pin 21 is fry's HS gate).
void test_attachinterrupt_after_preinstall_uses_rt_core();
void test_gpio_isr_lands_on_install_core_0();
void test_gpio_isr_lands_on_install_core_rt();

void test_meter();

void test_meter_storage();

void test_LinearTransform();

// test_ADCSampler() body is currently #if 0'd in test/test_sampler.cpp — skip
// the RUN_TEST below until that is re-enabled.
// void test_ADCSampler();

// adc/sampling.h — muxed-cycle Vout interleave + notch-rate correction
void test_vout_interleave_poll_order();
void test_vout_interleave_notch_rate();
void test_calibration_counts_each_sensor_once();
void test_calibration_reset_is_deferred_to_update();
void test_calibration_restart_is_deferred_to_update();
void test_calibration_cancel_is_deferred_to_update();
void test_cycle_no_interleave_two_channels();
void test_streamed_watchdog_does_not_deadlock_read();
// etc/rt.h — TaskNotification binary-semaphore wait() (boot "ADC error" burst regression)
void test_tasknotification_burst_reads_as_one_wakeup();
void test_tasknotification_single_wakeup_then_empty();

// adc/vconv.h + sim/vconv.h — coalesced-tick catch-up stepping (waitCount + capped hasData)
void test_vconv_waitcount_returns_burst_count();
void test_vconv_catchup_equals_uncoalesced();
void test_vconv_single_step_drops_simtime();
void test_vconv_hasdata_steps_capped_coalesced_count();
void test_vconv_inverter_ripple_is_sine();
void test_vconv_rectifier_ripple_has_2nd_harmonic();
void test_vconv_spiky_ripple_is_harmonic_rich();
void test_vconv_no_ripple_when_amp_zero();
void test_vconv_pluggable_custom_ripple_shape();

// test_dawn.cpp — replays fry's 2026-05-31 dawn (Voc ramp + idle-START gate timing)
void test_dawn_vin_gate_clears_fast_and_stays_open();
void test_dawn_calibration_not_the_blocker();
void test_dawn_high_voc_panel_only_yields_a_few_watts();

// adc/ina226_conv_time.h + adc/adc.h (AsyncADC contract via ADC_Dummy)
void test_ina226_convtime_exact_step();
void test_ina226_convtime_rounds_up_between_steps();
void test_ina226_convtime_clamps_low();
void test_ina226_convtime_clamps_high();
void test_ina226_alert_timeout_floor_and_headroom();
void test_ina226_sample_rate();
void test_asyncadc_channel_select_and_sequence();
void test_asyncadc_max_expected_voltage_roundtrip();
void test_asyncadc_scheme_is_all();

void test_float16();

// charger.h
void test_termination_line_at_zero_current();
void test_termination_line_at_tail_current();
void test_termination_line_clamps_above_cv_eoc();
void test_termination_line_midpoint();
void test_termination_does_not_latch_below_line();
void test_termination_latches_above_line();
void test_termination_latches_when_vcell_above_cv_eoc_at_high_current();
void test_termination_release_via_dod();
void test_termination_release_via_voltage_floor();
void test_termination_dod_release_skipped_when_cbat_missing();
void test_termination_reset_clears_latch();
void test_coulomb_counter_starts_at_zero();
void test_coulomb_counter_discharge_accumulates();
void test_coulomb_counter_charging_decrements();
void test_coulomb_counter_markfull_resets();
void test_coulomb_counter_drops_gap_over_maxdt();
void test_mqtt_vcell_message_updates_state();
void test_mqtt_ibat_message_drives_coulomb_counter();
void test_mqtt_ibat_lim_accepts_valid();
void test_mqtt_ibat_lim_rejects_negative();
void test_mqtt_ibat_lim_rejects_nan();

// asciichart/ascii.h
void test_ascii_empty_single_series_emits_nothing();
void test_ascii_empty_multi_series_emits_nothing();
void test_ascii_multi_with_only_empty_inner_emits_nothing();
void test_ascii_all_nan_series_emits_nothing();
void test_ascii_leading_nan_does_not_pollute_min();
void test_ascii_trailing_nan_does_not_pollute_max();
void test_ascii_embedded_nan_gap_does_not_crash();
void test_ascii_single_finite_among_nan_renders();
void test_ascii_first_sample_nan_does_not_misplace_marker();
void test_ascii_constant_series_does_not_divide_by_zero();
void test_ascii_two_point_series();
void test_ascii_custom_min_max_used();
void test_ascii_zero_crossing_draws_center_glyph();
void test_ascii_all_positive_uses_axis_glyph();
void test_ascii_zero_offset_clamps_safely();
void test_ascii_negative_offset_clamps_safely();
void test_ascii_every_emitted_line_ends_with_reset();
void test_ascii_multi_series_uses_distinct_colors();
void test_ascii_multi_series_unequal_length_does_not_crash();
void test_ascii_multi_series_with_one_empty_renders_other();
void test_ascii_regression_plot_h_first_bin_nan();

// math/ripple_freq.h — adaptive inverter-ripple notch
void test_ripple_detects_100hz();
void test_ripple_detects_120hz();
void test_ripple_subbin_interpolation();
void test_ripple_ignores_dc_no_false_positive();
void test_ripple_detects_real_fry_capture();
void test_adaptive_notch_beats_fixed_100hz_on_real_data();

// math/statmath.h
void test_ewma_seeds_on_first_finite();
void test_ewma_ignores_nan();
void test_ewma_converges_to_constant();
void test_ewma_reset_and_span();
void test_ewma_npass_converges_to_constant();
void test_median3_first_sample_passthrough();
void test_median3_rejects_spike();
void test_median3_picks_middle();
void test_median5_picks_middle();
void test_median5_rejects_spike();
void test_mean_accumulator_pop_resets();
void test_ewm_mean_tracks_and_var_zero_for_constant();
void test_integrator_constant_rate();
void test_integrator_drops_gap_over_maxdt();
void test_integrator_reset_and_restore();

// pd_control.h
void test_pd_proportional_only();
void test_pd_first_tick_has_zero_derivative();
void test_pd_derivative_on_step();
void test_pd_normalize_relative_error();
void test_pd_reset_clears_derivative();
void test_pd_smooth_setpoint_lags_step();
void test_pd_legacy_kd_is_sample_rate_coupled();
void test_pd_td_is_sample_rate_invariant();
void test_pd_td_matches_legacy_at_nominal_ts();
void test_pd_td_zero_dt_suppresses_derivative();
void test_pd_load_gains_empty_conf_is_noop();
void test_pd_load_gains_applies_keys();
void test_pd_load_gains_rejects_negative_td();
void test_pd_load_gains_rejects_nonfinite_kp();
void test_pd_negative_td_would_flip_limiter_sign();

// buck.h
void test_buck_ripple_current();
void test_buck_dcm_ccm_transition_and_hysteresis();
void test_buck_rect_ctrl_ratio();
void test_buck_current_sweep_no_crash();
void test_buck_ratio_clamped_below_unity();
void test_buck_ratio_clamped_when_vout_ge_vin();
void test_buck_ratio_never_negative_sweep();
void test_buck_dcm_rectmax_not_full_ccm_near_unity();
void test_buck_sync_rect_off_below_min_current();
void test_buck_sync_rect_active_in_dcm_with_current();
void test_buck_bootstrap_min_default();
void test_buck_bootstrap_min_scales_with_conf();
void test_boost_bootstrap_min_is_zero();
void test_boost_ratio_clamped_above_unity();
#if defined(HAVE_MCPWM) && defined(HAVE_LEGACY)
void test_buck_pwm_driver_runtime_select();
void test_pwm_driver_defaults_to_ledc();
void test_pwm_driver_invalid_throws();
#endif

// pwm — measurement-rig self-test (doc/pwm-test-spec1.md)
void test_pwm_rig_freq_path();
void test_pwm_rig_pulsewidth_path();
// pwm — MCPWM tests (spec1 §Tests/1, 2, 3, 4a, 4b)
void test_mcpwm_hs_freq();
void test_mcpwm_hs_duty();
void test_mcpwm_pwmmax_arithmetic();
void test_mcpwm_deadband_hs_to_ls();
void test_mcpwm_deadband_ls_to_hs();
void test_mcpwm_deadtime_linearity();
void test_mcpwm_ls_force_off();
void test_mcpwm_ls_force_on();
void test_mcpwm_d0_hs_low();
void test_mcpwm_d1_hs_high();
void test_mcpwm_glitch_free_duty_step();
void test_mcpwm_ost_brake_latches();
void test_mcpwm_interleaved_phase();
void test_mcpwm_boot_safe_low();
void test_mcpwm_enlogic_reenable();
void test_mcpwm_interleaved_phase_deadtime();
void test_rect_offset_ns_conversion();
void test_mcpwm_endpoint_duty_scope();

// tracker.h — perturb&observe MPPT
void test_tracker_low_power_forces_pump();
void test_tracker_reverses_on_power_drop();
void test_tracker_keeps_direction_on_power_rise();
void test_tracker_deadband_ignores_small_drop();
void test_tracker_cloud_recovery_reverses_on_vin_jump();
void test_tracker_converges_to_peak();

// conf.h
void test_conf_getlong_bases();
void test_conf_getlong_default_when_missing();
void test_conf_getfloat();
void test_conf_getstring();
void test_conf_required_missing_long_throws();
void test_conf_required_missing_string_throws();
void test_conf_operator_bool();
void test_conf_remove_drops_line_and_keeps_rest();
void test_conf_remove_strips_inline_comment_too();
void test_conf_remove_missing_key_returns_false();

// test_security.cpp — strntof OOB (#7), NVS readString boot-loop (#8), telnet defer/UAF (#9)
void test_strntof_empty_returns_nan_no_oob();
void test_strntof_parses_nul_terminated();
void test_strntof_parses_unterminated_slice();
void test_nvs_readstring_roundtrips_long_value();
void test_nvs_readstring_short_and_missing();
void test_telnet_command_is_deferred_then_runs_on_drain();
void test_telnet_blank_command_is_dropped();
// test_charger.cpp — empty MQTT payload survives all three BMS callbacks (#7 attack path)
void test_mqtt_empty_payload_is_safe();
// test_charger.cpp — releaseVoutPinning releases the Vout pin UP to Vbat_max, not down to fallback
void test_release_vout_pinning_goes_to_vbat_max();
// test_charger.cpp — EOC float floor drops below Vbat_fallback by vout_offset_max to absorb a Vout offset
void test_eoc_floor_allows_vout_offset_correction();
void test_eoc_floor_zero_offset_stops_at_fallback();
void test_eoc_floor_scales_with_offset();
void test_terminated_no_authority_yields_low();
void test_not_terminated_no_authority_releases_high();
// test_charger.cpp — releaseVoutPinning must reset the terminated-state memory so the float glide restarts
void test_release_vout_pinning_restarts_float_glide();

// test_psu_mode.cpp — PSU constant-voltage mode
void test_psu_ov_threshold_from_setpoint();
void test_psu_ov_threshold_clamped_to_vout_max();
void test_psu_ov_threshold_explicit_ovset_overrides();
void test_psu_ov_threshold_no_setpoint_falls_back();
void test_psu_trip_escalates_after_repeated_trips();
void test_psu_trip_latches_after_many_trips();
void test_psu_trip_sparse_does_not_escalate();
void test_psu_trip_permanent_fault_reaches_latch();
void test_psu_serious_fault_stays_out_of_fast_retry_bucket();
void test_psu_trip_latch_blocks_start();
void test_psu_reset_clears_latch();
void test_psu_setpoint_rejects_negative();
void test_psu_setpoint_rejects_zero();
void test_psu_setpoint_rejects_nan();
void test_psu_setpoint_rejects_above_vout_max();
void test_psu_setpoint_accepts_valid();
void test_psu_boost_rejects_setpoint_below_input();
void test_psu_boost_accepts_setpoint_with_headroom();
void test_psu_boost_rejects_unknown_input();
void test_psu_setpoint_rejects_explicit_ov_conflict();
void test_psu_enable_waits_for_fresh_telemetry();
void test_psu_later_override_wins_before_rt_apply();
void test_psu_completion_keeps_earlier_concurrent_ticket();
void test_psu_short_low_side_transition_is_rt_owned();
void test_psu_mode_flag_correct();

// test_pv_sim.cpp — PV-sim solar-array-simulator curve mode
void test_pv_rejects_bad_params();
void test_pv_rejects_ovset_conflict();
void test_pv_enable_enters_psu_mode_at_voc();
void test_pv_enable_waits_for_fresh_telemetry();
void test_pv_inplace_update_keeps_setpoint();
void test_pv_scale_rebase_semantics();
void test_pv_disable_paths_clear_active();
void test_pv_plain_psu_enable_reverts_to_cv();
void test_pv_pending_overridden_by_manual();
void test_pv_ov_threshold_pinned_at_voc();
void test_pv_requested_setpoint_reports_voc();
void test_pv_trips_reach_latch();
void test_pv_advance_slew_clamp();
void test_pv_advance_dt_clamp();
void test_pv_advance_vin_floor();
void test_pv_advance_voc_ceiling_holds_on_empty_interval();
void test_pv_advance_tracks_curve_point();

void setup() {
#ifdef TEST_ADC_HW
    // Safe to flash onto a live converter: force both gate-driver inputs low (fry pwm_hi=21,
    // pwm_li=14, HiLi logic -> both FETs off). The tests then re-run from loop() so a console
    // attached at any time (fugu_console.py) catches a fresh cycle — no reset-timing needed.
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);
    pinMode(14, OUTPUT);
    digitalWrite(14, LOW);
    return;
#endif

    UNITY_BEGIN();

    // charger.h — termination math
    RUN_TEST(test_termination_line_at_zero_current);
    RUN_TEST(test_termination_line_at_tail_current);
    RUN_TEST(test_termination_line_clamps_above_cv_eoc);
    RUN_TEST(test_termination_line_midpoint);
    // charger.h — latch/release
    RUN_TEST(test_termination_does_not_latch_below_line);
    RUN_TEST(test_termination_latches_above_line);
    RUN_TEST(test_termination_latches_when_vcell_above_cv_eoc_at_high_current);
    RUN_TEST(test_termination_release_via_dod);
    RUN_TEST(test_termination_release_via_voltage_floor);
    RUN_TEST(test_termination_dod_release_skipped_when_cbat_missing);
    RUN_TEST(test_termination_reset_clears_latch);
    // etc/coulomb_counter.h
    RUN_TEST(test_coulomb_counter_starts_at_zero);
    RUN_TEST(test_coulomb_counter_discharge_accumulates);
    RUN_TEST(test_coulomb_counter_charging_decrements);
    RUN_TEST(test_coulomb_counter_markfull_resets);
    RUN_TEST(test_coulomb_counter_drops_gap_over_maxdt);
    // charger.h — MQTT callback integration
    RUN_TEST(test_mqtt_vcell_message_updates_state);
    RUN_TEST(test_mqtt_ibat_message_drives_coulomb_counter);
    RUN_TEST(test_mqtt_ibat_lim_accepts_valid);
    RUN_TEST(test_mqtt_ibat_lim_rejects_negative);
    RUN_TEST(test_mqtt_ibat_lim_rejects_nan);

    // asciichart/ascii.h — NaN handling regression + edge cases
    RUN_TEST(test_ascii_empty_single_series_emits_nothing);
    RUN_TEST(test_ascii_empty_multi_series_emits_nothing);
    RUN_TEST(test_ascii_multi_with_only_empty_inner_emits_nothing);
    RUN_TEST(test_ascii_all_nan_series_emits_nothing);
    RUN_TEST(test_ascii_leading_nan_does_not_pollute_min);
    RUN_TEST(test_ascii_trailing_nan_does_not_pollute_max);
    RUN_TEST(test_ascii_embedded_nan_gap_does_not_crash);
    RUN_TEST(test_ascii_single_finite_among_nan_renders);
    RUN_TEST(test_ascii_first_sample_nan_does_not_misplace_marker);
    RUN_TEST(test_ascii_constant_series_does_not_divide_by_zero);
    RUN_TEST(test_ascii_two_point_series);
    RUN_TEST(test_ascii_custom_min_max_used);
    RUN_TEST(test_ascii_zero_crossing_draws_center_glyph);
    RUN_TEST(test_ascii_all_positive_uses_axis_glyph);
    RUN_TEST(test_ascii_zero_offset_clamps_safely);
    RUN_TEST(test_ascii_negative_offset_clamps_safely);
    RUN_TEST(test_ascii_every_emitted_line_ends_with_reset);
    RUN_TEST(test_ascii_multi_series_uses_distinct_colors);
    RUN_TEST(test_ascii_multi_series_unequal_length_does_not_crash);
    RUN_TEST(test_ascii_multi_series_with_one_empty_renders_other);
    RUN_TEST(test_ascii_regression_plot_h_first_bin_nan);

    // math/ripple_freq.h — adaptive inverter-ripple notch (synthetic + real fry capture)
    RUN_TEST(test_ripple_detects_100hz);
    RUN_TEST(test_ripple_detects_120hz);
    RUN_TEST(test_ripple_subbin_interpolation);
    RUN_TEST(test_ripple_ignores_dc_no_false_positive);
    RUN_TEST(test_ripple_detects_real_fry_capture);
    RUN_TEST(test_adaptive_notch_beats_fixed_100hz_on_real_data);

    // math/statmath.h — EWMA, median, mean, EWM, integrator
    RUN_TEST(test_ewma_seeds_on_first_finite);
    RUN_TEST(test_ewma_ignores_nan);
    RUN_TEST(test_ewma_converges_to_constant);
    RUN_TEST(test_ewma_reset_and_span);
    RUN_TEST(test_ewma_npass_converges_to_constant);
    RUN_TEST(test_median3_first_sample_passthrough);
    RUN_TEST(test_median3_rejects_spike);
    RUN_TEST(test_median3_picks_middle);
    RUN_TEST(test_median5_picks_middle);
    RUN_TEST(test_median5_rejects_spike);
    RUN_TEST(test_mean_accumulator_pop_resets);
    RUN_TEST(test_ewm_mean_tracks_and_var_zero_for_constant);
    RUN_TEST(test_integrator_constant_rate);
    RUN_TEST(test_integrator_drops_gap_over_maxdt);
    RUN_TEST(test_integrator_reset_and_restore);

    // pd_control.h
    RUN_TEST(test_pd_proportional_only);
    RUN_TEST(test_pd_first_tick_has_zero_derivative);
    RUN_TEST(test_pd_derivative_on_step);
    RUN_TEST(test_pd_normalize_relative_error);
    RUN_TEST(test_pd_reset_clears_derivative);
    RUN_TEST(test_pd_smooth_setpoint_lags_step);
    RUN_TEST(test_pd_legacy_kd_is_sample_rate_coupled);
    RUN_TEST(test_pd_td_is_sample_rate_invariant);
    RUN_TEST(test_pd_td_matches_legacy_at_nominal_ts);
    RUN_TEST(test_pd_td_zero_dt_suppresses_derivative);
    RUN_TEST(test_pd_load_gains_empty_conf_is_noop);
    RUN_TEST(test_pd_load_gains_applies_keys);
    RUN_TEST(test_pd_load_gains_rejects_negative_td);
    RUN_TEST(test_pd_load_gains_rejects_nonfinite_kp);
    RUN_TEST(test_pd_negative_td_would_flip_limiter_sign);

    // tracker.h — perturb&observe MPPT. Runs before buck.h because on the ESP32-classic the buck
    // tests drive LEDC on pwm_hi=1 (= U0TXD), hijacking the console UART — anything logged after
    // that is lost. (On the S3 those GPIOs aren't the console, so order is immaterial there.)
    RUN_TEST(test_tracker_low_power_forces_pump);
    RUN_TEST(test_tracker_reverses_on_power_drop);
    RUN_TEST(test_tracker_keeps_direction_on_power_rise);
    RUN_TEST(test_tracker_deadband_ignores_small_drop);
    RUN_TEST(test_tracker_cloud_recovery_reverses_on_vin_jump);
    RUN_TEST(test_tracker_converges_to_peak);

    // buck.h — diode-emulation / sync-rect math
    RUN_TEST(test_buck_ripple_current);
    RUN_TEST(test_buck_dcm_ccm_transition_and_hysteresis);
    RUN_TEST(test_buck_rect_ctrl_ratio);
    RUN_TEST(test_buck_current_sweep_no_crash);
    RUN_TEST(test_buck_ratio_clamped_below_unity);
    RUN_TEST(test_buck_ratio_clamped_when_vout_ge_vin);
    RUN_TEST(test_buck_ratio_never_negative_sweep);
    RUN_TEST(test_buck_dcm_rectmax_not_full_ccm_near_unity);
    RUN_TEST(test_buck_sync_rect_off_below_min_current);
    RUN_TEST(test_buck_sync_rect_active_in_dcm_with_current);
    RUN_TEST(test_buck_bootstrap_min_default);
    RUN_TEST(test_buck_bootstrap_min_scales_with_conf);
    RUN_TEST(test_boost_bootstrap_min_is_zero);
    RUN_TEST(test_boost_ratio_clamped_above_unity);
#if defined(HAVE_MCPWM) && defined(HAVE_LEGACY)
    RUN_TEST(test_buck_pwm_driver_runtime_select);
    RUN_TEST(test_pwm_driver_defaults_to_ledc);
    RUN_TEST(test_pwm_driver_invalid_throws);
#endif

    // conf.h — ConfFile getters
    RUN_TEST(test_conf_getlong_bases);
    RUN_TEST(test_conf_getlong_default_when_missing);
    RUN_TEST(test_conf_getfloat);
    RUN_TEST(test_conf_getstring);
    RUN_TEST(test_conf_required_missing_long_throws);
    RUN_TEST(test_conf_required_missing_string_throws);
    RUN_TEST(test_conf_operator_bool);
    RUN_TEST(test_conf_remove_drops_line_and_keeps_rest);
    RUN_TEST(test_conf_remove_strips_inline_comment_too);
    RUN_TEST(test_conf_remove_missing_key_returns_false);

    // security/robustness fixes: strntof OOB (#7), NVS readString boot-loop (#8), telnet defer (#9)
    RUN_TEST(test_strntof_empty_returns_nan_no_oob);
    RUN_TEST(test_strntof_parses_nul_terminated);
    RUN_TEST(test_strntof_parses_unterminated_slice);
    RUN_TEST(test_mqtt_empty_payload_is_safe);
    RUN_TEST(test_release_vout_pinning_goes_to_vbat_max);
    RUN_TEST(test_eoc_floor_allows_vout_offset_correction);
    RUN_TEST(test_eoc_floor_zero_offset_stops_at_fallback);
    RUN_TEST(test_eoc_floor_scales_with_offset);
    RUN_TEST(test_terminated_no_authority_yields_low);
    RUN_TEST(test_not_terminated_no_authority_releases_high);
    RUN_TEST(test_release_vout_pinning_restarts_float_glide);

    // PSU mode — constant-voltage operating mode
    RUN_TEST(test_psu_mode_flag_correct);
    RUN_TEST(test_psu_ov_threshold_from_setpoint);
    RUN_TEST(test_psu_ov_threshold_clamped_to_vout_max);
    RUN_TEST(test_psu_ov_threshold_explicit_ovset_overrides);
    RUN_TEST(test_psu_ov_threshold_no_setpoint_falls_back);
    RUN_TEST(test_psu_setpoint_rejects_negative);
    RUN_TEST(test_psu_setpoint_rejects_zero);
    RUN_TEST(test_psu_setpoint_rejects_nan);
    RUN_TEST(test_psu_setpoint_rejects_above_vout_max);
    RUN_TEST(test_psu_setpoint_accepts_valid);
    RUN_TEST(test_psu_boost_rejects_setpoint_below_input);
    RUN_TEST(test_psu_boost_accepts_setpoint_with_headroom);
    RUN_TEST(test_psu_boost_rejects_unknown_input);
    RUN_TEST(test_psu_setpoint_rejects_explicit_ov_conflict);
    RUN_TEST(test_psu_enable_waits_for_fresh_telemetry);
    RUN_TEST(test_psu_later_override_wins_before_rt_apply);
    RUN_TEST(test_psu_completion_keeps_earlier_concurrent_ticket);
    RUN_TEST(test_psu_short_low_side_transition_is_rt_owned);
    RUN_TEST(test_psu_trip_escalates_after_repeated_trips);
    RUN_TEST(test_psu_trip_latches_after_many_trips);
    RUN_TEST(test_psu_trip_sparse_does_not_escalate);
    RUN_TEST(test_psu_trip_permanent_fault_reaches_latch);
    RUN_TEST(test_psu_serious_fault_stays_out_of_fast_retry_bucket);
    RUN_TEST(test_psu_trip_latch_blocks_start);
    RUN_TEST(test_psu_reset_clears_latch);

    // PV-sim — solar-array-simulator curve mode (on the PSU machinery)
    RUN_TEST(test_pv_rejects_bad_params);
    RUN_TEST(test_pv_rejects_ovset_conflict);
    RUN_TEST(test_pv_enable_enters_psu_mode_at_voc);
    RUN_TEST(test_pv_enable_waits_for_fresh_telemetry);
    RUN_TEST(test_pv_inplace_update_keeps_setpoint);
    RUN_TEST(test_pv_scale_rebase_semantics);
    RUN_TEST(test_pv_disable_paths_clear_active);
    RUN_TEST(test_pv_plain_psu_enable_reverts_to_cv);
    RUN_TEST(test_pv_pending_overridden_by_manual);
    RUN_TEST(test_pv_ov_threshold_pinned_at_voc);
    RUN_TEST(test_pv_requested_setpoint_reports_voc);
    RUN_TEST(test_pv_trips_reach_latch);
    RUN_TEST(test_pv_advance_slew_clamp);
    RUN_TEST(test_pv_advance_dt_clamp);
    RUN_TEST(test_pv_advance_vin_floor);
    RUN_TEST(test_pv_advance_voc_ceiling_holds_on_empty_interval);
    RUN_TEST(test_pv_advance_tracks_curve_point);
    RUN_TEST(test_nvs_readstring_roundtrips_long_value);
    RUN_TEST(test_nvs_readstring_short_and_missing);
    RUN_TEST(test_telnet_command_is_deferred_then_runs_on_drain);
    RUN_TEST(test_telnet_blank_command_is_dropped);

    RUN_TEST(test_float16);

    RUN_TEST(test_meter);
    RUN_TEST(test_meter_storage);

    RUN_TEST(test_LinearTransform);
    // RUN_TEST(test_ADCSampler); // body is #if 0'd in test_sampler.cpp
    RUN_TEST(test_vout_interleave_poll_order);
    RUN_TEST(test_vout_interleave_notch_rate);
    RUN_TEST(test_calibration_counts_each_sensor_once);
    RUN_TEST(test_calibration_reset_is_deferred_to_update);
    RUN_TEST(test_calibration_restart_is_deferred_to_update);
    RUN_TEST(test_calibration_cancel_is_deferred_to_update);
    RUN_TEST(test_cycle_no_interleave_two_channels);
    RUN_TEST(test_streamed_watchdog_does_not_deadlock_read);
    RUN_TEST(test_tasknotification_burst_reads_as_one_wakeup);
    RUN_TEST(test_tasknotification_single_wakeup_then_empty);

    // adc/vconv.h + sim/vconv.h — coalesced-tick catch-up stepping
    RUN_TEST(test_vconv_waitcount_returns_burst_count);
    RUN_TEST(test_vconv_catchup_equals_uncoalesced);
    RUN_TEST(test_vconv_single_step_drops_simtime);
    RUN_TEST(test_vconv_hasdata_steps_capped_coalesced_count);
    RUN_TEST(test_vconv_inverter_ripple_is_sine);
    RUN_TEST(test_vconv_rectifier_ripple_has_2nd_harmonic);
    RUN_TEST(test_vconv_spiky_ripple_is_harmonic_rich);
    RUN_TEST(test_vconv_no_ripple_when_amp_zero);
    RUN_TEST(test_vconv_pluggable_custom_ripple_shape);

    // test_dawn.cpp — 2026-05-31 dawn replay
    RUN_TEST(test_dawn_vin_gate_clears_fast_and_stays_open);
    RUN_TEST(test_dawn_calibration_not_the_blocker);
    RUN_TEST(test_dawn_high_voc_panel_only_yields_a_few_watts);

    // adc/ina226_conv_time.h
    RUN_TEST(test_ina226_convtime_exact_step);
    RUN_TEST(test_ina226_convtime_rounds_up_between_steps);
    RUN_TEST(test_ina226_convtime_clamps_low);
    RUN_TEST(test_ina226_convtime_clamps_high);
    RUN_TEST(test_ina226_alert_timeout_floor_and_headroom);
    RUN_TEST(test_ina226_sample_rate);
    // adc/adc.h — AsyncADC contract
    RUN_TEST(test_asyncadc_channel_select_and_sequence);
    RUN_TEST(test_asyncadc_max_expected_voltage_roundtrip);
    RUN_TEST(test_asyncadc_scheme_is_all);

    // test_adc_min.cpp — GPIO-ISR core affinity. attachInterrupt test FIRST (it must be the run's
    // first attachInterrupt() — arduino-esp32 caches its lazy-install state).
    RUN_TEST(test_attachinterrupt_after_preinstall_uses_rt_core);
    RUN_TEST(test_gpio_isr_lands_on_install_core_0);
    RUN_TEST(test_gpio_isr_lands_on_install_core_rt);

    // PWM/MCPWM tests need the MCPWM peripheral; without it they ESP_ERROR_CHECK-abort the runner.
#if WITH_MCPWM
    // pwm — rig self-test first; if it fails, downstream MCPWM tests are suspect
    RUN_TEST(test_pwm_rig_freq_path);
    RUN_TEST(test_pwm_rig_pulsewidth_path);
    // pwm — MCPWM driver tests
    RUN_TEST(test_mcpwm_pwmmax_arithmetic);
    RUN_TEST(test_mcpwm_hs_freq);
    RUN_TEST(test_mcpwm_hs_duty);
    RUN_TEST(test_mcpwm_deadband_hs_to_ls);
    RUN_TEST(test_mcpwm_deadband_ls_to_hs);
    RUN_TEST(test_mcpwm_deadtime_linearity);
    RUN_TEST(test_mcpwm_ls_force_off);
    RUN_TEST(test_mcpwm_ls_force_on);
    RUN_TEST(test_mcpwm_d0_hs_low);
    RUN_TEST(test_mcpwm_d1_hs_high);
    RUN_TEST(test_mcpwm_glitch_free_duty_step);
    RUN_TEST(test_mcpwm_ost_brake_latches);
    RUN_TEST(test_mcpwm_interleaved_phase);
    RUN_TEST(test_mcpwm_interleaved_phase_deadtime);
    RUN_TEST(test_rect_offset_ns_conversion);
    RUN_TEST(test_mcpwm_boot_safe_low);
    RUN_TEST(test_mcpwm_enlogic_reenable);
    RUN_TEST(test_mcpwm_endpoint_duty_scope);
#endif // WITH_MCPWM

    UNITY_END();
}

void loop() {
#ifdef TEST_ADC_HW
    // One ADC at a time: if the board stalls, the last ESP_LOGI step localises which ADC + step.
    UNITY_BEGIN();
    printf("\n===== ADC #1: internal continuous =====\n");
    RUN_TEST(test_internal_adc_continuous_samples);
    delay(500);
    printf("\n===== ADC #2: INA226 (alert interrupt) =====\n");
    RUN_TEST(test_ina226_alert_interrupt);
    UNITY_END();
    delay(5000);
    return;
#endif
    //vTaskDelay(5);
    delay(10);
    //delay(1);
}
