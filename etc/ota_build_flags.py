"""Inspect a build's sdkconfig.json for feature flags that decide whether an OTA image is safe to
push to a real converter. Pure stdlib so it stays host-unit-testable (see test_ota_build_flags.py)
without ota.py's network/argparse side effects."""
import json
import re
import os


def _build_flag(bin_path, key):
    """Value of the sdkconfig key (CONFIG_<key> without the CONFIG_ prefix) from the sdkconfig.json
    next to bin_path's build dir, as bool; None if the config can't be read/parsed. None means
    'unknown' — callers must treat that as not-safe, never as the flag being off."""
    cfg = os.path.join(os.path.dirname(bin_path), 'config', 'sdkconfig.json')
    try:
        with open(cfg) as f:
            return bool(json.load(f).get(key))
    except (OSError, ValueError):
        return None


def build_has_networking(bin_path):
    """True/False if the build that produced bin_path has CONFIG_FUGU_WITH_NETW, None if unknown."""
    return _build_flag(bin_path, 'FUGU_WITH_NETW')


def build_is_plant_sim(bin_path):
    """True/False if the build that produced bin_path has CONFIG_FUGU_WITH_VCONV, None if unknown.
    With VCONV the real LEDC PwmDriver is swapped for a plant simulator (src/buck.h), so the
    half-bridge never switches: flashing it to a real converter yields 0W while the device still
    samples real sensors and looks alive."""
    return _build_flag(bin_path, 'FUGU_WITH_VCONV')


def build_gate_driver(bin_path):
    """Gate driver the build that produced bin_path compiles in: 'mcpwm', 'ledc' or 'vconv' (Kconfig
    choice FUGU_GATE_DRIVER), None if the config can't be read or names none of them."""
    for key, name in (('FUGU_WITH_MCPWM', 'mcpwm'), ('FUGU_GATE_LEDC', 'ledc'), ('FUGU_WITH_VCONV', 'vconv')):
        v = _build_flag(bin_path, key)
        if v is None:
            return None
        if v:
            return name
    return None


RE_PWM_FREQ_MCPWM = re.compile(r'pwm-freq [0-9.]+ Hz period_ticks=\d+')


def parse_device_gate_driver(lines, ok, rejected):
    """Running gate driver from a no-arg `pwm-freq` reply: 'mcpwm' (reports its timer period),
    'other' (LEDC or VCONV, which answer 'n/a'), None if the reply proves neither (timeout, unknown
    command on old firmware, garbled) — callers must treat None as unverified, never as a match."""
    if ok and any(RE_PWM_FREQ_MCPWM.search(l) for l in lines):
        return 'mcpwm'
    if rejected and any('pwm-freq: n/a' in l for l in lines):
        return 'other'
    return None


def gate_driver_verdict(image, device):
    """'ok', 'mismatch' or 'unverified' for pushing an image with gate driver `image` (see
    build_gate_driver) to a device running `device` (see parse_device_gate_driver)."""
    if image is None or device is None:
        return 'unverified'
    if (image == 'mcpwm') == (device == 'mcpwm'):
        return 'ok'
    return 'mismatch'
