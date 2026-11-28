#!/usr/bin/env python3
"""Host unit tests for ota_common.py's build-flag guards (plant-sim / networking detection).

Pure logic, no device or network — run with plain `python3 etc/test_ota_build_flags.py`.
Guards the OTA pusher from flashing a bench VCONV (plant-sim) image to a real converter.
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ota_common import (build_is_plant_sim, build_has_networking, build_gate_driver,  # noqa: E402
                        parse_device_gate_driver, gate_driver_verdict, write_build_config_sidecar)


def _bin_with_config(tmp, cfg):
    """Lay out <tmp>/build/{fugu-firmware.bin, config/sdkconfig.json=cfg}; return the bin path.
    cfg is a dict to write as sdkconfig.json, the string 'corrupt' for invalid JSON, or None to
    omit the file entirely (missing config)."""
    cfgdir = os.path.join(tmp, 'build', 'config')
    os.makedirs(cfgdir, exist_ok=True)
    binp = os.path.join(tmp, 'build', 'fugu-firmware.bin')
    open(binp, 'wb').close()
    if cfg == 'corrupt':
        with open(os.path.join(cfgdir, 'sdkconfig.json'), 'w') as f:
            f.write('{not valid json')
    elif cfg is not None:
        with open(os.path.join(cfgdir, 'sdkconfig.json'), 'w') as f:
            json.dump(cfg, f)
    return binp


# (label, cfg, want_sim, want_netw) — None means 'unknown', which callers must treat as not-safe.
CASES = [
    ("vconv bench build",  {"FUGU_WITH_VCONV": True,  "FUGU_WITH_NETW": True},  True,  True),
    ("production build",   {"FUGU_WITH_VCONV": False, "FUGU_WITH_NETW": True},  False, True),
    ("no-network build",   {"FUGU_WITH_VCONV": False, "FUGU_WITH_NETW": False}, False, False),
    ("vconv key absent",   {"FUGU_WITH_NETW": True},                            False, True),
    ("missing sdkconfig",  None,                                                None,  None),
    ("corrupt sdkconfig",  'corrupt',                                           None,  None),
]


# (label, cfg, want_driver) for build_gate_driver
DRIVER_CASES = [
    ("mcpwm build",        {"FUGU_WITH_MCPWM": True, "FUGU_GATE_LEDC": False, "FUGU_WITH_VCONV": False}, 'mcpwm'),
    ("ledc build",         {"FUGU_WITH_MCPWM": False, "FUGU_GATE_LEDC": True, "FUGU_WITH_VCONV": False}, 'ledc'),
    ("vconv build",        {"FUGU_WITH_MCPWM": False, "FUGU_GATE_LEDC": False, "FUGU_WITH_VCONV": True}, 'vconv'),
    ("no driver key set",  {"FUGU_WITH_NETW": True},                                                      None),
    ("missing sdkconfig",  None,                                                                          None),
]

MCPWM_LINE = 'pwm-freq 38995.86 Hz period_ticks=4103 res=160000000 pwmMax=4089 hs_off=2499 maxHS=3758 nominal=39000'
# (label, lines, ok, rejected, want) for parse_device_gate_driver
REPLY_CASES = [
    ("mcpwm reply",          [MCPWM_LINE],                                                   True,  False, 'mcpwm'),
    ("mcpwm amid status",    ['Vin=40.1 Vout=27.3 P=120', MCPWM_LINE],                       True,  False, 'mcpwm'),
    ("ledc/vconv n/a",       ['W (1) main: pwm-freq: n/a, needs the MCPWM gate driver build'], False, True,  'other'),
    ("old fw n/a wording",   ['pwm-freq: n/a, needs the MCPWM driver (converter.conf::pwm_driver)'], False, True, 'other'),
    ("timeout",              [],                                                             False, False, None),
    ("unknown command",      ['E (1) main: Unknown command pwm-freq'],                       False, True,  None),
    ("ok without the line",  ['some status line'],                                           True,  False, None),
]

# (image, device, want) for gate_driver_verdict
VERDICT_CASES = [
    ('mcpwm', 'mcpwm', 'ok'), ('ledc', 'other', 'ok'), ('vconv', 'other', 'ok'),
    ('mcpwm', 'other', 'mismatch'), ('ledc', 'mcpwm', 'mismatch'), ('vconv', 'mcpwm', 'mismatch'),
    (None, 'mcpwm', 'unverified'), ('mcpwm', None, 'unverified'), (None, None, 'unverified'),
]


MCPWM_CFG = {"FUGU_WITH_MCPWM": True, "FUGU_GATE_LEDC": False, "FUGU_WITH_VCONV": False}


def _sidecar_case(tmp, how):
    """A copied image in <tmp>/keep/ without its build dir; how: 'match' (sidecar of this image),
    'other' (sidecar written for different bytes), 'none' (no sidecar), 'corrupt', 'precedence'
    (sidecar plus another build's config/sdkconfig.json in the same dir)."""
    src = _bin_with_config(os.path.join(tmp, 'src'), MCPWM_CFG)
    with open(src, 'wb') as f:
        f.write(b'image-A')
    keep = os.path.join(tmp, 'keep')
    os.makedirs(keep)
    binp = os.path.join(keep, 'fugu-firmware.bin')
    cfg = os.path.join(tmp, 'src', 'build', 'config', 'sdkconfig.json')
    with open(binp, 'wb') as f:
        f.write(b'image-A')
    if how in ('match', 'other'):
        write_build_config_sidecar(binp, cfg)
    if how == 'other':
        with open(binp, 'wb') as f:
            f.write(b'image-B')
    if how == 'precedence':  # an unbound config of another (LEDC) build sits next to the copied bin
        write_build_config_sidecar(binp, cfg)
        os.makedirs(os.path.join(keep, 'config'))
        with open(os.path.join(keep, 'config', 'sdkconfig.json'), 'w') as f:
            json.dump({"FUGU_WITH_MCPWM": False, "FUGU_GATE_LEDC": True}, f)
    if how == 'corrupt':
        with open(binp[:-4] + '.sdkconfig.json', 'w') as f:
            f.write('{"bin_sha256": ')
    return binp


SIDECAR_CASES = [('match', 'mcpwm'), ('precedence', 'mcpwm'), ('other', None), ('none', None), ('corrupt', None)]


def main():
    fails = 0
    for how, want in SIDECAR_CASES:
        with tempfile.TemporaryDirectory() as tmp:
            binp = _sidecar_case(tmp, how)
            got = build_gate_driver(binp)
            got_sim = build_is_plant_sim(binp)
        ok = got == want and got_sim == (None if want is None else False)
        fails += not ok
        print(f"{'PASS' if ok else 'FAIL'}  sidecar {how}: driver={got} sim={got_sim}" + ("" if ok else f"  (want {want})"))
    for label, cfg, want in DRIVER_CASES:
        with tempfile.TemporaryDirectory() as tmp:
            got = build_gate_driver(_bin_with_config(tmp, cfg))
        ok = got == want
        fails += not ok
        print(f"{'PASS' if ok else 'FAIL'}  driver {label}: {got}" + ("" if ok else f"  (want {want})"))
    for label, lines, rok, rrej, want in REPLY_CASES:
        got = parse_device_gate_driver(lines, rok, rrej)
        ok = got == want
        fails += not ok
        print(f"{'PASS' if ok else 'FAIL'}  reply {label}: {got}" + ("" if ok else f"  (want {want})"))
    for img, dev, want in VERDICT_CASES:
        got = gate_driver_verdict(img, dev)
        ok = got == want
        fails += not ok
        print(f"{'PASS' if ok else 'FAIL'}  verdict {img}->{dev}: {got}" + ("" if ok else f"  (want {want})"))
    for label, cfg, want_sim, want_netw in CASES:
        with tempfile.TemporaryDirectory() as tmp:
            binp = _bin_with_config(tmp, cfg)
            got_sim = build_is_plant_sim(binp)
            got_netw = build_has_networking(binp)
        ok = got_sim == want_sim and got_netw == want_netw
        fails += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {label}: sim={got_sim} netw={got_netw}"
              + ("" if ok else f"  (want sim={want_sim} netw={want_netw})"))
    print(f"\n{'ALL PASS' if not fails else f'{fails} FAILED'}")
    return fails


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
