#!/usr/bin/env python3
"""Launch the adcscope client against fugu boards.

adcscope (etc/adcscope, github.com/fl4p/adcscope) is device-agnostic: it discovers by mDNS and
nothing else. The lab's NAT-routed boards (fry/flat) aren't mDNS-reachable, so this wrapper
registers adcscope's discovery hooks with the fugu adapter before handing over — pointing it at
this repo's `etc/nat.env` and putting `etc/` on the path so the adapter can reach
`fugu_console.probe_welcome` for hostname probing.

    ./etc/scope.py            # discover (mDNS + nat.env), pick in the UI
    ./etc/scope.py -m fry     # auto-pick a board by hostname substring

Arguments are passed through to adcscope unchanged; see `./etc/scope.py --help`.
"""
import os
import runpy
import sys

_ETC = os.path.dirname(os.path.abspath(__file__))
_ADCSCOPE = os.path.join(_ETC, "adcscope")

if not os.path.isdir(_ADCSCOPE):
    sys.exit("etc/adcscope is empty — run: git submodule update --init etc/adcscope")

sys.path[:0] = [_ADCSCOPE, _ETC]          # adcscope modules, then fugu_console for banner probing

import contrib.fugu_nat as fugu_nat        # noqa: E402  registers both discovery hooks

fugu_nat.NAT_ENV = os.path.join(_ETC, "nat.env")   # adapter defaults to nat.env beside itself

runpy.run_path(os.path.join(_ADCSCOPE, "adcscope.py"), run_name="__main__")
