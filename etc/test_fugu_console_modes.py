#!/usr/bin/env python3
"""Host unit tests for fugu_console: command-mode resolution + the prompt-safe log printer.

Pure logic, no device — run with plain `python3 etc/test_fugu_console_modes.py`. Stubs the optional
`serial` dep so the module imports in a bare environment (the transport import is unconditional).
"""
import io
import os
import sys
import time
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.modules.setdefault("serial", types.ModuleType("serial"))  # pyserial may be absent here

from fugu_console import (resolve_command_mode as r, _PromptSafePrinter,  # noqa: E402
                          _ble_target_name, parse_args)

# (label, args, expected) — args: explicit_cmds, use_stdin, want_coredump, stdin_is_tty
#                           expected: (read_stdin, active, delimit)
CASES = [
    ("single -c (tty)",        ([" mem "], False, False, True),  (False, True, False)),
    ("single -c (piped)",      (["mem"],   False, False, False), (False, True, False)),
    ("multi -c",               (["a", "b"], False, False, True), (False, True, True)),
    ("--stdin (tty)",          ([],        True,  False, True),  (True,  True, True)),
    ("auto-batch (piped)",     ([],        False, False, False), (True,  True, True)),
    ("REPL (tty, no mode)",    ([],        False, False, True),  (False, False, False)),
    ("--coredump wins/pipe",   ([],        False, True,  False), (False, False, False)),
    ("-c + pipe stays single", (["x"],     False, False, False), (False, True, False)),
    ("--stdin + -c combine",   (["x"],     True,  False, True),  (True,  True, True)),
]

BLE_CASES = [
    ("bare --ble", ["--ble"], True, "fugu"),
    ("--ble NAME", ["--ble", "fry"], "fry", "fry"),
    ("legacy --ble --name", ["--ble", "--name", "flat"], True, "flat"),
]


def _render(line, *, active, istty, buf, last_command):
    """Drive _PromptSafePrinter once with a stubbed libedit and return what it wrote to stdout."""
    p = _PromptSafePrinter()
    p.prompt = "fry> "
    p.active = active
    p.last_command = last_command
    p._istty = istty
    fake_readline = types.ModuleType("readline")
    setattr(fake_readline, "get_line_buffer", lambda: buf)
    saved_rl, sys.modules["readline"] = sys.modules.get("readline"), fake_readline
    cap, saved_out = io.StringIO(), sys.stdout
    sys.stdout = cap
    try:
        p(line)
        if active and istty:
            time.sleep(p.REDRAW_IDLE + 0.05)
    finally:
        sys.stdout = saved_out
        if saved_rl is None:
            sys.modules.pop("readline", None)
        else:
            sys.modules["readline"] = saved_rl
    return cap.getvalue()


# (label, kwargs, expected stdout) — the libedit fix: a buffer still equal to the last accepted
# command is stale (idle prompt), so the redraw must NOT echo it; fresh typing must survive.
PRINTER_CASES = [
    ("stale buffer suppressed",
     dict(active=True, istty=True, buf="dc 500", last_command="dc 500"), "\r\x1b[KV=1\n\r\x1b[Kfry> "),
    ("fresh typing preserved",
     dict(active=True, istty=True, buf="dc 6", last_command="dc 500"), "\r\x1b[KV=1\n\r\x1b[Kfry> dc 6"),
    ("idle empty prompt",
     dict(active=True, istty=True, buf="", last_command=""), "\r\x1b[KV=1\n\r\x1b[Kfry> "),
    ("not a tty -> plain",
     dict(active=True, istty=False, buf="dc 500", last_command="dc 500"), "V=1\n"),
    ("inactive -> plain",
     dict(active=False, istty=True, buf="dc 500", last_command="dc 500"), "V=1\n"),
]


def main():
    fails = 0
    for label, args, expected in CASES:
        got = r(*args)
        ok = got == expected
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + ("" if ok else f"  got {got} != {expected}"))
        fails += not ok
    for label, argv, expected_ble, expected_name in BLE_CASES:
        args = parse_args(argv)
        got = (args.ble, _ble_target_name(args))
        expected = (expected_ble, expected_name)
        ok = got == expected
        print(f"  [{'PASS' if ok else 'FAIL'}] parser: {label}"
              + ("" if ok else f"  got {got!r} != {expected!r}"))
        fails += not ok
    for label, kw, expected in PRINTER_CASES:
        got = _render("V=1", **kw)
        ok = got == expected
        print(f"  [{'PASS' if ok else 'FAIL'}] printer: {label}"
              + ("" if ok else f"  got {got!r} != {expected!r}"))
        fails += not ok
    total = len(CASES) + len(BLE_CASES) + len(PRINTER_CASES)
    print(f"\n{total - fails}/{total} passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
