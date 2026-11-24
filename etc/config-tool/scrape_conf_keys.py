#!/usr/bin/env python3
"""
Scrape `ConfFile::get*(key, default)` calls from src/ and main/, then regenerate
the auto-managed tables in etc/config-tool/conf-editor.html:

    - FILE_KEYS  (which keys the firmware reads per .conf file)
    - TYPE_KEYS  (which `get<Type>()` is used per key)
    - DEFAULTS   (hard-coded second-arg default values)

The blocks are bracketed by marker comments:

    // SCRAPER:BEGIN <NAME>
    ...
    // SCRAPER:END <NAME>

Each getter's receiver is resolved to its .conf file from the code: the path in
its `ConfFile{...}` / `loadConfSafe(...)` declaration or temporary, the
service's own conf for `_confPath`, or, for a `const ConfFile &` parameter, what
the function's callers pass. A key held in a helper parameter is followed to the
callers' literals. Whatever stays unresolved is an error (exit 2), never skipped.

The hand-curated META / FILE_META / CHAN_* tables and human descriptions are
*not* touched. The script prints which keys have no META entry and which META
entries point at keys the firmware no longer reads.

Usage:
    python3 etc/config-tool/scrape_conf_keys.py               # report drift
    python3 etc/config-tool/scrape_conf_keys.py --check       # exit 1 on drift
    python3 etc/config-tool/scrape_conf_keys.py --write       # rewrite html
    python3 etc/config-tool/scrape_conf_keys.py --write --html /tmp/copy.html
"""
from __future__ import annotations
import argparse
import ast
import bisect
import functools
import json
import operator as op
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

# ----------------------------------------------------------------------------
# Hand mappings. Each entry must be used by the scan; an unused one is an error
# (it went stale when the code moved).
# ----------------------------------------------------------------------------

# Conventional names of a `const ConfFile &` parameter whose callers the scraper
# cannot trace (virtual AsyncADC::init(boardConf) called through a base pointer).
# Only consulted when caller tracing finds no call site.
VAR_TO_CONF = {
    'boardConf':     'board.conf',
    'sensConf':      'sensor.conf',
    'limits':        'limits.conf',
    'limitsConf':    'limits.conf',
    'coilConf':      'coil.conf',
    'converterConf': 'converter.conf',
    'chargerConf':   'charger.conf',
    'trackerConf':   'tracker.conf',
    'teleConf':      'tele.conf',
    'mqttConf':      'mqtt.conf',
    'pprofConf':     'pprof.conf',
}

# Per-source overrides {rel: {var: conf}}; win over everything else. Use only
# where the code cannot be resolved; say why.
FILE_VAR_OVERRIDES: dict[str, dict[str, str]] = {}

# .conf files read by firmware that the editor does not expose. Their keys are
# resolved (so they cannot hide an unresolved variable) and then left out.
IGNORED_CONFS: set[str] = set()

# An anonymous lambda's ConfFile parameter, keyed by the call the lambda is passed
# to (`callee(first arg`): {rel: {call: conf}}.
LAMBDA_ARG_CONFS = {
    # confCheck() re-runs the loaders on the files it is given, and its only caller passes the
    # literal charger.conf / limits.conf paths (src/cli.cpp, `conf-check`). Following that path
    # through the generic `check` lambda is beyond the scraper.
    'src/cli.cpp': {'check(chargerPath': 'charger.conf', 'check(limitsPath': 'limits.conf'},
}

# Getter calls whose key is computed at runtime and cannot be followed to a
# literal. Each must be acknowledged here: {(rel, key expression): why}. The keys
# they read are pattern-based in the editor, listed in INDIRECT_KEYS, or covered
# by a dedicated resolver (scan_pd_gains).
_CHAN = 'sensor channel <hv_v|lv_v|hv_i|lv_i|ntc>_<suffix>: pattern-based in the editor (CHAN_RE)'
DYNAMIC_KEY_SITES = {
    **{('src/adc/sensor_setup.cpp', f'key + \'_\' + "{sfx}"'): _CHAN
       for sfx in ('ch', 'adc', 'rh', 'rl', 'gain', 'offset', 'factor', 'midpoint', 'filt_len')},
    ('src/adc/sensor_setup.cpp', 'k'): 'migration guard: throws on board.conf ina22x_* leftovers (now sensor.conf)',
    ('src/conv_side.h', 'key'): 'limits.conf side keys from limitSideKey(): INDIRECT_KEYS',
    ('src/pd_control.h', 'key'): 'pdReadGain(conf, "ctrl_<name>_kp|kd"): scan_pd_gains',
    ('src/pd_control.h', 'p + "td"'): 'ctrl_<name>_td: scan_pd_gains',
    ('src/cli.cpp', 'k'): 'pin diagnostics: is a pin one of the board.conf pin keys (read elsewhere)',
    ('src/cli.cpp', 'key.c_str()'): 'set-config / get-config: any key of any file',
    ('src/cli.cpp', 'key'): 'get-config: any key of any file',
    ('src/tele/telemetry.cpp', 'k'): 'wifi.conf ssid_* entries: pattern-based in the editor',
    ('src/tele/telemetry.cpp', 'k + "_psk"'): 'wifi.conf ssid_*_psk: pattern-based',
    ('src/tele/telemetry.cpp', 'label'): 'add_ap() probing for a free ssid_* label: pattern-based',
}


# Reads of a removed key that only exist to reject a leftover: {(rel, key): why}.
MIGRATION_GUARD_READS = {
    ('src/adc/temperature.h', 'ntc_ch'): 'board.conf ntc_ch moved to sensor.conf; setup throws if board.conf still sets it',
    ('src/buck.h', 'pwm_driver'): 'converter.conf pwm_driver is a build-time choice now; only warned about when set',
}


@dataclass
class Indirect:
    """Keys read through a helper the scraper cannot follow, with evidence that must still match."""
    conf: str
    keys: list
    jtype: str
    evidence: tuple  # (rel path, regex); the scan fails when it no longer matches
    why: str
    defaults: dict = field(default_factory=dict)


INDIRECT_KEYS = [
    Indirect('limits.conf', ['hv_max', 'lv_max', 'hv_i_max', 'lv_i_max'], 'float',
             ('src/conv_side.h', r'return std::string\(sideOf\(isInputRole\(roleKey\), boost\)\) \+ '
                                 r'\(roleKey\[0\] == \'i\' \? "_i_max" : "_max"\)'),
             'Limits() reads them via readMappedLimit()/limitSideKey(), side chosen by topo'),
]

# META keys the firmware does not read by name but the editor still describes on
# purpose. A META key outside this set that no getter reads is reported stale.
META_ONLY_OK: dict[str, str] = {}

# ----------------------------------------------------------------------------
# Source model
# ----------------------------------------------------------------------------
GETTERS = {'getByte': 'byte', 'getLong': 'long', 'getFloat': 'float',
           'getString': 'string', 'f': 'float', 'c': 'string'}
GETTER_RE = re.compile(r'(?:\.|->)\s*(?P<getter>getByte|getLong|getFloat|getString|f|c)\s*\(')
IDENT = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')
LIT_RE = re.compile(r'^"((?:[^"\\]|\\.)*)"$')
CONF_PATH_RE = re.compile(r'^"/littlefs/conf/([A-Za-z0-9_.-]+\.conf)"$')
KEY_RE = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')
CONTROL_KW = {'if', 'for', 'while', 'switch', 'catch', 'return', 'sizeof', 'decltype', 'alignof'}
TAIL_KW = {'const', 'override', 'noexcept', 'mutable', 'final', 'volatile'}
MAX_DEPTH = 6

SERVICE_ALL = '*service*'  # sentinel: the Service base class, i.e. every service conf


class Unresolved(Exception):
    pass


class NotConfFile(Unresolved):
    """The name is declared, with a type other than ConfFile (another overload's argument)."""


CONF_DECL_ARG = re.compile(r'\bConfFile\s*(?:&|\*|\s[A-Za-z_])')
DECL_ARG = re.compile(r'^(?:const\s+)?[A-Za-z_][\w:<>]*\s*[&*]*\s+[&*]*\s*[A-Za-z_]\w*(?:\s*=.*)?$|'
                      r'^(?:const\s+)?[A-Za-z_][\w:<>]*\s*[&*]+\s*[A-Za-z_]\w*(?:\s*=.*)?$')


def strip_comments(src: str):
    """Return (text, mask), both len(src). text has comments blanked; mask also blanks literal contents."""
    text, mask = list(src), list(src)
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if c == '/' and i + 1 < n and src[i + 1] == '/':
            j = src.find('\n', i)
            j = n if j < 0 else j
            for k in range(i, j):
                text[k] = mask[k] = ' '
            i = j
        elif c == '/' and i + 1 < n and src[i + 1] == '*':
            j = src.find('*/', i + 2)
            j = n if j < 0 else j + 2
            for k in range(i, j):
                if src[k] != '\n':
                    text[k] = mask[k] = ' '
            i = j
        elif c == 'R' and src.startswith('R"', i) and (i == 0 or not (src[i - 1].isalnum() or src[i - 1] == '_')):
            m = re.match(r'R"([^(\s]*)\(', src[i:])
            if not m:
                i += 1
                continue
            end = src.find(')' + m.group(1) + '"', i)
            end = n if end < 0 else end + len(m.group(1)) + 2
            for k in range(i + 2, end - 1):
                if src[k] != '\n':
                    mask[k] = ' '
            i = end
        elif c in '"\'':
            if c == "'" and i > 0 and (src[i - 1].isalnum()):  # digit separator 1'000
                i += 1
                continue
            j = i + 1
            while j < n and src[j] != c and src[j] != '\n':
                j += 2 if src[j] == '\\' else 1
            for k in range(i + 1, min(j, n)):
                mask[k] = ' '
            i = j + 1
        else:
            i += 1
    return ''.join(text), ''.join(mask)


class Source:
    def __init__(self, rel: str, raw: str):
        self.rel = rel
        self.text, self.mask = strip_comments(raw)
        self.match = {}
        self.parent = {}      # block open -> enclosing block open (-1 = file)
        self.opens = []       # block open positions, ascending
        stack = []
        for i, c in enumerate(self.mask):
            if c in '({[':
                stack.append(i)
            elif c in ')}]':
                if stack:
                    o = stack.pop()
                    self.match[o] = i
                    self.match[i] = o
        bstack = []
        for i, c in enumerate(self.mask):
            if c == '{' and i in self.match:
                self.parent[i] = bstack[-1] if bstack else -1
                self.opens.append(i)
                bstack.append(i)
            elif c == '}' and i in self.match and bstack and self.match[i] == bstack[-1]:
                bstack.pop()
        self._lines = [0] + [i + 1 for i, c in enumerate(self.text) if c == '\n']

    def line(self, pos: int) -> int:
        lo, hi = 0, len(self._lines)
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self._lines[mid] <= pos:
                lo = mid
            else:
                hi = mid
        return lo + 1

    def where(self, pos: int) -> str:
        return f'{self.rel}:{self.line(pos)}'

    def block_of(self, pos: int) -> int:
        """Innermost `{` block containing pos (-1 = file scope)."""
        i = bisect.bisect_left(self.opens, pos) - 1
        if i < 0:
            return -1
        b = self.opens[i]
        while b != -1 and self.match[b] <= pos:
            b = self.parent[b]
        return b

    def enclosing_blocks(self, pos: int):
        return self._chain(self.block_of(pos))

    def _chain(self, b: int):
        while b != -1:
            yield b
            b = self.parent[b]

    def skip_ws_back(self, i: int) -> int:
        while i >= 0 and self.mask[i].isspace():
            i -= 1
        return i

    def skip_ws(self, i: int) -> int:
        while i < len(self.mask) and self.mask[i].isspace():
            i += 1
        return i

    def ident_before(self, i: int):
        """(name, start) of the identifier ending at or before i (after skipping ws), else (None, i)."""
        i = self.skip_ws_back(i)
        j = i
        while j >= 0 and (self.mask[j].isalnum() or self.mask[j] == '_'):
            j -= 1
        if j == i:
            return None, i
        return self.mask[j + 1:i + 1], j + 1

    def args(self, open_pos: int):
        """[(text, start, end)] of the top-level comma-separated args of the group at open_pos."""
        close = self.match.get(open_pos)
        if close is None:
            return []
        out, start, depth = [], open_pos + 1, 0
        for i in range(open_pos + 1, close):
            c = self.mask[i]
            if c in '({[':
                depth += 1
            elif c in ')}]':
                depth -= 1
            elif c == ',' and depth == 0:
                out.append((self.text[start:i].strip(), start, i))
                start = i + 1
        last = self.text[start:close].strip()
        if last or out:
            out.append((last, start, close))
        return out

    @functools.lru_cache(maxsize=None)
    def classify_block(self, o: int):
        """('func', name, class, params_open) | ('class', name) | ('other',) for the block opening at o."""
        i = self.skip_ws_back(o - 1)
        head = self.mask[max(0, i - 400):i + 1]
        cm = re.search(r'\b(class|struct)\s+([A-Za-z_]\w*)[^;{}()]*$', head)
        if cm and '=' not in head[cm.end(2):]:
            return ('class', cm.group(2))
        while True:   # skip trailing qualifiers / trailing return type
            name, s = self.ident_before(i)
            if name in TAIL_KW:
                i = self.skip_ws_back(s - 1)
                continue
            if name:
                k = s
                while True:                                   # ns::Type
                    q = self.skip_ws_back(k - 1)
                    if q >= 1 and self.mask[q] == ':' and self.mask[q - 1] == ':':
                        _, k = self.ident_before(q - 2)
                        continue
                    break
                q = self.skip_ws_back(k - 1)
                if q >= 1 and self.mask[q] == '>' and self.mask[q - 1] == '-':
                    i = self.skip_ws_back(q - 2)
                    continue
            break
        if i < 0 or self.mask[i] != ')':
            return ('other',)
        popen = self.match.get(i)
        if popen is None:
            return ('other',)
        # constructor init list: walk `: a(x), b{y}` back to the parameter list
        while True:
            nm, s = self.ident_before(popen - 1)
            if nm is None:
                break
            k = self.skip_ws_back(s - 1)
            if k >= 0 and self.mask[k] == ',' :
                p = self.skip_ws_back(k - 1)
                if self.mask[p] in ')}' and p in self.match:
                    popen = self.match[p]
                    continue
            if k >= 0 and self.mask[k] == ':' and not (k > 0 and self.mask[k - 1] == ':'):
                p = self.skip_ws_back(k - 1)
                if self.mask[p] == ')' and p in self.match:
                    popen = self.match[p]
                    continue
            break
        k = self.skip_ws_back(popen - 1)
        if k >= 0 and self.mask[k] == ']':     # lambda: name is the variable it is assigned to
            lo = self.match.get(k, k)
            e = self.skip_ws_back(lo - 1)
            lname = None
            if e >= 0 and self.mask[e] == '=' and self.mask[e - 1] not in '=!<>':
                lname, _ = self.ident_before(e - 1)
            return ('func', lname, None, popen)
        name, s = self.ident_before(popen - 1)
        if name is None:
            om = re.search(r'\boperator\s*[^\s\w()]+\s*$', self.mask[max(0, popen - 40):popen])
            if om:
                name, s = 'operator', popen - 40 + om.start() if popen >= 40 else om.start()
        if name is None or name in CONTROL_KW:
            return ('other',)
        cls = None
        q = self.skip_ws_back(s - 1)
        if q >= 1 and self.mask[q] == ':' and self.mask[q - 1] == ':':
            cls, _ = self.ident_before(q - 2)
        else:
            for b in self.enclosing_blocks(o):
                kind = self.classify_block(b)
                if kind[0] == 'class':
                    cls = kind[1]
                    break
        return ('func', name, cls, popen)

    def enclosing_class(self, pos: int):
        return self._class_of_block(self.block_of(pos))

    @functools.lru_cache(maxsize=None)
    def _class_of_block(self, b0: int):
        for b in self._chain(b0):
            kind = self.classify_block(b)
            if kind[0] == 'class':
                return kind[1]
            if kind[0] == 'func' and kind[2]:
                return kind[2]
        return None


# ----------------------------------------------------------------------------
# Resolution
# ----------------------------------------------------------------------------
def arity(src: Source, popen: int):
    """(min, max) argument count of the parameter list at popen."""
    params = [t for t, _, _ in src.args(popen) if t and t != 'void']
    return sum('=' not in t for t in params), len(params)


class Repo:
    def __init__(self, sources: dict):
        self.sources = sources
        self.service_conf = {}     # class name -> conf basename
        for src in sources.values():
            for m in re.finditer(r'\b(\w+)\s*\(\s*\)\s*:\s*Service\s*\(\s*"[^"]*"\s*,\s*'
                                 r'"/littlefs/conf/([A-Za-z0-9_.-]+\.conf)"', src.text):
                self.service_conf[m.group(1)] = m.group(2)
        self.used_overrides = set()
        self._trace_memo = {}

    # -- receiver ------------------------------------------------------------
    def conf_of_path_expr(self, src: Source, expr: str, pos: int, depth: int):
        expr = expr.strip()
        m = CONF_PATH_RE.match(expr)
        if m:
            return {m.group(1)}
        if expr == '_confPath':
            cls = src.enclosing_class(pos)
            if cls == 'Service':
                return {SERVICE_ALL}
            if cls in self.service_conf:
                return {self.service_conf[cls]}
            raise Unresolved(f'_confPath in class {cls!r}, which has no `: Service("...", "/littlefs/conf/X.conf")` '
                             f'constructor')
        if IDENT.fullmatch(expr):
            lits = self.literal_values(src, expr, pos)
            if lits:
                out = set()
                for lit in lits:
                    m = CONF_PATH_RE.match(f'"{lit}"')
                    if not m:
                        raise Unresolved(f'{expr!r} = "{lit}" is not a /littlefs/conf/<file> path')
                    out.add(m.group(1))
                return out
        raise Unresolved(f'conf path {expr!r} is not a literal /littlefs/conf/<file>')

    def conf_of_expr(self, src: Source, start: int, end: int, depth: int):
        """Resolve the expression text[start:end] that yields a ConfFile."""
        seg = src.mask[start:end]
        s = start + len(seg) - len(seg.lstrip())
        e = start + len(seg.rstrip()) - 1
        if s > e:
            raise Unresolved('empty expression')
        if src.mask[e] in ')}' and e in src.match:
            o = src.match[e]
            callee, _ = src.ident_before(o - 1)
            if callee in ('ConfFile', 'loadConfSafe'):
                a = src.args(o)
                if not a:
                    raise Unresolved(f'{callee}{{}} without a path')
                return self.conf_of_path_expr(src, a[0][0], o, depth)
            raise Unresolved(f'ConfFile from {callee}(...)')
        name, ns = src.ident_before(e)
        if name and ns == s:
            return self.resolve_var(src, name, s, depth)
        raise Unresolved(f'expression {src.text[s:e + 1]!r}')

    # -- scopes ----------------------------------------------------------------
    def scopes(self, src: Source, pos: int):
        """Enclosing scopes of pos, innermost first: (block open or -1, function kind or None).

        A position in a constructor's member-init list belongs to that constructor's parameters.
        """
        b0 = src.block_of(pos)
        i = bisect.bisect_right(src.opens, pos)
        while i < len(src.opens):
            o = src.opens[i]
            if src.parent[o] != b0 or ';' in src.mask[pos:o]:
                break
            kind = src.classify_block(o)
            if kind[0] == 'func':
                if src.match[kind[3]] < pos:
                    yield None, kind
                break
            i = bisect.bisect_right(src.opens, src.match[o])
        for b in src.enclosing_blocks(pos):
            kind = src.classify_block(b)
            yield b, kind
        yield -1, ('other',)

    @staticmethod
    def find_param(src: Source, popen: int, name: str):
        for idx, (txt, a, b) in enumerate(src.args(popen)):
            core = txt.split('=')[0].strip()
            m = re.search(r'([A-Za-z_]\w*)\s*(\[[^\]]*\])?$', core)
            if m and m.group(1) == name and core != name:
                return idx, core
        return None

    def resolve_var(self, src: Source, name: str, pos: int, depth: int):
        if depth > MAX_DEPTH:
            raise Unresolved(f'{name!r}: resolution too deep')
        ov = FILE_VAR_OVERRIDES.get(src.rel, {})
        if name in ov:
            self.used_overrides.add((src.rel, name))
            return {ov[name]}
        for b, kind in self.scopes(src, pos):
            if b is not None:
                lo = b + 1
                hi = pos if b == -1 else min(pos, src.match[b])
                decl = None
                for m in re.finditer(r'\bConfFile\s*(&\s*)?' + re.escape(name) + r'\s*(?P<init>[{(=;])',
                                     src.mask[lo:hi]):
                    p = lo + m.start()
                    if src.block_of(p) == b and not self._in_paren(src, p, b):
                        decl = (p, lo + m.start('init'))
                if decl:
                    return self._conf_of_decl(src, name, decl, pos, depth)
            if kind[0] == 'class':
                continue
            if kind[0] == 'func':
                hit = self.find_param(src, kind[3], name)
                if hit:
                    idx, core = hit
                    if 'ConfFile' not in core:
                        raise NotConfFile(f'{name!r} is a parameter of type {core!r}, not ConfFile')
                    return self.trace_param(src, kind, idx, name, depth)
        raise Unresolved(f'no ConfFile declaration of {name!r} in scope')

    @staticmethod
    def _in_paren(src: Source, p: int, b: int) -> bool:
        """True if p sits inside a (...) group within block b, i.e. it is a parameter, not a local."""
        depth = 0
        for i in range(p - 1, b, -1):
            c = src.mask[i]
            if c == ')':
                depth += 1
            elif c == '(':
                if depth == 0:
                    return True
                depth -= 1
            elif c in ';{}' and depth == 0:
                return False
        return False

    def _conf_of_decl(self, src, name, decl, pos, depth):
        p, ip = decl
        c = src.mask[ip]
        if c in '{(' and ip in src.match:
            a = src.args(ip)
            if a:
                return self.conf_of_path_expr(src, a[0][0], ip, depth)
        elif c == '=':
            end = src.mask.find(';', ip)
            return self.conf_of_expr(src, ip + 1, end, depth)
        # declared without a path: take the assignments between declaration and use
        found = set()
        for m in re.finditer(r'(?<![\w.>])' + re.escape(name) + r'\s*=(?!=)', src.mask[p:pos]):
            ap = p + m.end()
            found |= self.conf_of_expr(src, ap, src.mask.find(';', ap), depth + 1)
        if found:
            return found
        raise Unresolved(f'{name!r} declared without a path at {src.where(p)} and never assigned one')

    def literal_values(self, src: Source, name: str, pos: int):
        """String literals a local `name` can hold, else None.

        The declaration must be `"lit"` or `cond ? "a" : "b"`, every later plain assignment in its
        scope too, and nothing in the scope may mutate it (+=, ++, .append(), &name, ...)."""
        lit_shape = re.compile(r'L|[^"?:]*\?\s*L\s*:\s*L')
        occ = re.compile(r'(?<![\w.>])' + re.escape(name) + r'(?!\w)')

        def literals(vs, ve):
            expr = src.text[vs:ve].strip()
            lits = re.findall(r'"((?:[^"\\]|\\.)*)"', expr)
            shape = re.sub(r'"(?:[^"\\]|\\.)*"', 'L', expr)
            return lits if lits and lit_shape.fullmatch(shape) else None

        for b, kind in self.scopes(src, pos):
            if b is None or kind[0] == 'class':
                continue
            lo = b + 1
            hi = pos if b == -1 else min(pos, src.match[b])
            decl = None
            for m in occ.finditer(src.mask, lo, hi):
                p = m.start()
                prev = src.skip_ws_back(p - 1)
                nxt = src.skip_ws(m.end())
                if src.block_of(p) == b and prev >= 0 and (src.mask[prev].isalnum() or src.mask[prev] in '_*&') \
                        and nxt < len(src.mask) and src.mask[nxt] in '={':
                    decl = (p, nxt)
            if decl is None:
                if kind[0] == 'func' and self.find_param(src, kind[3], name):
                    return None
                continue
            p, nxt = decl
            if src.mask[nxt] == '{':
                vals = literals(nxt + 1, src.match.get(nxt, nxt))
            else:
                vals = literals(nxt + 1, src.mask.find(';', nxt))
            if vals is None:
                return None
            end = len(src.mask) if b == -1 else src.match[b]
            for m in occ.finditer(src.mask, src.skip_ws(src.match.get(nxt, nxt) if src.mask[nxt] == '{' else nxt), end):
                q = m.start()
                before = src.mask[max(0, q - 2):q]
                after = src.mask[m.end():m.end() + 12].lstrip()
                if before.endswith(('++', '--', '&')) or re.match(r'(\+\+|--|[-+*/%|&^]=|<<=|>>=|\[|\.'
                                                                 r'(append|push_back|insert|replace|assign|erase|'
                                                                 r'clear|resize|swap)\b)', after):
                    return None
                if re.match(r'=(?!=)', after):
                    vs = m.end() + src.mask[m.end():].index('=') + 1
                    more = literals(vs, src.mask.find(';', vs))
                    if more is None:
                        return None
                    vals = vals + more
            return vals
        return None

    # -- callers ---------------------------------------------------------------
    @functools.lru_cache(maxsize=None)
    def type_of_receiver(self, recv: str):
        """Class names of variable `recv` from `Type recv` declarations anywhere in the sources."""
        pat = re.compile(r'\b([A-Z]\w*)\s*[*&]?\s+' + re.escape(recv) + r'\s*[{;=(\[]')
        types = set()
        for s in self.sources.values():
            types.update(t for t in pat.findall(s.mask) if t not in TAIL_KW)
        return types

    def call_sites(self, func: str, cls, def_src: Source, def_popen: int, lambda_scope=None):
        """[(src, open paren)] of calls to func (method of cls, or free function if cls is None)."""
        out = []
        cands = [def_src] if lambda_scope else self.sources.values()
        ctor = cls is not None and func == cls
        pat = re.compile(r'\b' + re.escape(func) + (r'(?:\s+[A-Za-z_]\w*)?\s*[({]' if ctor else r'\s*\('))
        for s in cands:
            for m in pat.finditer(s.mask):
                o = m.end() - 1
                if s is def_src and o == def_popen:
                    continue
                if lambda_scope and not (lambda_scope[0] < o < lambda_scope[1]):
                    continue
                a = s.args(o)
                if any(CONF_DECL_ARG.search(t) or DECL_ARG.match(t) for t, _, _ in a):
                    continue                                   # another declaration
                k = s.skip_ws_back(m.start() - 1)
                if ctor:
                    if k >= 1 and s.mask[k] == ':' and s.mask[k - 1] == ':':
                        continue
                    if s.mask[o] == '{' and s.classify_block(o)[0] == 'class':
                        continue
                elif k >= 0 and s.mask[k] in '.>' and (s.mask[k] == '.' or s.mask[k - 1] == '-'):
                    if cls is None or lambda_scope:
                        continue
                    recv, _ = s.ident_before(k - (1 if s.mask[k] == '.' else 2))
                    if not recv or cls not in self.type_of_receiver(recv):
                        continue
                elif k >= 1 and s.mask[k] == ':' and s.mask[k - 1] == ':':
                    q, _ = s.ident_before(k - 2)
                    if q != cls:
                        continue
                    nxt = s.skip_ws(s.match.get(o, o) + 1)
                    if nxt < len(s.mask) and (s.mask[nxt] in '{:' or s.mask[nxt:nxt + 5] == 'const'):
                        continue                               # out-of-line definition
                else:
                    if cls is not None and s.enclosing_class(o) != cls:
                        continue
                    prev, _ = s.ident_before(k)
                    if prev and prev != 'return' and s.mask[k].isalnum():
                        continue                               # `Type func(` declaration
                out.append((s, o))
        return out

    def _lambda_scope(self, src: Source, popen: int):
        if src.mask[src.skip_ws_back(popen - 1)] != ']':
            return None
        ob = src.block_of(popen)
        return popen, len(src.mask) if ob == -1 else src.match[ob]

    def trace_param(self, src: Source, kind, idx: int, name: str, depth: int):
        memo = (src.rel, kind[3], idx)
        if memo in self._trace_memo:
            res = self._trace_memo[memo]
            if isinstance(res, Unresolved):
                raise res
            return res
        try:
            return self._trace_param(src, kind, idx, name, depth, memo)
        except Unresolved as e:
            self._trace_memo[memo] = e
            raise

    def _trace_param(self, src: Source, kind, idx: int, name: str, depth: int, memo):
        _, func, cls, popen = kind
        if func is None:
            res = self._anon_lambda_param(src, popen, name)
            self._trace_memo[memo] = res
            return res
        self._trace_memo[memo] = Unresolved(f'parameter {name!r} of {func}() feeds itself')
        lo, hi = arity(src, popen)
        sites = [(s, o) for s, o in self.call_sites(func, cls, src, popen, self._lambda_scope(src, popen))
                 if lo <= len(s.args(o)) <= hi]
        confs, errs = set(), []
        for s, o in sites:
            a = s.args(o)[idx]
            try:
                confs |= self.conf_of_expr(s, a[1], a[2], depth + 1)
            except NotConfFile:
                continue                                       # call of another overload
            except Unresolved as e:
                errs.append(f'{s.where(o)}: {e}')
        label = f'{cls + "::" if cls else ""}{func}()'
        if errs:
            raise Unresolved(f'parameter {name!r} of {label}: ' + '; '.join(errs))
        if not confs:
            if name in VAR_TO_CONF:
                confs = {VAR_TO_CONF[name]}
            else:
                raise Unresolved(f'parameter {name!r} of {label}: no call site found')
        if len(confs) > 1 and SERVICE_ALL not in confs:
            raise Unresolved(f'parameter {name!r} of {label}: callers pass different files {sorted(confs)}')
        self._trace_memo[memo] = confs
        return confs

    def _anon_lambda_param(self, src: Source, popen: int, name: str):
        """An anonymous lambda passed to a call: LAMBDA_ARG_CONFS maps that call to the conf it is fed."""
        lb = src.match.get(src.skip_ws_back(popen - 1))
        b0 = src.block_of(lb)
        outer = None
        depth = 0
        for i in range(lb - 1, b0, -1):         # innermost ( enclosing the lambda
            c = src.mask[i]
            if c in ')]}':
                depth += 1
            elif c in '([{':
                if depth == 0:
                    outer = i if c == '(' else None
                    break
                depth -= 1
        if outer is not None:
            callee, _ = src.ident_before(outer - 1)
            a = src.args(outer)
            sig = f'{callee}({a[0][0]}' if a else None
            ov = LAMBDA_ARG_CONFS.get(src.rel, {})
            if sig in ov:
                self.used_overrides.add((src.rel, sig))
                return {ov[sig]}
        raise Unresolved(f'parameter {name!r} of an anonymous lambda at {src.where(popen)}; '
                         f'map the call it is passed to in LAMBDA_ARG_CONFS')

    def trace_key_param(self, src: Source, pos: int, keyname: str):
        """Literal keys a helper's key parameter receives from its callers, or None if not followable."""
        for b, kind in self.scopes(src, pos):
            if kind[0] == 'class':
                return None
            if kind[0] != 'func':
                continue
            hit = self.find_param(src, kind[3], keyname)
            if not hit:
                continue
            _, func, cls, popen = kind
            if func is None:
                return None
            keys = []
            lo, hi = arity(src, popen)
            for s, o in self.call_sites(func, cls, src, popen, self._lambda_scope(src, popen)):
                a = s.args(o)
                if not lo <= len(a) <= hi:
                    continue
                m = LIT_RE.match(a[hit[0]][0])
                if not m or not KEY_RE.match(m.group(1)):
                    return None
                keys.append(m.group(1))
            return keys or None
        return None

    def keys_of(self, src: Source, pos: int, expr: str):
        """Keys a getter's first argument names, or None when it is computed at runtime."""
        m = LIT_RE.match(expr)
        if m:
            return [m.group(1)] if KEY_RE.match(m.group(1)) else None
        if not IDENT.fullmatch(expr):
            return None
        lits = self.literal_values(src, expr, pos)
        if lits:
            return lits if all(KEY_RE.match(x) for x in lits) else None
        return self.trace_key_param(src, pos, expr)


REQUIRED = '<required>'   # getter called without a default
UNKNOWN = '<non-literal>'


@dataclass
class Scan:
    reads: list = field(default_factory=list)        # (conf, key, jtype, default, where)
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    resolved: dict = field(default_factory=dict)     # (rel, receiver) -> set of confs
    file_keys: dict = field(default_factory=dict)
    type_keys: dict = field(default_factory=dict)
    defaults: dict = field(default_factory=dict)     # key -> JS literal, only if every read agrees

    def add(self, conf, key, jtype, dflt, where):
        self.reads.append((conf, key, jtype, dflt, where))

    def finish(self, service_confs):
        """Expand Service-base reads, drop IGNORED_CONFS, derive the three tables."""
        reads = []
        for conf, key, jtype, dflt, where in self.reads:
            for c in (service_confs if conf == SERVICE_ALL else [conf]):
                if c not in IGNORED_CONFS:
                    reads.append((c, key, jtype, dflt, where))
        per_key = {}
        for conf, key, jtype, dflt, where in reads:
            self.file_keys.setdefault(conf, set()).add(key)
            self.type_keys.setdefault(jtype, set()).add(key)
            per_key.setdefault(key, {}).setdefault(dflt, []).append(f'{conf} {where}')
        for key, vals in sorted(per_key.items()):
            literal = [v for v in vals if v not in (REQUIRED, UNKNOWN)]
            if len(vals) == 1 and literal:
                self.defaults[key] = literal[0]
            elif literal:
                self.warnings.append(f'default of {key!r} differs between reads, left out of DEFAULTS: ' +
                                     '; '.join(f'{v} at {", ".join(sorted(set(w)))}' for v, w in vals.items()))
        multi = sorted({k for t, ks in self.type_keys.items() for k in ks
                        if sum(k in v for v in self.type_keys.values()) > 1})
        for k in multi:
            types = sorted(t for t, ks in self.type_keys.items() if k in ks)
            self.warnings.append(f'{k!r} is read as {"/".join(types)}')


# ----------------------------------------------------------------------------
# Default values
# ----------------------------------------------------------------------------
STR_RE = re.compile(r'^"[^"\\]*(?:\\.[^"\\]*)*"$')
HEX_RE = re.compile(r'^0x[0-9a-fA-F]+$')
BIN_RE = re.compile(r'^0b[01]+$')
NUM_EXPR = re.compile(r'^[\d.\s+\-*/()fe]+$')
_OPS = {ast.Add: op.add, ast.Sub: op.sub, ast.Mult: op.mul, ast.Div: op.truediv,
        ast.UAdd: op.pos, ast.USub: op.neg}


def safe_num_eval(s):
    tree = ast.parse(s, mode='eval')

    def go(n):
        if isinstance(n, ast.Expression): return go(n.body)
        if isinstance(n, ast.Constant) and isinstance(n.value, (int, float)):
            return n.value
        if isinstance(n, ast.UnaryOp) and type(n.op) in _OPS:
            return _OPS[type(n.op)](go(n.operand))
        if isinstance(n, ast.BinOp) and type(n.op) in _OPS:
            return _OPS[type(n.op)](go(n.left), go(n.right))
        raise ValueError('not a constant numeric expression')
    return go(tree)


def normalize_default(raw):
    """C++ default-value expression -> JS-literal string, or None if not a plain literal."""
    if raw is None:
        return None
    s = raw.strip()
    if not s:
        return None
    if STR_RE.match(s) or HEX_RE.match(s):
        return s
    if BIN_RE.match(s):
        return hex(int(s, 2))
    if NUM_EXPR.match(s):
        cleaned = re.sub(r'(\d\.?)f\b', r'\1', s)
        try:
            v = safe_num_eval(cleaned)
        except Exception:
            return None
        if isinstance(v, bool):
            return None
        if isinstance(v, int):
            return str(v)
        if isinstance(v, float):
            return f'{v:.1f}' if v == int(v) else repr(v)
    return None


def default_of(getter, args):
    """JS literal of a getter's default, REQUIRED when it has none, UNKNOWN when it is not a literal."""
    i = 2 if getter == 'getLong' and len(args) == 3 else 1   # getLong(key, base, def)
    if len(args) <= i:
        return REQUIRED
    if args[i][0] in ('nullptr', 'NULL') and getter == 'c':
        return REQUIRED
    v = normalize_default(args[i][0])
    return UNKNOWN if v is None else v


# ----------------------------------------------------------------------------
# Scan
# ----------------------------------------------------------------------------
def load_sources(repo: Path):
    out = {}
    for sub in ('src', 'main'):
        for ext in ('*.h', '*.hpp', '*.cpp', '*.c'):
            for p in (repo / sub).rglob(ext):
                rel = p.relative_to(repo).as_posix()
                out[rel] = Source(rel, p.read_text(encoding='utf-8', errors='replace'))
    return dict(sorted(out.items()))


def scan_pd_gains(rp: Repo, scan: Scan):
    """ctrl_<name>_{kp,kd,td}: pdLoadGains(conf, ctrl, "<name>") on top of the controller's constructor gains."""
    defs = [s for s in rp.sources.values() if re.search(r'\bpdLoadGains\s*\(\s*const\s+ConfFile', s.mask)]
    calls = [s for s in rp.sources.values() if re.search(r'(?<!void )\bpdLoadGains\s*\(\s*(?!const)', s.mask)]
    if not defs and not calls:
        return
    if len(defs) != 1 or 'std::string("ctrl_") + name + "_"' not in defs[0].text:
        scan.errors.append('pdLoadGains() no longer builds "ctrl_" + name + "_" (or is defined twice/never); '
                           'update scan_pd_gains')
        return
    n = 0
    for src in rp.sources.values():
        for m in re.finditer(r'\bpdLoadGains\s*\(', src.mask):
            o = m.end() - 1
            a = src.args(o)
            if len(a) != 3 or any(CONF_DECL_ARG.search(t) for t, _, _ in a):
                continue
            where = src.where(o)
            lm = LIT_RE.match(a[2][0])
            if not lm:
                scan.errors.append(f'{where}: pdLoadGains name {a[2][0]!r} is not a literal')
                continue
            try:
                confs = rp.conf_of_expr(src, a[0][1], a[0][2], 0)
            except Unresolved as e:
                scan.errors.append(f'{where}: pdLoadGains conf: {e}')
                continue
            gains = {}
            ctrl = a[1][0]
            for s in rp.sources.values():
                cm = re.search(r'\bPD_Control\w*\s+' + re.escape(ctrl) + r'\s*\{([^}]*)\}', s.text)
                if cm:
                    vals = [v.strip() for v in cm.group(1).split(',')]
                    gains = {'kp': normalize_default(vals[0]), 'kd': normalize_default(vals[1])}
                    break
            else:
                scan.warnings.append(f'{where}: no `PD_Control {ctrl}{{kp, kd, ...}}` found; ctrl_* defaults left out')
            for conf in confs:
                for g in ('kp', 'kd', 'td'):
                    scan.add(conf, f'ctrl_{lm.group(1)}_{g}', 'float', gains.get(g) or UNKNOWN, where)
            n += 1
    if not n:
        scan.errors.append('pdLoadGains() is defined but no call was understood; ctrl_* keys would be dropped')


def scan_repo(repo: Path) -> Scan:
    sources = load_sources(repo)
    return scan_sources(sources)


def scan_sources(sources: dict) -> Scan:
    rp = Repo(sources)
    scan = Scan()
    dyn_seen, guard_seen = set(), set()
    for src in sources.values():
        for m in GETTER_RE.finditer(src.mask):
            o = m.end() - 1
            getter = m.group('getter')
            jtype = GETTERS[getter]
            where = src.where(m.start())
            args = src.args(o)
            if not args or not args[0][0]:
                continue                                    # e.g. getString() with no args (not a conf read)
            keys = rp.keys_of(src, o, args[0][0])
            if keys is None:
                ack = (src.rel, re.sub(r'\s+', ' ', args[0][0]))
                dyn_seen.add(ack)
                if ack not in DYNAMIC_KEY_SITES:
                    scan.errors.append(f'{where}: key {args[0][0]!r} is computed at runtime; follow it or acknowledge '
                                       f'{ack!r} in DYNAMIC_KEY_SITES (and list its keys in INDIRECT_KEYS)')
                continue
            if len(keys) == 1 and (src.rel, keys[0]) in MIGRATION_GUARD_READS:
                guard_seen.add((src.rel, keys[0]))
                continue
            dot = m.start()
            rs = _recv_start(src, dot)
            recv = src.text[rs:dot].strip()
            try:
                confs = rp.conf_of_expr(src, rs, dot, 0)
            except Unresolved as e:
                scan.errors.append(f'{where}: unresolved getter receiver `{recv}`.{getter}(): {e}')
                continue
            scan.resolved.setdefault((src.rel, recv), set()).update(confs)
            dflt = default_of(getter, args)
            for conf in confs:
                for key in keys:
                    scan.add(conf, key, jtype, dflt, where)

    if re.search(r'\bclass\s+Service\b', ''.join(s.mask for s in sources.values())) and \
            not any(r[0] == SERVICE_ALL for r in scan.reads):
        scan.errors.append('src/service.h: Service::loadConf() getters not found; enabled/log_level would be dropped')

    scan_pd_gains(rp, scan)

    for ind in INDIRECT_KEYS:
        rel, rx = ind.evidence
        s = sources.get(rel)
        if s is None or not re.search(rx, s.text):
            scan.errors.append(f'INDIRECT_KEYS {ind.keys} ({ind.conf}): evidence /{rx}/ no longer in {rel}')
            continue
        for k in ind.keys:
            scan.add(ind.conf, k, ind.jtype, ind.defaults.get(k, REQUIRED), f'INDIRECT_KEYS ({rel})')

    for ack in sorted(set(DYNAMIC_KEY_SITES) - dyn_seen):
        scan.errors.append(f'DYNAMIC_KEY_SITES entry {ack!r} matches no getter (stale)')
    for ack in sorted(set(MIGRATION_GUARD_READS) - guard_seen):
        scan.errors.append(f'MIGRATION_GUARD_READS entry {ack!r} matches no getter (stale)')
    for rel, ov in list(FILE_VAR_OVERRIDES.items()) + list(LAMBDA_ARG_CONFS.items()):
        for var in ov:
            if (rel, var) not in rp.used_overrides:
                scan.errors.append(f'override [{rel!r}][{var!r}] matches no getter (stale)')

    scan.finish(sorted(set(rp.service_conf.values())))
    return scan


def _recv_start(src: Source, dot: int) -> int:
    """Start of the receiver expression ending right before the `.`/`->` at dot."""
    i = src.skip_ws_back(dot - 1)
    if i >= 0 and src.mask[i] in ')}' and i in src.match:
        o = src.match[i]
        _, s = src.ident_before(o - 1)
        return s if s < o else o
    _, s = src.ident_before(i)
    return s


# ----------------------------------------------------------------------------
# Table model
# ----------------------------------------------------------------------------
TAB_ORDER = ['board.conf', 'sensor.conf', 'limits.conf', 'coil.conf',
             'converter.conf', 'charger.conf', 'tracker.conf', 'mqtt.conf',
             'tele.conf', 'ftp.conf', 'http.conf', 'telnet.conf', 'tlscon.conf',
             'scope.conf', 'bsync.conf', 'lcd.conf', 'ble.conf', 'pprof.conf', 'wifi.conf']


def file_order(files):
    return [f for f in TAB_ORDER if f in files] + sorted(f for f in files if f not in TAB_ORDER)


def js(s):
    return json.dumps(s)


def _wrap(prefix, items, wrap_at=96, indent='    '):
    out, cur = [], prefix
    first = True
    for it in items:
        piece = ('' if first else ',') + it
        if len(cur) + len(piece) > wrap_at and not first:
            out.append(cur + ',')
            cur = indent + it
        else:
            cur += piece
        first = False
    out.append(cur)
    return out


def emit_file_keys(file_keys):
    out = ['const FILE_KEYS = {']
    for f in file_order(file_keys):
        lines = _wrap('  ' + js(f) + ':[', [js(k) for k in sorted(file_keys[f])])
        lines[-1] += '],'
        out.extend(lines)
    out.append('};')
    return '\n'.join(out)


def emit_type_keys(type_keys):
    out = ['const TYPE_KEYS = {']
    for jtype in ('byte', 'long', 'float', 'string'):
        lines = _wrap(f'  {jtype}: new Set([', [js(k) for k in sorted(type_keys.get(jtype, set()))])
        lines[-1] += ']),'
        out.extend(lines)
    out.append('};')
    return '\n'.join(out)


def emit_defaults(defaults, file_keys):
    lines = ['const DEFAULTS = {']
    written = set()
    for f in file_order(file_keys):
        keys_here = sorted(k for k in file_keys[f] if k in defaults and k not in written)
        if not keys_here:
            continue
        lines.append(f'  // {f}')
        block = _wrap('  ', [f'{js(k)}:{js(defaults[k])}' for k in keys_here], indent='  ')
        block[-1] += ','
        lines.extend(block)
        written.update(keys_here)
    lines.append('};')
    return '\n'.join(lines)


MARKER_RE = re.compile(
    r'(//\s*SCRAPER:BEGIN\s+(?P<name>[A-Z_]+)\s*\n)'
    r'(?P<body>[\s\S]*?)'
    r'(\n[ \t]*//\s*SCRAPER:END\s+(?P=name)\s*\n)'
)


def patch_html(html, blocks):
    found = set()

    def repl(m):
        name = m.group('name')
        if name not in blocks:
            return m.group(0)
        found.add(name)
        return m.group(1) + blocks[name] + m.group(4)
    new_html = MARKER_RE.sub(repl, html)
    return new_html, [k for k in blocks if k not in found]


def parse_tables(html):
    """(file_keys, type_keys, defaults) parsed back out of the SCRAPER blocks of an html."""
    bodies = {m.group('name'): m.group('body') for m in MARKER_RE.finditer(html)}
    fk, tk, df = {}, {}, {}
    for m in re.finditer(r'"([^"]+\.conf)"\s*:\s*\[([^\]]*)\]', bodies.get('FILE_KEYS', '')):
        fk[m.group(1)] = set(re.findall(r'"([^"]+)"', m.group(2)))
    for m in re.finditer(r'\b(byte|long|float|string)\s*:\s*new Set\(\[([^\]]*)\]\)', bodies.get('TYPE_KEYS', '')):
        tk[m.group(1)] = set(re.findall(r'"([^"]+)"', m.group(2)))
    body = re.sub(r'//[^\n]*', '', bodies.get('DEFAULTS', ''))
    for m in re.finditer(r'"([^"]+)"\s*:\s*("(?:[^"\\]|\\.)*")', body):
        df[m.group(1)] = json.loads(m.group(2))
    return fk, tk, df


def table_drift(old, new):
    """Human-readable differences between two (file_keys, type_keys, defaults) triples."""
    ofk, otk, odf = old
    nfk, ntk, ndf = new
    out = []
    for f in file_order(set(ofk) | set(nfk)):
        a, b = ofk.get(f, set()), nfk.get(f, set())
        if a - b:
            out.append(f'FILE_KEYS {f}: in html, not read by firmware: {", ".join(sorted(a - b))}')
        if b - a:
            out.append(f'FILE_KEYS {f}: read by firmware, missing in html: {", ".join(sorted(b - a))}')
    for t in ('byte', 'long', 'float', 'string'):
        a, b = otk.get(t, set()), ntk.get(t, set())
        if a - b:
            out.append(f'TYPE_KEYS {t}: in html only: {", ".join(sorted(a - b))}')
        if b - a:
            out.append(f'TYPE_KEYS {t}: firmware only: {", ".join(sorted(b - a))}')
    def same(a, b):
        if a == b:
            return True
        try:
            return a is not None and b is not None and float(a) == float(b)
        except ValueError:
            return False
    for k in sorted(set(odf) | set(ndf)):
        if not same(odf.get(k), ndf.get(k)):
            out.append(f'DEFAULTS {k}: html {odf.get(k, "<none>")} -> firmware {ndf.get(k, "<none>")}')
    return out


# ----------------------------------------------------------------------------
# META drift report
# ----------------------------------------------------------------------------
META_KEY_RE = re.compile(r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*\{', re.MULTILINE)
CHAN_KEY_RE = re.compile(r'^(hv_v|lv_v|hv_i|lv_i|ntc)_(adc|ch|rh|rl|factor|midpoint|filt_len|gain|offset)$')


def scrape_meta_keys(html):
    keys = set()
    m = re.search(r'const META = \{([\s\S]*?)^\};', html, re.MULTILINE)
    if m:
        keys.update(META_KEY_RE.findall(m.group(1)))
    fm = re.search(r'const FILE_META = \{([\s\S]*?)^\};', html, re.MULTILINE)
    if fm:
        keys.update(re.findall(r'\b([a-zA-Z_][a-zA-Z0-9_]*)\s*:\s*\{[^{}]*unit', fm.group(1)))
    return keys


def has_meta(key, meta_keys):
    return key in meta_keys or bool(CHAN_KEY_RE.match(key)) or key.startswith('ssid_')


# ----------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repo', type=Path, default=None, help='repo root (default: walk up from this script)')
    ap.add_argument('--html', type=Path, default=None,
                    help='editor html to check/rewrite (default: <repo>/etc/config-tool/conf-editor.html)')
    ap.add_argument('--write', action='store_true', help='rewrite the SCRAPER-managed blocks in the html')
    ap.add_argument('--check', action='store_true', help='exit non-zero if the html tables drift from firmware')
    ap.add_argument('--resolved', action='store_true', help='list which conf file each getter receiver resolved to')
    args = ap.parse_args(argv)

    repo = args.repo
    if repo is None:
        here = Path(__file__).resolve()
        for p in [here.parent, *here.parents]:
            if (p / 'src').is_dir() and (p / 'main').is_dir():
                repo = p
                break
    if repo is None or not repo.is_dir():
        ap.error('could not locate repo root (pass --repo)')
    html_path = args.html or repo / 'etc' / 'config-tool' / 'conf-editor.html'
    if not html_path.exists():
        ap.error(f'no editor html at {html_path}')

    scan = scan_repo(repo)
    defaults = scan.defaults
    print(f'scanned: {sum(len(v) for v in scan.file_keys.values())} (key,file) pairs across '
          f'{len(scan.file_keys)} files; {len(defaults)} defaults')
    for w in scan.warnings:
        print('  warning:', w)
    if args.resolved:
        print('\nresolved receivers:')
        for (rel, recv), confs in sorted(scan.resolved.items()):
            print(f'  {rel}: {recv} -> {", ".join(sorted(confs))}')
    if scan.errors:
        print(f'\n{len(scan.errors)} error(s); tables not derived, nothing written:', file=sys.stderr)
        for e in scan.errors:
            print('  error:', e, file=sys.stderr)
        return 2

    blocks = {
        'FILE_KEYS': emit_file_keys(scan.file_keys),
        'TYPE_KEYS': emit_type_keys(scan.type_keys),
        'DEFAULTS':  emit_defaults(defaults, scan.file_keys),
    }
    html = html_path.read_text(encoding='utf-8')
    new_html, missing = patch_html(html, blocks)
    drift = table_drift(parse_tables(html), parse_tables(new_html))

    meta_keys = scrape_meta_keys(html)
    all_keys = {k for ks in scan.file_keys.values() for k in ks}
    no_meta = sorted(k for k in all_keys if not has_meta(k, meta_keys) and k not in {'enabled', 'log_level'})
    stale_meta = sorted(k for k in meta_keys - all_keys if not has_meta(k, set()) and k not in META_ONLY_OK)

    if missing:
        print(f'\nmissing markers in {html_path.name}: ' + ', '.join(missing))
    if drift:
        print(f'\ntable drift ({len(drift)}):')
        for d in drift:
            print(' ', d)
    if no_meta:
        print(f'\nkeys read by firmware with no META[] description ({len(no_meta)}):')
        for k in no_meta:
            print(' ', k)
    if stale_meta:
        print(f'\nMETA[] keys not read by any firmware getter ({len(stale_meta)}), likely stale:')
        for k in stale_meta:
            print(' ', k)

    if args.write:
        if missing:
            print('\nrefusing to write: add the missing markers first.', file=sys.stderr)
            return 2
        if new_html == html:
            print('\nhtml already in sync, no write needed.')
            return 0
        html_path.write_text(new_html, encoding='utf-8')
        print(f'\nwrote {html_path}')
        return 0
    if args.check:
        if missing or drift or stale_meta:
            print('\nout of sync', file=sys.stderr)
            return 1
        print('\nin sync' + (' (--write would only reformat the blocks).' if new_html != html else '.'))
        return 0
    print('\n(report only; --write to apply, --check for CI)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
