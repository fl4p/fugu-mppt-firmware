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

The hand-curated META / FILE_META / CHAN_* tables and human descriptions are
*not* touched — those still need a human eye. The script prints which keys
have no META entry (so you know what to write a description for) and which
META entries point at keys no longer seen in firmware (likely stale).

Usage:
    python3 etc/config-tool/scrape_conf_keys.py             # report drift
    python3 etc/config-tool/scrape_conf_keys.py --write     # rewrite html
"""
from __future__ import annotations
import argparse
import ast
import json
import operator as op
import re
import sys
from pathlib import Path

# ----------------------------------------------------------------------------
# Variable -> conf-file mapping. Every getter call uses an object that maps to
# one .conf file. Ambiguous local names (`c`, `conf`) get per-file overrides
# below, plus the Service-base-class special-case at the bottom of scan_repo().
# ----------------------------------------------------------------------------
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
    'ftpConf':       'ftp.conf',
    'wifiConf':      'wifi.conf',
    'pprofConf':     'pprof.conf',
    'pin':           'board.conf',  # FugBackflow::init(pin) — caller passes boardConf
}

# In a few translation units a local var unambiguously refers to one conf.
# A value of None means "ignore this var here silently" (e.g. the Service base
# class reads `c.<key>` generically, and the SERVICE_CONFS broadcast below
# already covers those keys; or vconv-related vars that aren't exposed by the
# editor at all).
FILE_VAR_OVERRIDES = {
    'src/tele/mqtt.cpp':         {'conf': 'mqtt.conf'},
    'src/console_ble_service.h': {'c': 'ble.conf'},
    'src/service.h':             {'c': None},
    'src/adc/vconv.h':           {'vconvConf': None},
    'src/sensor_setup.cpp':      {'vc': None, 'coil': None},
    'src/conv_side.h':           {'conv': 'converter.conf'},   # readTopoIsBoost()
}

# Service base class reads these two keys from EVERY service's own conf file.
SERVICE_CONFS = ['mqtt.conf', 'tele.conf', 'ftp.conf', 'telnet.conf',
                 'scope.conf', 'lcd.conf', 'ble.conf']

# Keys the scraper cannot reach because:
#   - the key is a runtime variable (`boardConf.getByte(pnCtrl)` where pnCtrl
#     is one of "pwm_hi"/"pwm_li"/...)
#   - the call uses an inline ConfFile temporary (`ConfFile{path}.getByte(...)`)
# Hand-listed here so they still appear in FILE_KEYS / TYPE_KEYS / DEFAULTS.
# Format: { file: [(key, type, default-or-None), ...] }
EXTRA_KEYS = {
    'board.conf': [
        # buck.h selects one HiLi/InEn pin set at runtime from pnCtrl/pnRect
        ('pwm_hi',     'byte', None),
        ('pwm_li',     'byte', None),
        ('pwm_in',     'byte', None),
        ('pwm_en',     'byte', None),
        # LED data pins picked indirectly
        ('led_WS2812',  'long', None),
        ('led_WS2812B', 'long', None),
        # i2c_scl is read via getLong but the line we sample uses getByte/getLong
        # depending on context — keep as long
        ('i2c_scl',    'long', None),
    ],
    'limits.conf': [
        # Limits() reads these via readMappedLimit() (src/conv_side.h), which builds the side key
        # from limitSideKey() at runtime. Sensor channel keys (hv_v_rh, ...) are built at runtime
        # too; the editor knows them by pattern (CHAN_RE), not from FILE_KEYS.
        ('hv_max',   'float', None),
        ('lv_max',   'float', None),
        ('hv_i_max', 'float', None),
        ('lv_i_max', 'float', None),
    ],
    'lcd.conf': [
        # read via `ConfFile{_confPath}.getByte("addr", 0)` — temporary, not a var
        ('addr', 'byte', '0'),
    ],
}

# ----------------------------------------------------------------------------
# Source scanning
# ----------------------------------------------------------------------------
HEAD_RE = re.compile(
    r'(?P<var>[A-Za-z_][A-Za-z0-9_]*)'
    r'\.(?P<getter>getByte|getLong|getFloat|getString|f)'
    r'\s*\('
)
GETTERS = {'getByte': 'byte', 'getLong': 'long', 'getFloat': 'float',
           'getString': 'string', 'f': 'float'}

STR_RE = re.compile(r'^"[^"\\]*(?:\\.[^"\\]*)*"$')
HEX_RE = re.compile(r'^0x[0-9a-fA-F]+$')
BIN_RE = re.compile(r'^0b[01]+$')
NUM_EXPR = re.compile(r'^[\d.\s+\-*/()fe]+$')


def parse_call_args(text, start):
    """Return (args, end_pos) for the call beginning at text[start-1]='('.

    Honours nested parens and quoted strings. Bails on unterminated input.
    """
    args, cur, depth, in_str = [], '', 0, False
    i = start
    while i < len(text):
        c = text[i]
        if in_str:
            cur += c
            if c == '\\' and i + 1 < len(text):
                cur += text[i + 1]
                i += 2
                continue
            if c == '"':
                in_str = False
        elif c == '"':
            cur += c
            in_str = True
        elif c == '(':
            cur += c
            depth += 1
        elif c == ')':
            if depth == 0:
                args.append(cur.strip())
                return args, i + 1
            cur += c
            depth -= 1
        elif c == ',' and depth == 0:
            args.append(cur.strip())
            cur = ''
        else:
            cur += c
        i += 1
    return None, i


# Safe constant-arithmetic evaluator: only literals + ( ) + - * / and unary +-.
# No names, no calls, no attribute access — keeps the linter quiet and the
# scraper safe to run against any input.
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
    """Turn a C++ default-value expression into a JS-literal string.

    Returns None when the default isn't a recognisable literal (NAN, identifier,
    function call, ternary, …); those keys stay unannotated in the HTML.
    """
    if raw is None:
        return None
    s = re.sub(r'//.*$', '', raw).strip()
    if not s:
        return None
    if STR_RE.match(s):
        return s
    if HEX_RE.match(s):
        return s
    if BIN_RE.match(s):
        return hex(int(s, 2))
    if NUM_EXPR.match(s):
        cleaned = re.sub(r'(\d)f\b', r'\1', s)   # drop trailing C++ 'f' float suffix
        try:
            v = safe_num_eval(cleaned)
        except Exception:
            return None
        if isinstance(v, bool):
            return None
        if isinstance(v, int):
            return str(v)
        if isinstance(v, float):
            if v == int(v):
                return f'{v:.1f}'
            return repr(v)
    return None


def scan_repo(repo):
    """Walk src/ and main/, return (file_keys, type_keys, defaults, warnings)."""
    file_keys = {}
    type_keys = {}
    defaults = {}
    unknown_vars = {}
    warnings = []

    sources = []
    for sub in ('src', 'main'):
        sources.extend((repo / sub).rglob('*.h'))
        sources.extend((repo / sub).rglob('*.cpp'))

    for path in sources:
        rel = path.relative_to(repo).as_posix()
        per_file_override = FILE_VAR_OVERRIDES.get(rel, {})
        try:
            text = path.read_text(encoding='utf-8', errors='replace')
        except OSError:
            continue
        # strip comments so we don't scan disabled getters
        text = re.sub(r'/\*[\s\S]*?\*/', '', text)
        text = re.sub(r'//[^\n]*', '', text)

        for m in HEAD_RE.finditer(text):
            var = m.group('var')
            jtype = GETTERS[m.group('getter')]

            args, _ = parse_call_args(text, m.end())
            if not args:
                continue
            km = re.match(r'^"([A-Za-z_][A-Za-z0-9_]*)"$', args[0])
            if not km:
                continue                          # dynamic key (chn + '_' + ...)
            key = km.group(1)

            if var in per_file_override:
                conf = per_file_override[var]
                if conf is None:
                    continue                      # silenced for this file
            else:
                conf = VAR_TO_CONF.get(var)
                if not conf:
                    unknown_vars.setdefault(var, set()).add(rel)
                    continue

            file_keys.setdefault(conf, set()).add(key)
            type_keys.setdefault(jtype, set()).add(key)
            if len(args) >= 2:
                d = normalize_default(args[1])
                if d is not None and key not in defaults:
                    defaults[key] = d

    # Service base class: every service .conf reads enabled + log_level.
    for svc in SERVICE_CONFS:
        file_keys.setdefault(svc, set()).update(['enabled', 'log_level'])
    type_keys.setdefault('byte',   set()).add('enabled')
    type_keys.setdefault('string', set()).add('log_level')
    defaults.setdefault('enabled',   '1')
    defaults.setdefault('log_level', '"info"')

    # Hand-listed keys for getters the scraper cannot reach automatically.
    for fname, entries in EXTRA_KEYS.items():
        for key, jtype, dflt in entries:
            file_keys.setdefault(fname, set()).add(key)
            type_keys.setdefault(jtype, set()).add(key)
            if dflt is not None and key not in defaults:
                defaults[key] = dflt

    for var, paths in sorted(unknown_vars.items()):
        warnings.append(
            f'unknown getter var {var!r} in: ' + ', '.join(sorted(paths)) +
            ' — add it to VAR_TO_CONF or FILE_VAR_OVERRIDES'
        )
    return file_keys, type_keys, defaults, warnings


# ----------------------------------------------------------------------------
# JS code generation
# ----------------------------------------------------------------------------
def js(s):
    return json.dumps(s)


def _wrap(prefix, items, wrap_at=96, indent='    '):
    """Join `items` with commas, wrapping to keep lines under wrap_at chars."""
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


def emit_file_keys(file_keys, order):
    out = ['const FILE_KEYS = {']
    for f in order:
        if f not in file_keys:
            continue
        keys = sorted(file_keys[f])
        prefix = '  ' + js(f) + ':['
        lines = _wrap(prefix, [js(k) for k in keys])
        lines[-1] = lines[-1] + '],'
        out.extend(lines)
    out.append('};')
    return '\n'.join(out)


def emit_type_keys(type_keys):
    out = ['const TYPE_KEYS = {']
    for jtype in ('byte', 'long', 'float', 'string'):
        keys = sorted(type_keys.get(jtype, set()))
        prefix = f'  {jtype}: new Set(['
        lines = _wrap(prefix, [js(k) for k in keys])
        lines[-1] = lines[-1] + ']),'
        out.extend(lines)
    out.append('};')
    return '\n'.join(out)


def emit_defaults(defaults, file_keys, order):
    """One commented block per conf file, files in TAB_ORDER, keys A→Z inside."""
    lines = ['const DEFAULTS = {']
    written = set()
    for f in order:
        keys_here = sorted(k for k in file_keys.get(f, set())
                           if k in defaults and k not in written)
        if not keys_here:
            continue
        lines.append(f'  // {f}')
        items = [f'{js(k)}:{js(defaults[k])}' for k in keys_here]
        block = _wrap('  ', items, indent='  ')
        block[-1] = block[-1] + ','
        lines.extend(block)
        written.update(keys_here)
    lines.append('};')
    return '\n'.join(lines)


# ----------------------------------------------------------------------------
# HTML patcher
# ----------------------------------------------------------------------------
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
    missing = [k for k in blocks if k not in found]
    return new_html, missing


# ----------------------------------------------------------------------------
# META drift report (no rewriting)
# ----------------------------------------------------------------------------
META_KEY_RE = re.compile(r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*\{', re.MULTILINE)
CHAN_KEY_RE = re.compile(r'^(hv_v|lv_v|hv_i|lv_i|ntc)_(adc|ch|rh|rl|factor|midpoint|filt_len|gain|offset)$')


def scrape_meta_keys(html):
    """Union of META[key] and FILE_META[file][key] entries declared in the HTML."""
    keys = set()
    m = re.search(r'const META = \{([\s\S]*?)^\};', html, re.MULTILINE)
    if m:
        keys.update(META_KEY_RE.findall(m.group(1)))
    fm = re.search(r'const FILE_META = \{([\s\S]*?)^\};', html, re.MULTILINE)
    if fm:
        # FILE_META lines look like:  "charger.conf": { vout_max:{...} }
        keys.update(re.findall(r'\b([a-zA-Z_][a-zA-Z0-9_]*)\s*:\s*\{[^{}]*unit', fm.group(1)))
    return keys


def has_meta(key, meta_keys):
    if key in meta_keys: return True
    if CHAN_KEY_RE.match(key): return True       # hv_v_/lv_v_/hv_i_/lv_i_/ntc_ + suffix
    if key.startswith('ssid_'): return True       # wifi.conf is pattern-based
    return False


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument('--repo', type=Path, default=None,
                    help='repo root (default: walk up from this script)')
    ap.add_argument('--write', action='store_true',
                    help='rewrite the SCRAPER-managed blocks in conf-editor.html')
    ap.add_argument('--check', action='store_true',
                    help='exit non-zero if the html is out of sync (CI use)')
    args = ap.parse_args()

    here = Path(__file__).resolve()
    repo = args.repo
    if repo is None:
        for p in [here.parent, *here.parents]:
            if (p / 'src').is_dir() and (p / 'main').is_dir():
                repo = p
                break
    if repo is None or not repo.is_dir():
        ap.error('could not locate repo root (pass --repo)')

    html_path = repo / 'etc' / 'config-tool' / 'conf-editor.html'
    if not html_path.exists():
        ap.error(f'no conf-editor.html at {html_path}')

    file_keys, type_keys, defaults, warnings = scan_repo(repo)

    TAB_ORDER = ['board.conf', 'sensor.conf', 'limits.conf', 'coil.conf',
                 'converter.conf', 'charger.conf', 'tracker.conf', 'mqtt.conf',
                 'tele.conf', 'ftp.conf', 'telnet.conf', 'scope.conf',
                 'lcd.conf', 'ble.conf', 'pprof.conf', 'wifi.conf']
    order = TAB_ORDER + sorted(f for f in file_keys if f not in TAB_ORDER)

    blocks = {
        'FILE_KEYS': emit_file_keys(file_keys, order),
        'TYPE_KEYS': emit_type_keys(type_keys),
        'DEFAULTS':  emit_defaults(defaults, file_keys, order),
    }

    html = html_path.read_text(encoding='utf-8')
    new_html, missing = patch_html(html, blocks)
    in_sync = new_html == html

    meta_keys = scrape_meta_keys(html)
    all_keys = {k for ks in file_keys.values() for k in ks}
    no_meta  = sorted(k for k in all_keys
                      if not has_meta(k, meta_keys) and k not in {'enabled', 'log_level'})
    # don't flag channel-pattern META names or known legacy keys as stale
    legacy = {'boost', 'mode', 'conversion_eff', 'iin_min_supply_voltage',
              'vin_calib', 'vout_calib'}
    stale_meta = sorted(k for k in meta_keys - all_keys
                        if not has_meta(k, set()) and k not in legacy)

    print(f'scanned: {sum(len(v) for v in file_keys.values())} (key,file) pairs across '
          f'{len(file_keys)} files; {len(defaults)} defaults')
    for w in warnings:
        print('  warning:', w)
    if missing:
        print(f'\nmissing markers in {html_path.name}: ' + ', '.join(missing))
        print('  (add `// SCRAPER:BEGIN <NAME>` / `// SCRAPER:END <NAME>` around each block)')
    if no_meta:
        print(f'\nkeys read by firmware with no META[] description ({len(no_meta)}):')
        for k in no_meta: print(' ', k)
    if stale_meta:
        print(f'\nMETA[] keys not seen in any firmware getter ({len(stale_meta)}) — likely stale:')
        for k in stale_meta: print(' ', k)

    if args.write:
        if missing:
            print('\nrefusing to write: add the missing markers first.', file=sys.stderr)
            return 2
        if in_sync:
            print('\nhtml already in sync, no write needed.')
            return 0
        html_path.write_text(new_html, encoding='utf-8')
        print(f'\nwrote {html_path}')
        return 0

    if args.check:
        if missing or not in_sync:
            print('\nout of sync — run with --write', file=sys.stderr)
            return 1
        print('\nin sync.')
        return 0

    print('\n(report only — re-run with --write to apply, or --check for CI)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
