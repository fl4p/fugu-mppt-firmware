// Decoder for the firmware's binary telemetry wire (src/tele/sym_line_protocol.h),
// a port of etc/fugu/teledec.py. Dependency-free UMD: module.exports / window.TeleDec.
//
//   payload = (<varint len> <frame>)*            frame[0]: 1 = data, 2 = table
//   table   : (<SID> <name\0> <DT byte>)* 0
//   data    : <SID meas> (<SID tagK><SID tagV>)* 0 (<SID fieldK><raw LE>)* 0 <ts_ms varint>
//   BLE record: <0x7E><varint len><cid><payload>, cid 0 = raw, 1 = tamp
(function (root, factory) {
    if (typeof module === 'object' && module.exports) module.exports = factory();
    else root.TeleDec = factory();
})(typeof self !== 'undefined' ? self : this, function () {
    'use strict';

    const RECORD_MAGIC = 0x7E;
    const MAX_RECORD = 4096;
    // WireDT -> byte width (0 = Str, never a field value)
    const DT_WIDTH = [0, 1, 1, 1, 2, 2, 4, 4, 2, 4, 8];
    const DT = {STR: 0, BOOL: 1, I8: 2, U8: 3, I16: 4, U16: 5, I32: 6, U32: 7, F16: 8, F32: 9, F64: 10};

    const utf8 = new TextDecoder('utf-8', {fatal: true});

    function eof() { return new RangeError('unexpected end of data'); }

    // LEB128; accumulates with multiplication so values above 2^32 (ms timestamps) stay exact.
    function readVarint(b, i) {
        let v = 0, s = 0;
        for (;;) {
            if (i >= b.length) throw eof();
            const x = b[i++];
            v += (x & 0x7F) * 2 ** s;
            if (!(x & 0x80)) return {value: v, pos: i};
            s += 7;
        }
    }

    function cstr(b, i) {
        const j = b.indexOf(0, i);
        if (j < 0) throw eof();
        return {value: utf8.decode(b.subarray(i, j)), pos: j + 1};
    }

    function f16ToF32(h) {
        const s = h & 0x8000 ? -1 : 1, e = (h >> 10) & 0x1F, m = h & 0x3FF;
        if (e === 0) return s * m * 2 ** -24;
        if (e === 31) return m ? NaN : s * Infinity;
        return s * (1 + m / 1024) * 2 ** (e - 15);
    }

    // Python '%.{prec}g' (correctly rounded, ties to even on the exact binary value).
    function fmtG(x, prec = 6) {
        if (Number.isNaN(x)) return 'nan';
        if (!Number.isFinite(x)) return x > 0 ? 'inf' : '-inf';
        if (x === 0) return Object.is(x, -0) ? '-0' : '0';
        const sign = x < 0 ? '-' : '';
        x = Math.abs(x);
        const dv = new DataView(new ArrayBuffer(8));
        dv.setFloat64(0, x);
        const hi = dv.getUint32(0), lo = dv.getUint32(4), be = (hi >>> 20) & 0x7FF;
        let N = (BigInt(hi & 0xFFFFF) << 32n) | BigInt(lo), D = 1n;
        let e2 = -1074;
        if (be) { N |= 1n << 52n; e2 = be - 1075; }
        if (e2 >= 0) N <<= BigInt(e2); else D <<= BigInt(-e2);
        const p10 = (k) => 10n ** BigInt(k);
        const ge = (e) => e >= 0 ? N >= D * p10(e) : N * p10(-e) >= D;   // N/D >= 10^e
        let e10 = Math.floor(Math.log10(x));
        while (!ge(e10)) e10--;
        while (ge(e10 + 1)) e10++;
        const sh = prec - 1 - e10;
        const num = sh >= 0 ? N * p10(sh) : N, den = sh >= 0 ? D : D * p10(-sh);
        let q = num / den;
        const r2 = (num % den) * 2n;
        if (r2 > den || (r2 === den && (q & 1n))) q++;
        if (q === p10(prec)) { q = p10(prec - 1); e10++; }
        const d = q.toString();
        const strip = (s) => s.replace(/0+$/, '').replace(/\.$/, '');
        if (e10 >= -4 && e10 < prec) {
            const s = e10 >= 0 ? d.slice(0, e10 + 1) + '.' + d.slice(e10 + 1) : '0.' + '0'.repeat(-e10 - 1) + d;
            return sign + (s.includes('.') ? strip(s) : s);
        }
        const m = d.length > 1 ? strip(d[0] + '.' + d.slice(1)) : d;
        const ae = Math.abs(e10);
        return sign + m + 'e' + (e10 < 0 ? '-' : '+') + (ae < 10 ? '0' : '') + ae;
    }

    // --- tamp (plain v1: no custom dictionary, no extended tokens, no dictionary reset) -------

    const TAMP_CHARS = [0x20, 0, 0x30, 0x65, 0x69, 0x3E, 0x74, 0x6F, 0x3C, 0x61, 0x6E, 0x73, 0x0A, 0x72, 0x2F, 0x2E];
    const TAMP_FLUSH = 14;
    // Huffman codes following the match flag, keyed by (bit length, code)
    const HUFF = new Map([[1, 0b0], [2, 0b11], [4, 0b1000], [4, 0b1011], [5, 0b10100], [6, 0b100100],
        [6, 0b100110], [6, 0b101011], [7, 0b1001011], [7, 0b1010100], [8, 0b10010100], [8, 0b10010101],
        [8, 0b10101010], [6, 0b100111], [8, 0b10101011]].map(([len, code], sym) => [len * 256 + code, sym]));

    function tampInitDictionary(size = 1024) {
        const w = new Uint8Array(size);
        let seed = 3758097560, r = 0;
        for (let i = 0; i < size; i++) {
            if ((i & 7) === 0) {
                seed ^= seed << 13; seed >>>= 0;
                seed ^= seed >>> 17;
                seed ^= seed << 5; seed >>>= 0;
                r = seed;
            }
            w[i] = TAMP_CHARS[r & 0xF];
            r >>>= 4;
        }
        return w;
    }

    // Decompress one independent tamp stream. EOF mid-token ends the output (as the C decoder).
    function tampDecompress(input) {
        if (!input.length) throw new Error('tamp: missing header');
        const h = input[0];
        const wbits = ((h >> 5) & 7) + 8, lbits = ((h >> 3) & 3) + 5;
        if (h & 0x04) throw new Error('tamp: custom dictionary not supported');
        if (h & 0x02) throw new Error('tamp: extended stream not supported');
        if (h & 0x01) throw new Error('tamp: extra header bytes not supported');
        const wsize = 1 << wbits, mask = wsize - 1;
        const minPat = 2 + (wbits > 10 + ((lbits - 5) << 1) ? 1 : 0);
        const win = tampInitDictionary(wsize);
        let wp = 0, out = new Uint8Array(Math.max(64, input.length * 3)), n = 0;
        const total = input.length * 8;
        let bp = 8;
        const bits = (k) => {
            let v = 0;
            for (let j = 0; j < k; j++, bp++) v = (v << 1) | ((input[bp >> 3] >> (7 - (bp & 7))) & 1);
            return v;
        };
        const emit = (byte) => {
            if (n === out.length) { const o = new Uint8Array(out.length * 2); o.set(out); out = o; }
            out[n++] = byte;
        };
        while (bp < total) {
            if (bits(1)) {
                if (total - bp < lbits) break;
                const c = bits(lbits);
                emit(c);
                win[wp] = c; wp = (wp + 1) & mask;
                continue;
            }
            let code = 0, len = 0, sym = -1;
            while (len < 8 && bp < total) {
                code = (code << 1) | bits(1); len++;
                const s = HUFF.get(len * 256 + code);
                if (s !== undefined) { sym = s; break; }
            }
            if (sym < 0) {
                if (len < 8) break;                       // EOF inside the Huffman code
                throw new Error('tamp: bad huffman code');
            }
            if (sym === TAMP_FLUSH) { bp = (bp + 7) & ~7; continue; }
            if (total - bp < wbits) break;
            const off = bits(wbits), size = sym + minPat;
            if (off + size > wsize) throw new Error('tamp: match out of window');
            const m = win.slice(off, off + size);      // read the whole match before writing
            for (let j = 0; j < size; j++) { emit(m[j]); win[wp] = m[j]; wp = (wp + 1) & mask; }
        }
        return out.slice(0, n);
    }

    // --- payload ------------------------------------------------------------------------------

    function readValue(dt, raw) {
        const w = DT_WIDTH[dt];
        if (!w) throw new Error(`unknown field datatype ${dt}`);
        if (raw.length < w) throw eof();
        const v = new DataView(raw.buffer, raw.byteOffset, w);
        switch (dt) {
            case DT.BOOL: return v.getUint8(0) !== 0;
            case DT.I8: return v.getInt8(0);
            case DT.U8: return v.getUint8(0);
            case DT.I16: return v.getInt16(0, true);
            case DT.U16: return v.getUint16(0, true);
            case DT.I32: return v.getInt32(0, true);
            case DT.U32: return v.getUint32(0, true);
            case DT.F16: return f16ToF32(v.getUint16(0, true));
            case DT.F32: return v.getFloat32(0, true);
            default: return v.getFloat64(0, true);
        }
    }

    // Point[] {meas, tags, fields, types, ts}. Points are appended to `out` as they decode, so a
    // later error keeps the earlier ones (as the Python generator does). `table` (Map SID ->
    // [name, dt]) persists across payloads.
    function decodePayload(cid, payload, table, out = []) {
        if (cid === 1) payload = tampDecompress(payload);
        else if (cid !== 0) throw new Error(`unknown compressor id ${cid}`);
        let i = 0;
        const sym = (sid) => {
            const e = table.get(sid);
            if (!e) throw new Error(`unknown SID ${sid}`);
            return e;
        };
        while (i < payload.length) {
            let r = readVarint(payload, i);
            const f = payload.subarray(r.pos, r.pos + r.value);
            i = r.pos + r.value;
            let j = 0;
            const vi = () => { const x = readVarint(f, j); j = x.pos; return x.value; };
            const peek = () => { if (j >= f.length) throw eof(); return f[j]; };
            const ft = peek(); j++;
            if (ft === 2) {
                for (;;) {
                    const sid = vi();
                    if (sid === 0) break;
                    const s = cstr(f, j); j = s.pos;
                    table.set(sid, [s.value, peek()]); j++;
                }
            } else if (ft === 1) {
                const p = {meas: sym(vi())[0], tags: {}, fields: {}, types: {}, ts: 0};
                while (peek() !== 0) { const k = sym(vi())[0]; p.tags[k] = sym(vi())[0]; }
                j++;
                while (peek() !== 0) {
                    const [k, dt] = sym(vi());
                    const w = DT_WIDTH[dt];
                    p.fields[k] = readValue(dt, f.subarray(j, j + (w || 0)));
                    p.types[k] = dt;
                    j += w;
                }
                j++;
                p.ts = vi();
                out.push(p);
            } else {
                throw new Error(`unknown frame type ${ft}`);
            }
        }
        return out;
    }

    function fmtField(dt, v) {
        if (dt === DT.BOOL) return v ? 'true' : 'false';
        if (dt <= DT.U32) return v + 'i';
        return fmtG(v);
    }

    function pointToLine(p) {
        const tags = Object.entries(p.tags).map(([k, v]) => `,${k}=${v}`).join('');
        const fields = Object.keys(p.fields).map((k) => `${k}=${fmtField(p.types[k], p.fields[k])}`).join(',');
        return `${p.meas}${tags} ${fields} ${p.ts}`;
    }

    // --- BLE record framing -------------------------------------------------------------------

    // Reassembles <0x7E><varint len><cid><payload> records from a notify byte stream; feed()
    // returns decoded points. 0x7E occurs inside payloads, so a candidate record must pass the
    // length bound and cid check; otherwise rescan from the next 0x7E. A framed record that fails
    // to decode is skipped and the framing kept.
    class TeleStream {
        constructor(onError) {
            this.buf = new Uint8Array(0);
            this.table = new Map();
            this.onError = onError || null;
            this.stats = {records: 0, resyncs: 0, errors: 0, bytes: 0};
        }

        get pending() { return this.buf.length; }

        _resync() {
            this.stats.resyncs++;
            const nxt = this.buf.indexOf(RECORD_MAGIC, 1);
            this.buf = this.buf.subarray(nxt > 0 ? nxt : this.buf.length);
        }

        feed(data) {
            data = data instanceof Uint8Array ? data : new Uint8Array(data);
            this.stats.bytes += data.length;
            const b = new Uint8Array(this.buf.length + data.length);
            b.set(this.buf); b.set(data, this.buf.length);
            this.buf = b;
            const points = [];
            while (this.buf.length) {
                const buf = this.buf;
                if (buf[0] !== RECORD_MAGIC) { this._resync(); continue; }
                let n = 0, s = 0, i = 1;
                for (;;) {
                    if (i >= buf.length) return points;
                    const x = buf[i++];
                    n += (x & 0x7F) * 2 ** s; s += 7;
                    if (!(x & 0x80)) break;
                    if (s > 28) { n = MAX_RECORD + 1; break; }
                }
                if (n < 1 || n > MAX_RECORD || (buf.length > i && buf[i] !== 0 && buf[i] !== 1)) {
                    this._resync(); continue;
                }
                if (buf.length < i + n) return points;
                const rec = buf.slice(i, i + n);
                this.buf = buf.subarray(i + n);
                const before = points.length;
                try {
                    decodePayload(rec[0], rec.subarray(1), this.table, points);
                    this.stats.records++;
                } catch (e) {
                    this.stats.errors++;
                    if (this.onError) this.onError(e, points.slice(before));
                }
            }
            return points;
        }
    }

    const ANSI_RE = /\x1b\[[0-9;?]*[ -\/]*[@-~]/g;
    function stripAnsi(s) { return s.replace(ANSI_RE, ''); }

    // --- sweep curve console lines ------------------------------------------------------------
    //   sc: t=<epoch_s> n=<N> mpp=<W>,<V> sweeping=0|1
    //   sc: V P V P ...      (repeated)
    //   sc: end   |   sc: none
    const SC_RE = /(?:^|[^A-Za-z0-9_])sc:\s*(.*?)\s*$/;
    const NUM_RE = /^[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?$|^[-+]?(?:inf|nan)$/i;

    // Returns null for a non-`sc:` line, else {kind: 'header'|'data'|'end'|'none'|'invalid', ...}.
    function parseScLine(line) {
        const m = SC_RE.exec(stripAnsi(line));
        if (!m) return null;
        const body = m[1];
        if (body === 'end' || body === 'none') return {kind: body};
        if (/(?:^|\s)t=/.test(body)) {
            const kv = {};
            for (const tok of body.split(/\s+/)) {
                const e = tok.indexOf('=');
                if (e > 0) kv[tok.slice(0, e)] = tok.slice(e + 1);
            }
            const mpp = (kv.mpp || '').split(',');
            const h = {kind: 'header', t: Number(kv.t), n: Number(kv.n),
                mpp: {P: Number(mpp[0]), V: Number(mpp[1])}, sweeping: kv.sweeping === '1'};
            if (!Number.isFinite(h.t) || !Number.isInteger(h.n) || h.n < 0 || mpp.length !== 2 ||
                !NUM_RE.test(mpp[0]) || !NUM_RE.test(mpp[1])) return {kind: 'invalid', text: body};
            return h;
        }
        const vals = [];
        let truncated = false;
        for (const tok of body ? body.split(/\s+/) : []) {
            if (!NUM_RE.test(tok)) { truncated = true; break; }
            vals.push(Number(tok));
        }
        if (vals.length % 2) truncated = true;
        const points = [];
        for (let i = 0; i + 1 < vals.length; i += 2) points.push({V: vals[i], P: vals[i + 1]});
        if (!points.length && !truncated) return {kind: 'invalid', text: body};
        return {kind: 'data', points, truncated};
    }

    // Assembles one curve from header, data and `end` lines. feed() returns undefined until a
    // curve finishes, then {complete, curve, reason}; `sc: none` gives {complete: true, curve: null}.
    class SweepCurveParser {
        constructor() { this.cur = null; }

        _finish(reason) {
            const c = this.cur;
            this.cur = null;
            const curve = {t: c.t, n: c.n, mpp: c.mpp, sweeping: c.sweeping, points: c.points};
            if (!reason && c.points.length !== c.n) reason = 'short';
            if (!reason && c.truncated) reason = 'truncated';
            return reason ? {complete: false, curve, reason} : {complete: true, curve};
        }

        feed(line) {
            const p = parseScLine(line);
            if (!p) return undefined;
            switch (p.kind) {
                case 'header': {
                    const prev = this.cur ? this._finish('restart') : undefined;
                    this.cur = {t: p.t, n: p.n, mpp: p.mpp, sweeping: p.sweeping, points: [], truncated: false};
                    return prev;
                }
                case 'data':
                    if (this.cur) {
                        this.cur.points.push(...p.points);
                        if (p.truncated) this.cur.truncated = true;
                    }
                    return undefined;
                case 'end':
                    return this.cur ? this._finish() : undefined;
                case 'none':
                    this.cur = null;
                    return {complete: true, curve: null};
                default:
                    return undefined;
            }
        }
    }

    return {
        RECORD_MAGIC, MAX_RECORD, DT, DT_WIDTH,
        readVarint, cstr, f16ToF32, fmtG, tampInitDictionary, tampDecompress,
        decodePayload, pointToLine, TeleStream, stripAnsi, parseScLine, SweepCurveParser,
    };
});
