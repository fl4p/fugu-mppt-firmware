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
    const MAX_SYMBOLS = 4096;          // per table; a larger table is reset
    const MAX_TAMP_OUT = 65536;
    const PMA2_MAX_EVENTS = 512, PMA2_MAX_PREFIX = 4096, PMA2_MAX_UNION = 32, PMA2_MAX_ROWS = 128, PMA2_MAX_PACKET = 8192;
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

    function crc32(b, end = b.length) {
        let c = 0xFFFFFFFF;
        for (let i = 0; i < end; i++) {
            c ^= b[i];
            for (let j = 0; j < 8; j++) c = (c & 1) ? ((c >>> 1) ^ 0xEDB88320) : (c >>> 1);
        }
        return (c ^ 0xFFFFFFFF) >>> 0;
    }

    function pminiVarint(b, i, end) {
        let v = 0, s = 0, first = true;
        for (;;) {
            if (i >= end) throw new Error('PMA2 truncated varint');
            const x = b[i++];
            if (!first && x === 0 && s) throw new Error('PMA2 non-canonical varint');
            first = false;
            v += (x & 0x7F) * 2 ** s;
            if (!(x & 0x80)) return {value: v, pos: i};
            s += 7;
            if (s > 35) throw new Error('PMA2 varint too long');
        }
    }

    class PMiniBits {
        constructor(b, start, end) { this.b = b; this.p = start; this.end = end; this.acc = 0; this.n = 0; }
        read(n) {
            if (n < 0 || n > 32) throw new Error('pmini invalid bit count');
            while (this.n < n) {
                if (this.p >= this.end) throw eof();
                this.acc += this.b[this.p++] * 2 ** this.n; this.n += 8;
            }
            const v = n ? this.acc % 2 ** n : 0;
            this.acc = Math.floor(this.acc / 2 ** n); this.n -= n;
            return v;
        }
        gamma() {
            let z = 0;
            while (this.read(1) === 0) { if (++z > 32) throw new Error('pmini bad gamma'); }
            return 2 ** z + this.read(z) - 1;
        }
        big() { const n = this.read(6); if (n > 32) throw new Error('pmini value wider than 32 bits'); return this.read(n); }
        finish() { if (this.p !== this.end || this.acc) throw new Error('pmini bit stream not consumed'); }
    }
    class PMiniRans {
        constructor(b, start, end) {
            this.b = b; this.start = start; this.p = end; this.x = null;
            if (end > start) { if (end - start < 3) throw new Error('pmini rANS stream too short');
                this.x = b[end - 1] * 65536 + b[end - 2] * 256 + b[end - 3]; this.p = end - 3; }
        }
        finish() { if (this.x !== null && (this.x !== 65536 || this.p !== this.start)) throw new Error('pmini rANS state'); }
    }
    function pminiLatents(br, rr, count) {
        if (count > 4096) throw new Error('pmini latent count');
        const k = br.read(2), nu = br.gamma() + 1;
        if (nu > 136 || nu > count) throw new Error('pmini bucket count');
        const max = 2 ** k + ((31 - k) << k) + 2 ** k - 1, used = [];
        let prev = -1;
        for (let i = 0; i < nu; i++) { prev += 1 + br.gamma(); if (prev > max) throw new Error('pmini bucket id'); used.push(prev); }
        const cnt = [], rem0 = count;
        let rem = rem0;
        for (let i = 0; i < nu - 1; i++) { const c = br.gamma() + 1; cnt.push(c); rem -= c; }
        if (rem < 1) throw new Error('pmini frequency table'); cnt.push(rem);
        const fr = cnt.map(c => Math.max(1, Math.floor((c * 4096) / count)));
        let largest = 0; for (let i = 1; i < fr.length; i++) if (fr[i] > fr[largest]) largest = i;
        fr[largest] += 4096 - fr.reduce((a, x) => a + x, 0); if (fr[largest] <= 0) throw new Error('pmini frequencies');
        const lut = new Uint8Array(4096), cum = []; let at = 0;
        for (let i = 0; i < nu; i++) { cum.push(at); lut.fill(i, at, at + fr[i]); at += fr[i]; }
        if (rr.x === null) throw new Error('pmini missing rANS');
        let x = rr.x, p = rr.p; const out = [];
        for (let q = 0; q < count; q++) {
            const slot = x % 4096, i = lut[slot]; x = fr[i] * Math.floor(x / 4096) + slot - cum[i];
            while (x < 65536) { if (--p < rr.start) throw new Error('pmini rANS overrun'); x = x * 256 + rr.b[p]; }
            const s = used[i];
            if (s < 2 ** k) out.push(s);
            else { const t = s - 2 ** k, e = Math.floor(t / 2 ** k); out.push(((2 ** k + (t % 2 ** k)) * 2 ** e) + br.read(e)); }
        }
        rr.x = x; rr.p = p; return out;
    }
    function pminiSimple(br, rr, t, n) {
        if (t === 1) { const mn = br.big(); return pminiLatents(br, rr, n).map(x => (x + mn) >>> 0); }
        if (t === 2) {
            let v = br.big() >>> 0, out = [v];
            if (n > 1) for (const u of pminiLatents(br, rr, n - 1)) { const d = (u % 2) ? -Math.floor((u + 1) / 2) : Math.floor(u / 2); v = (v + d) >>> 0; out.push(v); }
            return out;
        }
        if (t === 4) {
            const nd = br.gamma() + 1; if (nd > 32) throw new Error('pmini dictionary size');
            const keys = [br.big()]; for (let i = 1; i < nd; i++) keys.push(keys[i - 1] + 1 + br.gamma());
            return pminiLatents(br, rr, n).map(i => { if (i >= nd) throw new Error('pmini dictionary index'); return keys[i]; });
        }
        throw new Error('pmini transform tag');
    }
    function pminiInspect(blob) {
        const b = blob instanceof Uint8Array ? blob : new Uint8Array(blob);
        if (b.length < 10 || b.length > PMA2_MAX_PACKET || b[0] !== 0x4D || b[1] !== 0x54 || b[2] !== 0x50 || b[3] !== 0x32) throw new Error('not a pmini MTP2 shard');
        const want = new DataView(b.buffer, b.byteOffset + b.length - 4, 4).getUint32(0, true);
        if (crc32(b, b.length - 4) !== want) throw new Error('pmini CRC-32 mismatch');
        const end = b.length - 4; let p = 4, q = pminiVarint(b, p, end); const rows = q.value; p = q.pos;
        q = pminiVarint(b, p, end);
        if (rows > 4096) throw new Error('pmini rows exceed MAX_ROWS');
        return {rows, cols: q.value};
    }
    function pminiDecodeWords(blob, ncols) {
        const b = blob instanceof Uint8Array ? blob : new Uint8Array(blob);
        const shape = pminiInspect(b);
        const end = b.length - 4; let p = 4, q = pminiVarint(b, p, end); const rows = q.value; p = q.pos;
        q = pminiVarint(b, p, end); const cols = q.value; p = q.pos;
        if (rows !== shape.rows || cols !== shape.cols) throw new Error('pmini header changed');
        if (rows > 4096 || cols !== ncols) throw new Error('pmini dimensions');
        const blocks = []; let e = end;
        while (e > p) { if (e - 6 < p) throw new Error('pmini trailer'); const dv = new DataView(b.buffer, b.byteOffset + e - 6, 6);
            const lb = dv.getUint16(0, true), lr = dv.getUint16(2, true), bc = dv.getUint16(4, true), s = e - 6 - lb - lr;
            if (s < p || !bc) throw new Error('pmini bad trailer'); blocks.push([s, lb, lr, bc]); e = s; }
        blocks.reverse(); if (!rows) { if (blocks.length) throw new Error('pmini empty blocks'); return []; }
        if (blocks.reduce((a, x) => a + x[3], 0) !== cols) throw new Error('pmini block columns');
        const out = [], gaps = new Map();
        for (const [s, lb, lr, bc] of blocks) { const br = new PMiniBits(b, s, s + lb), rr = new PMiniRans(b, s + lb, s + lb + lr);
            for (let z = 0; z < bc; z++) { const ci = out.length, t = br.read(3);
                if (t === 0) { out.push(Array(rows).fill(br.big())); continue; }
                if (t === 5 || t === 6) { const bg = br.big(); let gs;
                    if (t === 5) { const m = br.gamma() + 1; if (m >= rows) throw new Error('pmini split count'); gs = pminiLatents(br, rr, m); gaps.set(ci, gs); }
                    else { const ref = ci - br.gamma() - 1; if (!gaps.has(ref)) throw new Error('pmini same reference'); gs = gaps.get(ref); }
                    const fg = pminiSimple(br, rr, br.read(3), gs.length), vals = Array(rows).fill(bg); let at = -1;
                    for (let k = 0; k < gs.length; k++) { at += gs[k] + 1; if (at >= rows) throw new Error('pmini split position'); vals[at] = fg[k]; } out.push(vals);
                } else out.push(pminiSimple(br, rr, t, rows));
            } br.finish(); rr.finish(); }
        return Array.from({length: rows}, (_, r) => out.map(c => c[r] >>> 0));
    }

    function pma2Varint(v) {
        const out = [];
        do { const b = Number(v & 0x7Fn); v >>= 7n; out.push(b | (v ? 0x80 : 0)); } while (v);
        return out;
    }
    function pma2Decode(payload) {
        const b = payload instanceof Uint8Array ? payload : new Uint8Array(payload);
        if (b.length < 9 || b.length > PMA2_MAX_PACKET || String.fromCharCode(...b.subarray(0, 4)) !== 'PMA2') throw new Error('PMA2 bad magic or truncation');
        const dv = new DataView(b.buffer, b.byteOffset + b.length - 4, 4);
        if (crc32(b, b.length - 4) !== dv.getUint32(0, true)) throw new Error('PMA2 CRC-32 mismatch');
        const end = b.length - 4; let p = 4;
        if (b[p++] !== 1) throw new Error('PMA2 unsupported version');
        let q = pminiVarint(b, p, end); const events = q.value; p = q.pos;
        if (events < 1 || events > PMA2_MAX_EVENTS) throw new Error('PMA2 invalid event count');
        q = pminiVarint(b, p, end); const prefixLen = q.value; p = q.pos;
        if (prefixLen < 1 || prefixLen > PMA2_MAX_PREFIX || prefixLen > end - p) throw new Error('PMA2 invalid prefix length');
        const prefix = b.slice(p, p + prefixLen); p += prefixLen;
        let j = 0; if (prefix[j++] !== 1) throw new Error('PMA2 prefix is not a data frame');
        q = pminiVarint(prefix, j, prefix.length); if (!q.value) throw new Error('PMA2 zero measurement SID'); j = q.pos;
        for (;;) { q = pminiVarint(prefix, j, prefix.length); j = q.pos; if (!q.value) break;
            q = pminiVarint(prefix, j, prefix.length); j = q.pos; if (!q.value) throw new Error('PMA2 zero tag SID'); }
        if (j !== prefix.length) throw new Error('PMA2 prefix trailing bytes');
        q = pminiVarint(b, p, end); const unionCount = q.value; p = q.pos;
        if (unionCount < 1 || unionCount > PMA2_MAX_UNION) throw new Error('PMA2 invalid union count');
        const union = [], seen = new Set();
        for (let k = 0; k < unionCount; k++) { q = pminiVarint(b, p, end); const sid = q.value; p = q.pos;
            if (!sid || sid > MAX_SYMBOLS || seen.has(sid) || p >= end) throw new Error('PMA2 invalid union SID');
            const dt = b[p++]; if (!DT_WIDTH[dt] || dt === DT.STR || DT_WIDTH[dt] > 4) throw new Error('PMA2 unsupported union datatype');
            seen.add(sid); union.push([sid, dt]); }
        const desc = [];
        for (let k = 0; k < events; k++) { if (p >= end) throw eof(); const kind = b[p++];
            if (kind === 1) { desc.push(null); continue; }
            if (kind !== 0) throw new Error('PMA2 unknown event descriptor');
            q = pminiVarint(b, p, end); const n = q.value; p = q.pos;
            if (n < 1 || n > MAX_RECORD || n > end - p) throw new Error('PMA2 invalid table literal length');
            const lit = b.slice(p, p + n); p += n; q = pminiVarint(lit, 0, lit.length);
            if (q.pos + q.value !== lit.length || q.value < 1 || lit[q.pos] !== 2) throw new Error('PMA2 invalid table literal');
            desc.push(lit); }
        const shape = pminiInspect(b.subarray(p, end)), dataCount = desc.filter(x => x === null).length;
        if (shape.rows !== dataCount || shape.cols !== 3 + union.length || shape.rows > PMA2_MAX_ROWS)
            throw new Error('PMA2 row/event or column count mismatch');
        const rows = pminiDecodeWords(b.subarray(p, end), 3 + union.length);
        if (!dataCount || dataCount > PMA2_MAX_ROWS || rows.length !== dataCount) throw new Error('PMA2 row/event count mismatch');
        const raw = [], add = (x) => { for (const y of x) raw.push(y); };
        let ri = 0;
        for (const item of desc) {
            if (item !== null) { add(item); continue; }
            const row = rows[ri++], lo = row[0], hi = row[1], mask = row[2];
            const validMask = union.length === 32 ? 0xFFFFFFFF : 2 ** union.length - 1;
            if ((mask >>> 0) & (~validMask >>> 0)) throw new Error('PMA2 presence mask has unknown bits');
            for (let bit = 0; bit < union.length; bit++) if (!(mask & (2 ** bit)) && row[3 + bit] !== 0)
                throw new Error('PMA2 absent field word is nonzero');
            const frame = Array.from(prefix);
            for (let bit = 0; bit < union.length; bit++) if (mask & (2 ** bit)) {
                const [sid, dt] = union[bit], width = DT_WIDTH[dt], word = row[3 + bit] >>> 0;
                if (width < 4 && Math.floor(word / 2 ** (8 * width))) throw new Error('PMA2 field word overflow');
                frame.push(...pma2Varint(BigInt(sid))); for (let z = 0; z < width; z++) frame.push((word / 2 ** (8 * z)) & 0xFF);
            }
            frame.push(0); frame.push(...pma2Varint(BigInt(lo) | (BigInt(hi) << 32n))); add(pma2Varint(BigInt(frame.length))); add(frame);
        }
        return Uint8Array.from(raw);
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
            if (n >= MAX_TAMP_OUT) throw new Error('tamp: output over 64 KB');
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
    // A data frame referencing a SID the table lacks (joined mid-stream, table frame lost) is
    // skipped rather than aborting the payload: the next table frame may follow in the same batch.
    function frameDecodable(f, j, table) {
        const vi = () => { const x = readVarint(f, j); j = x.pos; return x.value; };
        if (!table.has(vi())) return false;                       // measurement
        while (j < f.length && f[j] !== 0) { if (!table.has(vi()) || !table.has(vi())) return false; }
        j++;
        while (j < f.length && f[j] !== 0) {
            const e = table.get(vi());
            if (!e || !DT_WIDTH[e[1]]) return false;           // unknown SID or field type
            j += DT_WIDTH[e[1]];
        }
        return true;
    }

    function decodePayload(cid, payload, table, out = []) {
        if (cid === 2) {
            // Decode PMA2 and the reconstructed raw frames against a copy. A
            // malformed packet must not leave a partially learned symbol table.
            const local = new Map(table), tmp = [];
            let raw;
            try { raw = pma2Decode(payload); }
            catch (e) {
                // Keep the historical unknown-cid diagnostic useful to callers
                // that probe ids while still retaining the PMA2 failure detail.
                if (/bad magic|truncation/.test(String(e.message))) throw new Error(`unknown compressor id 2: ${e.message}`);
                throw e;
            }
            decodePayload(0, raw, local, tmp);
            table.clear(); for (const [sid, entry] of local) table.set(sid, entry);
            out.push(...tmp);
            return out;
        }
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
                    if (!table.has(sid) && table.size >= MAX_SYMBOLS) table.clear();
                    table.set(sid, [s.value, peek()]); j++;
                }
            } else if (ft === 1) {
                // table not seen yet, or a field type this decoder lacks: skip this frame
                if (!frameDecodable(f, j, table)) continue;
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
                const badCid2Header = buf.length > i && buf[i] === 2 &&
                    (n < 5 || (buf.length >= i + n && String.fromCharCode(...buf.subarray(i + 1, i + 5)) !== 'PMA2'));
                if (n < 1 || n > MAX_RECORD || (buf.length > i && buf[i] !== 0 && buf[i] !== 1 && buf[i] !== 2) || badCid2Header) {
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
    //   sc: t=<epoch_s> n=<N> mpp=<W>,<V> sweeping=0|1 [flag=0|light land=<W>]
    //   sc: V P V P ...      (repeated)
    //   sc: end   |   sc: none
    const SC_RE = /(?:^|[^A-Za-z0-9_])sc:\s*(.*?)\s*$/;
    const NUM_RE = /^[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?$|^[-+]?(?:inf|nan)$/i;
    const scNum = (tok) => /^[-+]?inf$/i.test(tok) ? (tok[0] === '-' ? -Infinity : Infinity)
        : /nan$/i.test(tok) ? NaN : Number(tok);

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
                mpp: {P: scNum(mpp[0]), V: scNum(mpp[1])}, sweeping: kv.sweeping === '1'};
            if (kv.flag !== undefined) {                          // sweep verdict: '' clean, else the reason
                h.flag = kv.flag === '0' ? '' : kv.flag;
                h.land = NUM_RE.test(kv.land || '') ? scNum(kv.land) : NaN;
            }
            if (!Number.isFinite(h.t) || !Number.isInteger(h.n) || h.n < 0 || mpp.length !== 2 ||
                !NUM_RE.test(mpp[0]) || !NUM_RE.test(mpp[1])) return {kind: 'invalid', text: body};
            return h;
        }
        const vals = [];
        let truncated = false;
        for (const tok of body ? body.split(/\s+/) : []) {
            if (!NUM_RE.test(tok)) { truncated = true; break; }
            vals.push(scNum(tok));
        }
        if (vals.length % 2) truncated = true;
        const points = [];
        let dropped = 0;
        for (let i = 0; i + 1 < vals.length; i += 2) {
            if (Number.isFinite(vals[i]) && Number.isFinite(vals[i + 1])) points.push({V: vals[i], P: vals[i + 1]});
            else dropped++;                                       // inf/nan pair
        }
        if (!points.length && !dropped && !truncated) return {kind: 'invalid', text: body};
        return {kind: 'data', points, dropped, truncated};
    }

    // Assembles one curve from header, data and `end` lines. feed() returns undefined until a
    // curve finishes, then {complete, curve, reason}; `sc: none` gives {complete: true, curve: null}.
    class SweepCurveParser {
        constructor() { this.cur = null; }

        _finish(reason) {
            const c = this.cur;
            this.cur = null;
            const curve = {t: c.t, n: c.n, mpp: c.mpp, sweeping: c.sweeping, points: c.points};
            if (c.flag !== undefined) curve.flag = c.flag, curve.land = c.land;
            if (!reason && c.points.length + c.dropped !== c.n) reason = 'short';
            if (!reason && c.truncated) reason = 'truncated';
            return reason ? {complete: false, curve, reason} : {complete: true, curve};
        }

        feed(line) {
            const p = parseScLine(line);
            if (!p) return undefined;
            switch (p.kind) {
                case 'header': {
                    const prev = this.cur ? this._finish('restart') : undefined;
                    this.cur = {t: p.t, n: p.n, mpp: p.mpp, sweeping: p.sweeping, flag: p.flag, land: p.land,
                        points: [], dropped: 0, truncated: false};
                    return prev;
                }
                case 'data':
                    if (this.cur) {
                        this.cur.points.push(...p.points);
                        this.cur.dropped += p.dropped;
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
        RECORD_MAGIC, MAX_RECORD, MAX_SYMBOLS, MAX_TAMP_OUT, DT, DT_WIDTH,
        readVarint, cstr, f16ToF32, fmtG, tampInitDictionary, tampDecompress,
        decodePayload, pma2Decode, pminiInspect, pminiDecodeWords, pointToLine, TeleStream, stripAnsi, parseScLine, SweepCurveParser,
    };
});
