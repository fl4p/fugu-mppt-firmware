// MIT License, Copyright (c) 2026 Fabian Schlieper. See LICENSE.
// MTP2 decoder. Words are unsigned uint32; decodeWords returns row-major arrays.
(function (root, factory) {
    if (typeof module === 'object' && module.exports) module.exports = factory();
    else root.PMini = factory();
})(typeof self !== 'undefined' ? self : this, function () {
    'use strict';
    function eof() { return new RangeError('unexpected end of data'); }
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
            if (i >= end) throw new Error('pmini truncated varint');
            const x = b[i++];
            if (!first && x === 0 && s) throw new Error('pmini non-canonical varint');
            first = false;
            v += (x & 0x7F) * 2 ** s;
            if (!(x & 0x80)) return {value: v, pos: i};
            s += 7;
            if (s > 35) throw new Error('pmini varint too long');
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
        if (b.length < 10 || b[0] !== 0x4D || b[1] !== 0x54 || b[2] !== 0x50 || b[3] !== 0x32) throw new Error('not a pmini MTP2 shard');
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

    return {inspect: pminiInspect, decodeWords: pminiDecodeWords};
});
