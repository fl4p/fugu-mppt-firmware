// Test harness for dashboard.html: load it into jsdom with the mqtt.js CDN script replaced by a
// recording stub, teledec.js inlined, and a scriptable fake navigator.bluetooth.
const fs = require('node:fs');
const path = require('node:path');
const { JSDOM } = require('jsdom');

const DIR = path.resolve(__dirname, '..');
const HTML_PATH = path.join(DIR, 'dashboard.html');

// Same recording stub as _setup.js; `connect` fires on the next microtask.
const MQTT_STUB = String.raw`<script>
window.mqtt = {
  connect(url, opts){
    const client = {
      url, opts, ended:false,
      _listeners:new Map(), _publishes:[], _subscribes:[],
      on(ev, fn){ let a=this._listeners.get(ev); if(!a) this._listeners.set(ev, a=[]); a.push(fn); return this; },
      once(ev, fn){ const w=(...a)=>{ this.off(ev,w); fn(...a); }; return this.on(ev, w); },
      off(ev, fn){ const a=this._listeners.get(ev); if(!a) return this; const i=a.indexOf(fn); if(i>=0) a.splice(i,1); return this; },
      removeAllListeners(ev){ if(ev) this._listeners.delete(ev); else this._listeners.clear(); return this; },
      emit(ev, ...args){ (this._listeners.get(ev)||[]).slice().forEach(fn=>fn(...args)); return this; },
      publish(topic, msg, opts, cb){ this._publishes.push({topic, msg:String(msg), opts}); if(cb) cb(); return this; },
      subscribe(topic, opts, cb){ this._subscribes.push(topic); if(cb) cb(); return this; },
      end(){ this.ended=true; return this; },
    };
    window._mqttClients = (window._mqttClients||[]).concat(client);
    Promise.resolve().then(()=>client.emit('connect'));
    return client;
  }
};
</script>
`;

function buildHtml({ mqtt = true } = {}) {
  let html = fs.readFileSync(HTML_PATH, 'utf8');
  html = html.replace(/\s*<script src="https:\/\/[^"]+"[^>]*><\/script>/g, '');
  const teledec = fs.readFileSync(path.join(DIR, 'teledec.js'), 'utf8');
  html = html.replace('<script src="teledec.js"></script>',
    (mqtt ? MQTT_STUB : '') + '<script>\n' + teledec + '\n</script>');
  return html;
}

const UUID = {
  SVC: '6e400001-b5a3-f393-e0a9-e50e24dcca9e',
  RX: '6e400002-b5a3-f393-e0a9-e50e24dcca9e',
  TX: '6e400003-b5a3-f393-e0a9-e50e24dcca9e',
  TELE: '6e400005-b5a3-f393-e0a9-e50e24dcca9e',
};

function fakeChar(uuid) {
  const ls = [];
  return {
    uuid, notifying: false,
    addEventListener(t, fn) { if (t === 'characteristicvaluechanged') ls.push(fn); },
    removeEventListener(t, fn) { const i = ls.indexOf(fn); if (i >= 0) ls.splice(i, 1); },
    async startNotifications() { this.notifying = true; return this; },
    async stopNotifications() { this.notifying = false; return this; },
    notify(bytes) {
      const u = bytes instanceof Uint8Array ? bytes : new TextEncoder().encode(bytes);
      const value = new DataView(u.buffer, u.byteOffset, u.byteLength);
      ls.slice().forEach(fn => fn({ target: { value } }));
    },
  };
}

function defaultReply(cmd, opts) {
  if (cmd === 'hostname') return opts.hostname ? [`I (5) main: Hostname: ${opts.hostname}`, 'OK: hostname'] : [];
  if (cmd.startsWith('set-time ')) return [`OK: ${cmd}`];
  if (cmd === 'tele-ble 1') return ['tele-ble: on (dropped 0 B)', 'OK: tele-ble 1'];
  if (cmd === 'tele-ble') return ['tele-ble: on (dropped 7 B)', 'OK: tele-ble'];
  if (cmd === 'sweep-curve') return ['sc: none', 'OK: sweep-curve'];
  return [`OK: ${cmd}`];
}

// A scriptable NUS peripheral. `reply(cmd)` returns the console lines the device prints.
function makeFakeBleDevice({ name = 'fugu-flat', id = 'dev1', hostname = 'flat', tele = true, reply, failConnect = 0 } = {}) {
  const opts = { hostname };
  const discLs = [];
  const dev = {
    id, name, writes: [], slices: [], failConnect,
    tx: fakeChar(UUID.TX), rx: null, tele: tele ? fakeChar(UUID.TELE) : null,
    addEventListener(t, fn) { if (t === 'gattserverdisconnected') discLs.push(fn); },
    removeEventListener() {},
    dropLink() { dev.gatt.connected = false; discLs.forEach(fn => fn({ target: dev })); },
    // send console lines as notifications cut into 20-byte pieces
    print(lines) {
      const b = new TextEncoder().encode(lines.map(l => l + '\r\n').join(''));
      for (let i = 0; i < b.length; i += 20) dev.tx.notify(b.slice(i, i + 20));
    },
  };
  let rxBuf = '';
  dev.rx = {
    uuid: UUID.RX,
    async writeValueWithResponse(data) {
      const u = new Uint8Array(data.buffer ? data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength) : data);
      dev.slices.push(u.length);
      rxBuf += new TextDecoder().decode(u);
      let i;
      while ((i = rxBuf.indexOf('\n')) >= 0) {
        const cmd = rxBuf.slice(0, i).replace(/\r$/, '');
        rxBuf = rxBuf.slice(i + 1);
        dev.writes.push(cmd);
        const lines = (reply && reply(cmd, opts)) || defaultReply(cmd, opts);
        setTimeout(() => dev.print(lines), 0);
      }
    },
  };
  const svc = {
    async getCharacteristic(u) {
      if (u === UUID.RX) return dev.rx;
      if (u === UUID.TX) return dev.tx;
      if (u === UUID.TELE && dev.tele) return dev.tele;
      throw new Error('NotFoundError: no characteristic ' + u);
    },
  };
  dev.gatt = {
    connected: false,
    async connect() {
      if (dev.failConnect > 0) { dev.failConnect--; throw new Error('Connection attempt failed'); }
      dev.gatt.connected = true;
      return { connected: true, async getPrimaryService(u) { if (u !== UUID.SVC) throw new Error('no svc'); return svc; }, disconnect() {} };
    },
    disconnect() { dev.gatt.connected = false; },
  };
  return dev;
}

function makeFakeBluetooth(devicesToOffer = []) {
  const bt = {
    requests: [], granted: [],
    async requestDevice(o) {
      bt.requests.push(o);
      const d = devicesToOffer.shift();
      if (!d) throw new Error('NotFoundError: User cancelled');
      bt.granted.push(d);
      return d;
    },
    async getDevices() { return bt.granted.slice(); },
  };
  return bt;
}

const doms = [];

async function loadDash({ storage = {}, bluetooth = makeFakeBluetooth(), mqtt = true, url = 'http://localhost/' } = {}) {
  const dom = new JSDOM(buildHtml({ mqtt }), {
    url,
    runScripts: 'dangerously',
    pretendToBeVisual: true,
    beforeParse(window) {
      window.TextEncoder = TextEncoder;
      window.TextDecoder = TextDecoder;
      if (!url.startsWith('file:')) for (const [k, v] of Object.entries(storage)) window.localStorage.setItem(k, v);
      if (bluetooth) Object.defineProperty(window.navigator, 'bluetooth', { value: bluetooth, configurable: true });
    },
  });
  doms.push(dom);
  await settle();
  return { dom, window: dom.window, Dash: dom.window.Dash, bluetooth };
}

function closeAll() { while (doms.length) doms.pop().window.close(); }

const settle = (ms = 0) => new Promise(r => setTimeout(r, ms));

// --- minimal encoder for the binary telemetry wire (src/tele/sym_line_protocol.h) ---------------
const DT = { STR: 0, U8: 3, U16: 5, U32: 7, F32: 9 };
const FIELD_DT = { mppt_state: DT.U8, pwm_duty: DT.U16, heap: DT.U32 };

function varint(n) {
  const out = [];
  do {
    let b = n % 128;
    n = Math.floor(n / 128);
    if (n) b |= 0x80;
    out.push(b);
  } while (n);
  return out;
}

function fieldBytes(dt, v) {
  const b = Buffer.alloc({ 3: 1, 5: 2, 7: 4, 9: 4 }[dt]);
  if (dt === DT.U8) b.writeUInt8(v);
  else if (dt === DT.U16) b.writeUInt16LE(v);
  else if (dt === DT.U32) b.writeUInt32LE(v);
  else b.writeFloatLE(v);
  return [...b];
}

// points: [{meas='mppt', tags={}, fields:{P:..}, ts}] -> Uint8Array payload (raw, no cid)
function encodePayload(points, enc = { syms: new Map() }) {
  const sym = (name, dt) => {
    if (!enc.syms.has(name)) enc.syms.set(name, { sid: enc.syms.size + 1, dt, sent: false });
    return enc.syms.get(name);
  };
  const out = [];
  const frame = (bytes) => out.push(...varint(bytes.length), ...bytes);
  for (const p of points) {
    const meas = sym(p.meas || 'mppt', DT.STR);
    const tags = Object.entries(p.tags || {}).map(([k, v]) => [sym(k, DT.STR), sym(v, DT.STR)]);
    const fields = Object.entries(p.fields).map(([k, v]) => [sym(k, FIELD_DT[k] ?? DT.F32), v]);
    const fresh = [...enc.syms.entries()].filter(([, s]) => !s.sent);
    if (fresh.length) {
      const t = [2];
      for (const [name, s] of fresh) { t.push(...varint(s.sid), ...Buffer.from(name), 0, s.dt); s.sent = true; }
      t.push(0);
      frame(t);
    }
    const d = [1, ...varint(meas.sid)];
    for (const [k, v] of tags) d.push(...varint(k.sid), ...varint(v.sid));
    d.push(0);
    for (const [s, v] of fields) d.push(...varint(s.sid), ...fieldBytes(s.dt, v));
    d.push(0, ...varint(p.ts));
    frame(d);
  }
  return new Uint8Array(out);
}

// MQTT message: <cid><payload>
const mqttMsg = (points, enc) => Buffer.from([0, ...encodePayload(points, enc)]);
// BLE record: <0x7E><varint len><cid><payload>
function bleRecord(points, enc) {
  const body = [0, ...encodePayload(points, enc)];
  return new Uint8Array([0x7E, ...varint(body.length), ...body]);
}

function fakeScrollGeometry(el, { scrollHeight, clientHeight, scrollTop = 0 }) {
  Object.defineProperty(el, 'scrollHeight', { configurable: true, value: scrollHeight });
  Object.defineProperty(el, 'clientHeight', { configurable: true, value: clientHeight });
  el.scrollTop = scrollTop;
}

module.exports = {
  loadDash, closeAll, settle, buildHtml, makeFakeBleDevice, makeFakeBluetooth, UUID,
  encodePayload, mqttMsg, bleRecord, varint, fakeScrollGeometry,
};
