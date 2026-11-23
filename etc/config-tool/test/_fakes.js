// Fake devices for the editor's jsdom tests: a firmware that answers hostname / get-config with the
// interim status line every command prints, an MQTT hook that answers pv/log/<host>/cmd, and a
// granted Web Bluetooth NUS device.
const STATUS = 'V=38.2/27.10 I=3.10/4.20A 118.4W 31℃40℃ 99sps  0㎅/s CCM(H|L|Lm)=1200|1100|1100 st=↑MPPT,1 lag=120㎲ N=1000 rssi=-60';
const CONF = { 'charger.conf': { vout_max: '28.8' }, 'mqtt.conf': { broker_uri: 'mqtt://broker' } };

// Fake firmware: hostname and get-config, then the interim status line every command prints.
function firmware(host, conf = CONF) {
  const sent = [];
  const handler = (cmd, say) => {
    sent.push(cmd);
    let g;
    if (cmd === 'hostname') say('Hostname: ' + host);
    else if ((g = cmd.match(/^get-config (\S+)$/))) {
      for (const [k, v] of Object.entries(conf[g[1]] || {})) say(`I (100) main: Conf '/littlefs/conf/${g[1]}:${k}' = '${v}'`);
      say('OK: get-config ' + g[1]);
      return;
    } else { say('ERR: ' + cmd); return; }
    say(STATUS);
    say('OK: ' + cmd);
  };
  return { handler, sent };
}

// MQTT: answer publishes on pv/log/<host>/cmd for every client, including those made during load.
function mqttHook(window, devs) {
  window._mqttHook = (c) => {
    const pub = c.publish.bind(c);
    c.publish = (topic, msg, po, cb) => {
      pub(topic, msg, po, cb);
      const m = topic.match(/^pv\/log\/([^/]+)\/cmd$/), d = m && devs[m[1]];
      if (d) setTimeout(() => d.handler(String(msg),
        s => c.emit('message', 'pv/log/' + m[1], new TextEncoder().encode(s + '\n'))), 1);
      return c;
    };
  };
}

// BLE: a granted NUS device. busy: gatt.connect always fails; advert: whether watchAdvertisements fires.
function bleDevice(window, { id = 'id-1', name = 'fugu-devB', fw, busy = false, advert } = {}) {
  const ls = [], advLs = [];
  const enc = new TextEncoder();
  const tx = {
    async startNotifications() {}, async stopNotifications() {},
    addEventListener(t, fn) { ls.push(fn); }, removeEventListener(t, fn) { const i = ls.indexOf(fn); if (i >= 0) ls.splice(i, 1); },
  };
  let buf = '';
  const rx = {
    async writeValueWithResponse(d) {
      buf += new TextDecoder().decode(d);
      let i;
      while ((i = buf.indexOf('\n')) >= 0) {
        const cmd = buf.slice(0, i).replace(/\r$/, ''); buf = buf.slice(i + 1);
        setTimeout(() => fw.handler(cmd, s => ls.slice().forEach(fn => fn({ target: { value: enc.encode(s + '\n') } }))), 1);
      }
    },
  };
  const dev = {
    id, name, connects: 0, disconnects: 0,
    addEventListener(t, fn) { if (t === 'advertisementreceived') advLs.push(fn); },
    removeEventListener(t, fn) { const i = advLs.indexOf(fn); if (i >= 0) advLs.splice(i, 1); },
    gatt: {
      connected: false,
      disconnect() { dev.disconnects++; dev.gatt.connected = false; },
      async connect() {
        dev.connects++;
        if (busy) throw new Error('Connection attempt failed.');
        dev.gatt.connected = true;
        return { get connected() { return dev.gatt.connected; }, disconnect() { dev.gatt.disconnect(); },
          async getPrimaryService() { return { async getCharacteristic(u) { return /0002-/.test(u) ? rx : tx; } }; } };
      },
    },
  };
  if (advert !== undefined) dev.watchAdvertisements = async () => { if (advert) setTimeout(() => advLs.slice().forEach(fn => fn({ device: dev })), 5); };
  return dev;
}
function fakeBluetooth(devices, { getDevices = true } = {}) {
  const bt = { requests: 0, async requestDevice() { bt.requests++; throw new Error('NotFoundError: User cancelled'); } };
  if (getDevices) bt.getDevices = async () => devices.slice();
  return bt;
}

module.exports = { STATUS, CONF, firmware, mqttHook, bleDevice, fakeBluetooth };
