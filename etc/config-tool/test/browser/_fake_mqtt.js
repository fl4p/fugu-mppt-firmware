// In-page fake of mqtt.js plus one converter on it, for browser tests and screenshots. Install with
// context.addInitScript(fakeMqtt, opts) and abort the unpkg CDN request so the real bundle does not
// replace it. The device answers hostname / get-config / any command with the firmware's interim
// status line and an OK marker, and logs a status line every `every` ms (0: never).
function fakeMqtt({ host = 'devA', every = 3000, conf = {} } = {}) {
  const enc = new TextEncoder();
  const status = () => `V=38.2/27.10 I=3.10/4.20A 118.4W 31℃40℃ 99sps  0㎅/s CCM(H|L|Lm)=1200|1100|1100 st=↑MPPT,1 lag=120㎲ N=1000 rssi=-60`;
  window.__mqttLog = [];
  window.mqtt = {
    connect(url, opts) {
      const ls = new Map();
      const c = {
        url, opts, connected: true,
        on(ev, fn) { (ls.get(ev) || ls.set(ev, []).get(ev)).push(fn); return c; },
        once(ev, fn) { const w = (...a) => { c.off(ev, w); fn(...a); }; return c.on(ev, w); },
        off(ev, fn) { const a = ls.get(ev) || []; const i = a.indexOf(fn); if (i >= 0) a.splice(i, 1); return c; },
        removeAllListeners(ev) { if (ev) ls.delete(ev); else ls.clear(); return c; },
        emit(ev, ...a) { (ls.get(ev) || []).slice().forEach(fn => fn(...a)); return c; },
        say(line) { c.emit('message', 'pv/log/' + host, enc.encode(line + '\n'), { retain: false }); },
        publish(topic, msg, o, cb) {
          window.__mqttLog.push([topic, String(msg)]);
          if (cb) cb();
          if (topic !== 'pv/log/' + host + '/cmd') return c;
          const cmd = String(msg); let g;
          setTimeout(() => {
            if (cmd === 'hostname') c.say('I (5) main: Hostname: ' + host);
            else if ((g = cmd.match(/^get-config (\S+)$/))) {
              for (const [k, v] of Object.entries(conf[g[1]] || {})) c.say(`I (100) main: Conf '/littlefs/conf/${g[1]}:${k}' = '${v}'`);
              c.say('I (100) main: OK: get-config ' + g[1]); return;
            }
            c.say(status()); c.say('I (6) main: OK: ' + cmd);
          }, 5);
          return c;
        },
        subscribe(t, o, cb) { if (cb) cb(null); return c; },
        unsubscribe() { return c; },
        end() { c.connected = false; clearInterval(c.tick); return c; },
      };
      setTimeout(() => c.emit('connect'), 5);
      if (every) c.tick = setInterval(() => c.say(status()), every);
      return c;
    },
  };
}

const CONF = {
  'board.conf': { mcu: 'esp32s3', pwm_freq: '39000' },
  'charger.conf': { vout_max: '28.8', cv_eoc: '3.55' },
  'mqtt.conf': { broker_uri: 'mqtt://broker', cmd_input: '1' },
};

async function installFakeMqtt(context, opts = {}) {
  await context.route('https://unpkg.com/**', r => r.abort());
  await context.addInitScript(fakeMqtt, { conf: CONF, ...opts });
}

module.exports = { fakeMqtt, installFakeMqtt, CONF };
