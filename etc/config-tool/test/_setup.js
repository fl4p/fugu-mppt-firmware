// Test harness: load conf-editor.html into jsdom with the two CDN dependencies
// (mqtt.js, ansi-to-html) replaced by inline stubs and a tiny block of test
// hooks appended to the main script so tests can reach script-scoped bindings.
const fs = require('node:fs');
const path = require('node:path');
const { JSDOM } = require('jsdom');

const HTML_PATH = process.env.CONF_EDITOR_HTML ? path.resolve(process.env.CONF_EDITOR_HTML) : path.resolve(__dirname, '..', 'conf-editor.html');

// Stub: capture publish/subscribe calls, fire 'connect' on next microtask.
// String.raw so the embedded JS keeps its own escape sequences intact.
const STUBS = String.raw`<script>
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
      reconnect(){ this.reconnects=(this.reconnects||0)+1; Promise.resolve().then(()=>this.emit('connect')); return this; },
    };
    window._mqttClients = (window._mqttClients||[]).concat(client);
    if (window._mqttHook) window._mqttHook(client);   // set in beforeParse: reach clients made during load
    Promise.resolve().then(()=>client.emit('connect'));
    return client;
  }
};
window.ansiConvert = {
  toHtml(text){
    if (!/\x1b\[/.test(text)) return text.replace(/[&<>]/g, c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
    const clean = text.replace(/\x1b\[[0-9;]*m/g, '');
    return '<span style="color:#0a0">' + clean.replace(/[&<>]/g, c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c])) + '</span>';
  }
};
</script>
`;

// Appended inside the main script (same lexical scope), so we can reach the
// script-local const/let bindings the spec talks about (`lineListeners`,
// `state`, `activeWrite`, `lastTransport`, `mqttClient`, `mqttDeviceHost`).
const TEST_HOOKS = `
window._lineListeners = lineListeners;
window._dispatchLine = (ln) => lineListeners.forEach(fn => fn(ln));
window.onBleNotify = onBleNotify;
window._tables = { META, FILE_META, FILE_KEYS, TYPE_KEYS, DEFAULTS, DEPENDS };
Object.defineProperty(window, "_state",          { get: () => state });
Object.defineProperty(window, "_activeWrite",    { get: () => activeWrite, set: v => { activeWrite = v; } });
Object.defineProperty(window, "_lastTransport",  { get: () => lastTransport, set: v => { lastTransport = v; } });
Object.defineProperty(window, "_mqttClient",     { get: () => mqttClient });
Object.defineProperty(window, "_mqttDeviceHost", { get: () => mqttDeviceHost });
Object.defineProperty(window, "_live",           { get: () => live, set: v => { live = v; } });
`;

// teledec: inline teledec.js where the page loads it (jsdom fetches no subresources, so without
// this the page runs as if teledec.js were missing).
function buildHtml({ teledec = false } = {}) {
  let html = fs.readFileSync(HTML_PATH, 'utf8');
  const tag = '<script src="teledec.js"></script>';
  html = html.replace(tag, teledec
    ? '<script>\n' + fs.readFileSync(path.resolve(path.dirname(HTML_PATH), 'teledec.js'), 'utf8') + '\n</script>' : '');
  // drop the two CDN scripts — we provide stubs instead
  html = html.replace(/\s*<script src="https:\/\/[^"]+"[^>]*><\/script>/g, '');
  html = html.replace(/\s*<script type="module">[\s\S]*?<\/script>/g, '');
  // inject stubs immediately before the main inline script
  html = html.replace('<script>\n"use strict";', STUBS + '<script>\n"use strict";');
  // append test hooks at the *end* of the main inline script so they execute in
  // the same lexical scope as `lineListeners` & friends.
  const lastClose = html.lastIndexOf('</script>');
  html = html.slice(0, lastClose) + TEST_HOOKS + html.slice(lastClose);
  return html;
}

// opts.url: page URL (a handoff fragment runs at load); opts.storage: localStorage seeded before
// the script runs; opts.beforeParse(window): more setup before the script runs (navigator.bluetooth).
async function loadEditor(opts = {}) {
  const dom = new JSDOM(buildHtml(opts), {
    url: opts.url || 'http://localhost/',
    runScripts: 'dangerously',
    pretendToBeVisual: true,
    beforeParse(window) {
      for (const [k, v] of Object.entries(opts.storage || {})) window.localStorage.setItem(k, v);
      if (opts.beforeParse) opts.beforeParse(window);
      // jsdom v24 doesn't expose these on window by default; the page treats
      // them as globals (`new TextDecoder()` at top level of the script).
      window.TextEncoder = TextEncoder;
      window.TextDecoder = TextDecoder;
      if (typeof Blob !== 'undefined')                  window.Blob = Blob;
      if (typeof DecompressionStream !== 'undefined')   window.DecompressionStream = DecompressionStream;
    },
  });
  // drain microtasks (mqtt connect callback, etc.)
  await new Promise(r => setTimeout(r, 0));
  return dom;
}

// jsdom doesn't lay out, so scroll geometry is always zero. Tests that exercise
// the tail-follow logic patch these properties to drive the < 4 px check.
function fakeScrollGeometry(el, { scrollHeight, clientHeight, scrollTop = 0 }) {
  Object.defineProperty(el, 'scrollHeight', { configurable: true, value: scrollHeight });
  Object.defineProperty(el, 'clientHeight', { configurable: true, value: clientHeight });
  el.scrollTop = scrollTop;
}

module.exports = { loadEditor, buildHtml, fakeScrollGeometry };
