// A local wss:// MQTT stub for browser tests: an https server with a fresh self-signed certificate
// (made with openssl), a WebSocket listener on any path and enough MQTT 3.1.1 to answer CONNECT with a
// CONNACK return code (0 accepts, 5 = not authorized), SUBSCRIBE and PINGREQ. ws:false leaves only
// the https server: its TLS works but the WebSocket upgrade is dropped.
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const https = require('node:https');
const { execFileSync } = require('node:child_process');
const { WebSocketServer } = require('ws');

function haveOpenssl() {
  try { execFileSync('openssl', ['version'], { stdio: 'ignore' }); return true; } catch (_) { return false; }
}

function selfSigned() {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'wss-broker-'));
  const key = path.join(dir, 'key.pem'), cert = path.join(dir, 'cert.pem');
  execFileSync('openssl', ['req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '1', '-subj', '/CN=localhost',
    '-addext', 'subjectAltName=DNS:localhost,IP:127.0.0.1', '-keyout', key, '-out', cert], { stdio: 'ignore' });
  const r = { key: fs.readFileSync(key), cert: fs.readFileSync(cert) };
  fs.rmSync(dir, { recursive: true, force: true });
  return r;
}

// MQTT fixed header: type byte, remaining length (varint); returns [type, body] per packet.
function packets(buf) {
  const out = [];
  let i = 0;
  while (i < buf.length) {
    const type = buf[i++];
    let len = 0, mul = 1, b;
    do { b = buf[i++]; len += (b & 0x7f) * mul; mul *= 128; } while (b & 0x80);
    out.push([type, buf.subarray(i, i + len)]);
    i += len;
  }
  return out;
}

async function startWssBroker({ rc = 0, ws = true } = {}) {
  const st = { rc, connects: 0, upgrades: 0, gets: 0 };
  const srv = https.createServer(selfSigned(), (req, res) => { st.gets++; res.writeHead(404); res.end(); });
  srv.on('upgrade', () => { st.upgrades++; });
  if (ws) {
    const wss = new WebSocketServer({ server: srv, handleProtocols: p => (p.has('mqtt') ? 'mqtt' : false) });
    wss.on('connection', sock => sock.on('message', data => {
      for (const [type, body] of packets(Buffer.from(data))) {
        const t = type >> 4;
        if (t === 1) {
          st.connects++;
          sock.send(Buffer.from([0x20, 2, 0, st.rc]));
          if (st.rc) setTimeout(() => sock.close(), 20);
        } else if (t === 8) {
          let n = 0;
          for (let i = 2; i + 2 <= body.length; i += 3 + body.readUInt16BE(i)) n++;
          sock.send(Buffer.from([0x90, 2 + n, body[0], body[1], ...Array(n).fill(0)]));
        }
        else if (t === 12) sock.send(Buffer.from([0xd0, 0]));
      }
    }));
  } else srv.on('upgrade', (req, sock) => sock.destroy());
  await new Promise(r => srv.listen(0, '127.0.0.1', r));
  const port = srv.address().port;
  return {
    port, st, url: `wss://localhost:${port}`, origin: `https://localhost:${port}/`,
    close: () => new Promise(r => { srv.closeAllConnections && srv.closeAllConnections(); srv.close(() => r()); }),
  };
}

module.exports = { startWssBroker, haveOpenssl };
