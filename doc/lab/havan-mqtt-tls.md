*this document is an LLM generated placeholder*

# MQTT over TLS on havan (wss for the hosted dashboard)

Set up 2026-11-19. Lab-specific, not published.

The Mosquitto add-on (`core_mosquitto` 7.1.3) on havan maps container ports 1883/1884/8883/8884 to host ports
1882/1885/8883/8884. Its options already named `certfile: fullchain.pem`, `keyfile: privkey.pem`; the files were
simply missing from HA's `/ssl` (`/var/lib/homeassistant/ssl` on the host), so 8883/8884 answered without TLS.

## Certificates

A private CA and a server certificate live in `/var/lib/homeassistant/ssl/`:

| file | what |
|---|---|
| `fugu-lab-ca.crt` / `.key` | CA "fugu lab CA", EC P-256, valid 10 years from 2026-11-19 |
| `fullchain.pem` / `privkey.pem` | server cert `CN=havan`, SANs `IP:192.168.1.200`, `IP:100.64.0.10`, `DNS:havan.farm.internal`, `DNS:havan.local`, `DNS:havan`; expires 2029-02-21 |

Renew or re-issue with the script below (run on havan as fab, needs `sudo -n`), then `sudo ha apps restart core_mosquitto`.

```bash
D=/tmp/fugu-ca; mkdir -p $D; cd $D
sudo cp /var/lib/homeassistant/ssl/fugu-lab-ca.{crt,key} .   # reuse the CA; omit to mint a new one (then re-install it on clients)
openssl req -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -nodes -keyout privkey.pem -out srv.csr -subj "/CN=havan"
printf 'subjectAltName=IP:192.168.1.200,IP:100.64.0.10,DNS:havan.farm.internal,DNS:havan.local,DNS:havan\nextendedKeyUsage=serverAuth\nbasicConstraints=CA:FALSE\n' > san.cnf
openssl x509 -req -in srv.csr -CA fugu-lab-ca.crt -CAkey fugu-lab-ca.key -CAcreateserial -out srv.crt -days 825 -extfile san.cnf
cat srv.crt fugu-lab-ca.crt > fullchain.pem
sudo install -m 644 fullchain.pem /var/lib/homeassistant/ssl/fullchain.pem
sudo install -m 600 privkey.pem /var/lib/homeassistant/ssl/privkey.pem
```

## Clients

- Dashboard (hosted https copy or local): `wss://192.168.1.200:8884` on the LAN, `wss://100.64.0.10:8884` over
  Tailscale. The discovery probe tries 8884 first on an https page.
- Install `fugu-lab-ca.crt` once per device: macOS Keychain (System, trust "Always"), Android Settings → Security →
  Install a certificate → CA certificate (Chrome honours user CAs), iOS: open the file, install the profile, then
  Settings → General → About → Certificate Trust Settings → enable full trust.
- Scripted clients: `paho` with `tls_set(ca_certs="fugu-lab-ca.crt")` on 8883 (mqtts) or `transport="websockets"` on 8884.
- Verify from havan: `openssl s_client -connect 127.0.0.1:8884 -CAfile /var/lib/homeassistant/ssl/fugu-lab-ca.crt </dev/null`.

The plain listeners 1882 (MQTT) and 1885 (ws) are unchanged; the converters keep using 1882.

## Plain-http copy on havan (no CA needed)

Fab declined to install the private CA (2026-11-19). The dashboard is served instead from Home Assistant on havan,
which listens on plain http port 80 and serves its `www` folder under `/local/`:

```
scp etc/config-tool/dashboard.html etc/config-tool/teledec.js havan-ts:/tmp/
ssh havan-ts sudo -n install -m 644 /tmp/dashboard.html /tmp/teledec.js /var/lib/homeassistant/homeassistant/www/fugu/
```

URL on the LAN: `http://192.168.1.200/local/fugu/dashboard.html?mqtt=ws://192.168.1.200:1885&user=pv`
Over Tailscale: `http://100.64.0.10/local/fugu/dashboard.html?mqtt=ws://100.64.0.10:1885&user=pv`

An http page can open `ws://`, so the 8884 wss listener stays unused. Web Bluetooth is unavailable on an http page
(secure context required); BLE cards need the hosted https copy or `localhost`. Re-copy after dashboard changes.
