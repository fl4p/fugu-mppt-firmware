---
name: project_wifi_reconnect
description: AP loss panic (mqtt_task log recursion fixed; real AP power-off reboot still unexplained), power-gated reconnect strands daytime reboots at 0.0.0.0, nettools crash without WiFi
metadata:
  type: project
---

## AP loss reboot and "joins the other WiFi"
2026-05-22 fixed partly; 2026-05-29 still open for real AP power-off. Symptom: after AP loss a device reboots, comes up while the router is down, takes the only AP left (weak "other wifi") and stays there (no roam-while-connected; on fresh boot `prevSsid` is empty so the stick logic never runs).
- Found cause (RESET_REASON=4 panic): stack overflow in `mqtt_task` — esp-mqtt `transport_base` errors re-entered `mqttLogCallback`, which republished on the dead socket, recursing. Fix: per-task re-entrancy guard in `mqttLogCallback` (a4cf19d, src/tele/mqtt.cpp). Optional services were ruled out by bisection.
- **Not fully fixed:** router `GET /scan` only bounces its upstream STA; its soft-AP stays up, so the fugu never disassociates. flat (63f599b, with the guard) still rebooted on a real AP power-off. Since reconnect is power-gated, that crash is in the background net stack (mqtt auto-reconnect / UDP tele / mDNS netif-down). Root cause unknown; needs a coredump ([[project_coredump]]). `etc/e2e-test/test_stick_to_wifi.py` has the same `/scan` blind spot.
- Bench trick to force a weak AP: `set-config wifi.conf ssid_<strong> zz_off`, reboot, restore. Control channel must be serial.

## Power-gated reconnect strands daytime reboots
2026-05-29. `wifiLoop()` only calls blocking `wait_for_wifi()` when `(converter.disabled() || _curPower < 10) && ucTemp < 80` (associating injects RT latency). A converter rebooting in good sun converts >10 W at once and sits at `0.0.0.0` until evening; `discover.py` returns `[]`, havan log goes silent.
- **How to apply:** a converter "off WiFi" midday — check uptime over BLE first; low uptime = it rebooted and is gated out, not a router fault. Recovery now needs a brief `dc 0` (power decision, ask Fab). flat RSSI -75..-83 vs fry -45. Related [[project_rt_loop_latency]].

## nettools need WiFi up
`curl`/`ping`/`nslookup`/`tcpconnect` post to the lwip tcpip thread, which exists only after WiFi came up; without it they hard-fault in `tcpip_send_msg_wait_sem`. Gated on `nettoolsNetUp()` (`WiFi.status()==WL_CONNECTED`); `netstat` is safe.
- **Why:** a wiped bench unit crashed on `ping 8.8.8.8`, where a graceful reject was assumed.
- **How to apply:** any non-RT code that resolves names or opens sockets checks the network is up first.
