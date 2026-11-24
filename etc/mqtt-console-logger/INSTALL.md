*this document is an LLM generated placeholder*

# MQTT console logger: install

`mqtt_console.py` records every device console line published on `pv/log/<host>` to `fugu_console.log`
(10 MB × 10 rotation). The oldest backup is compressed into `archive/` (xz, about 1 MB per day) instead of
deleted. It runs as the systemd unit `fugu-mqtt-console-log` on the broker host, in `~/pv`.

Credentials come from the environment or from `fugu-tools/mqtt.env` (`MQTT_USER`, `MQTT_PASS`, `MQTT_PORT`;
see `etc/mqtt.env.example`; never commit the filled-in file). It connects to `localhost`; set
`MQTT_CONSOLE_HOST` to log from another host.

Install or update:

```bash
scp mqtt_console.py archiving_handler.py <host>:pv/
scp fugu-mqtt-console-log.service <host>:/tmp/ && ssh <host> 'sudo mv /tmp/fugu-mqtt-console-log.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now fugu-mqtt-console-log && sudo systemctl restart fugu-mqtt-console-log'
```

Read old logs with `xzcat archive/fugu_console.log.<first-timestamp>.<n>.xz`.
