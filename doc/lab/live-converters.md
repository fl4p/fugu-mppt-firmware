*this document is an LLM generated placeholder*

# Agent access to live converters

Internal lab notes, not published. Extracted verbatim from the pre-scrub `doc/Agentic Programming.md` (commit 695feee); the generic part is `website/docs/development/agentic-programming.md`.

## 12. Keeping the Agent in the Loop with a Live Converter

The vconv path above avoids real hardware entirely, but the firmware is also designed so an
agent can observe, influence, and verify a **physical converter** (fry, flat) without standing
next to it.

### Remote access stack

| Layer | Mechanism | When to use |
|---|---|---|
| Primary | telnet via NAT router (:232–:235) | WiFi up, NAT reachable |
| Fallback 1 | MQTT console (`--mqtt <broker> --mqtt-port 1882 --name <dev>`) | telnet unreachable (NAT wedged) |
| Fallback 2 | BLE NUS via ESPHome proxy (`--ble-proxy 192.168.1.231`) | WiFi down, BLE range |
| Last resort | SSH to havan + `tail pv/fugu_console.log` | read-only observation |

Always confirm the device with `hostname` first — NAT port mappings are not static.

### Read-only observation (always safe)

An agent can continuously tail state without touching the control loop:

```bash
# poll sensor averages every 5 seconds
while true; do
    python etc/fugu_console.py --ip 192.168.1.231:232 -c "sensor avg"; sleep 5
done
```

`rt-stats`, `tasks`, `mem`, `bootinfo`, `status`, and `coredump info` are all read-only and
safe on a live converter. InfluxDB telemetry provides a passive view without any console round-trip.

### Config changes (low risk, instantly reversible)

`set-config` / `get-config` / `conf-check` edit the littlefs partition in place without
rebooting. Changes take effect on the next parameter re-read cycle (charger: ~1 s; most others:
at the next `svc restart` or reboot). An agent can:

1. Record the current value with `get-config <file> <key>`.
2. Apply the change with `set-config`.
3. Observe the effect via telemetry or `sensor avg`.
4. Revert with `set-config <file> <key> <original>` if the effect is wrong.

This loop is fast and safe because the protection stack (OV/OC/UV/loop-latency watchdogs) is
always active and cuts the converter independently of config.

### PWM commands (handle with care)

`dc <duty>`, `+N`, `-N`, `sweep`, and `mppt` directly manipulate the half-bridge. Safe use:

- Only drive these in **manual PWM mode** (`dc <duty>` engages it; `mppt` exits it).
- Keep `+N` steps small (≤ 5) and watch Iin — large positive jumps cause current transients.
- Protection cuts out at the output limits (`limits.conf` `lv_i_max`/`lv_max` in a buck, `hv_i_max`/`hv_max` in a boost; legacy `iout_max`/`vout_max`); the converter stops and backs off.
- `sync off` (diode emulation) is safer than `sync forced` (no reverse-current check).
- `measure-coil l0` / `measure-coil ls` uses a controlled DCM sweep and restores MPPT when
  done — it is the intended on-device calibration path, not raw PWM stepping.

An agent should validate PWM commands on a vconv build first, then apply the same sequence to
the live unit with telemetry open.

### OTA to a live converter

OTA halts the converter and ADC during the flash write (~30 s). For fry and flat:

1. **Validate on vconv first** — same firmware image, different littlefs config.
2. **`ota.py -n -m <name>`** — confirm the target device, current version, and image version.
3. **Watch InfluxDB** for the version field flip and healthy resumption of MPPT within ~60 s.
4. **Check logs** for `ADC error`, `Loop latency high`, or panic markers — an agent should grep
   the device log (via `ssh havan.local tail pv/fugu_console.log`) after every OTA.
5. OTA rollback is active: if `setup()` hangs for >30 s, the boot watchdog restarts into the
   prior slot. An agent that sees the device not come back after 90 s should check the version
   — if it reverted, the new image has a bug.

### Automated regression check after an OTA

```python
from fugu.transport import SocketTransport
from fugu.console import Console
import time, re

c = Console(SocketTransport("192.168.1.231", 232), eol="\r\n")
# wait for the device to come back
assert c.wait_ready(probe="mem", timeout=90), "device did not recover"

# confirm ADC is healthy (climbing N= count, no 'ADC error' in recent lines)
reply = c.command("sensor avg", timeout=3.0)
assert reply.ok and re.search(r"vin=\d", reply.text), "sensor avg failed"

# check for panic marker in device log (host-side)
import subprocess
log = subprocess.check_output(["ssh", "havan.local", "tail -n 100 pv/fugu_console.log"],
                              text=True)
for marker in ("ADC error", "Guru Meditation", "Backtrace:", "assert failed"):
    assert marker not in log, f"found panic marker: {marker}"

print("post-OTA checks passed")
```

---

