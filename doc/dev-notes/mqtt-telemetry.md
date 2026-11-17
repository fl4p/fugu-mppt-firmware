*this document is an LLM generated placeholder*

# MQTT telemetry sink (`tele-mqtt`)

Build option `CONFIG_FUGU_WITH_TELE_MQTT` (default off, selects `CONFIG_FUGU_WITH_TELE_BINARY`).
Source: `src/tele/tele_mqtt.{h,cpp}`, host test `test/host-stub/tele-mqtt-test.cpp`, e2e
`etc/e2e-test/test_mqtt_tele.py`.

## Topic and payload

- Topic `pv/tele/<hostname>`, QoS 0, not retained. The topic string is built when a lease starts.
- Payload `<cid><payload>` with `cid = 0x00` (uncompressed), byte-identical to one UDP telemetry
  datagram of the binary wire: a sequence of `<varint len><frame>` sym_line_protocol frames
  (frame type 2 = symbol table, 1 = data point).
- Points are batched for up to 500 ms or until the batch reaches 1024 - 128 bytes.
- Decoder: `etc/fugu/teledec.py::decode_payload(msg[0], msg[1:], tab)`; `tab` persists across
  messages.

## Lease

- `tele-mqtt <s>` starts a lease of `min(s, 3600)` seconds; a repeated command sets the end to
  now + s (it renews, it does not add). `tele-mqtt 0` stops, `tele-mqtt` prints the state.
- The stream stops when the lease expires or when the `mqtt` service is not Running
  (`svc off mqtt`, a failed restart). A host that wants a continuous stream re-leases before expiry.
- While active, `g_teleBinary` is forced on (`teleLoadWireConf()` keeps it forced); on stop
  `teleLoadWireConf()` restores `tele.conf::binary`.
- The tick (`teleMqttTick`, network loop, after `g_services.tickAll()`) calls `mppt.telemetry()`
  so points are produced without the UDP `tele` service; the 20 ms point rate limit dedups
  with the UDP path.

## Preconditions (checked in order, start only)

1. clock set (`timeSynced`, SNTP or `set-time`)
2. MQTT connected
3. not the UDP text wire (`tele` running with `binary=0` caches a text compressor)
4. at least 16 KB free heap (`MALLOC_CAP_DEFAULT`)
5. the 1 KB batch buffer can be reserved

## Heap policy

One `std::string` reserved to 1024 + 64 + 1 bytes on start (byte 0 is the cid, kept so a publish
needs no second buffer) and freed on stop. A frame that does not fit is dropped (drop-newest) and
counted in `dropped`, as is a batch the client refuses to publish. The esp-mqtt side of a QoS 0
publish (buffering, blocking on the client lock or socket) is not measured on hardware yet.

## Disconnect behaviour

While the broker is disconnected the lease keeps running but the batch is discarded each tick.
On reconnect the batch is cleared and `g_symtab.forceResend()` makes the next messages carry the
full symbol table, so a subscriber can decode from its first message after the gap.

## Fleet alternative

A host-side relay that republishes the UDP wire on the same topic covers devices built without
the option; the payload format is the same.
