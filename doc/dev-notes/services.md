*this document is an LLM generated placeholder*

So ownership and logic are split across three files, joined by a global. That indirection is exactly the kind of thing the service
refactor is trying to eliminate (ftp_service.h:4 even notes "no dependency on the main.cpp globals" — but it does depend on one
indirectly).


  

# Here is Claude's plan:
╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌
Plan: A small service architecture (systemd-lite)

Context

Today the firmware's optional non-RT subsystems (MQTT, FTP, telnet, telemetry/InfluxDB,
LCD, scope) are constructed ad-hoc in setup() and ticked from scattered direct calls in
loopNetwork_task (src/main.cpp:813-861). There is no uniform way to start/stop/restart one
at runtime, no status reporting, and no per-module log control — esp_log_level_set() is
never called even though CONFIG_LOG_DYNAMIC_LEVEL_CONTROL=y makes runtime per-tag levels
possible.

This change introduces a small service abstraction: each service wraps one building block
and exposes start() / stop() / restart(), reports a status (Running / Stopped / Failed),
has its own runtime-adjustable log level, and an optional periodic tick(). State persists:
each service owns a config file on littlefs (enabled + log_level keys), read at boot and
written on runtime changes. There is already an empty class Service {} at src/tele/mqtt.h:9
(only MqttService derives from it) — this is promoted into the real base class.

Scope: framework + wrap the six non-RT subsystems above. RT-critical core (ADC / MPPT /
converter on core 1) is out of scope and stays non-stoppable. WiFi stays a precondition,
not a service (it's gated by RT state in the net loop and already has wifi on/off commands).

Design decisions (confirmed with user)

- Per-service own config file under /littlefs/conf/, with enabled (0/1) and
  log_level (error/warn/info) keys. State persists across reboot.
- Optional tick() hook driven by the network loop (core 0), centralizing today's scattered
  per-iteration calls.
- Log levels: ERROR / WARN / INFO only (don't bump CONFIG_LOG_MAXIMUM_LEVEL; DEBUG/VERBOSE
  are compiled out). A service's name() is its ESP_LOG tag; setLogLevel() →
  esp_log_level_set(name(), level).

Files

- new src/service.h — Service base, ServiceManager, inline ServiceManager g_services, free helpers (levelToStr/strToLevel/stateStr).
  Header-only (no main/CMakeLists.txt edit).
- src/tele/mqtt.h / mqtt.cpp — replace empty Service include; make MqttService derive from new base + implement hooks.
- src/main.cpp — define service wrapper globals near :46-51; register + start in setup(); rewrite tick dispatch in loopNetwork_task
  (:813-861); add service … commands to handleCommand (:882-1082).
- src/tele/telemetry.cpp / .h — split ftpUpdate() (:44-49) into ftpHandle()/telnetLoop()/scopeUpdate(); add ftpEnd()/telnetStop()
  wrappers; trim duplicate scope/ftp/telnet bring-up in _wifiConnected() (:241-256).
- src/viz/lcd.h, src/tele/scope.h — start/stop/tick targets (no rewrites).
- Config images: add ftp.conf, telnet.conf, lcd.conf, scope.conf to config/lab/wokwi_mock/conf/ and config/fmetal/conf/;
  mqtt.conf/tele.conf already exist.

Step 1 — Service base + ServiceManager (src/service.h)

enum class ServiceState { Stopped, Running, Failed };

class Service {
public:
Service(const char *name, const char *confPath); // confPath e.g. "/littlefs/conf/mqtt.conf"
virtual ~Service() = default;

     const char *name() const;            // also the ESP_LOG tag
     ServiceState state() const;          // calls liveState() for live-condition services
     esp_log_level_t logLevel() const;
     bool enabled() const;                // persisted desired state

     bool start();   // no-op if Running; sets Failed on throw / onStart()==false
     void stop();    // no-op if Stopped; clears Failed -> Stopped
     bool restart(); // default: stop(); return start();
     void tick();    // if Running: try{ onTick(); } catch{ fail(); }

     void setLogLevel(esp_log_level_t, bool persist = true); // esp_log_level_set + write conf
     void setEnabledPersist(bool);                            // write enabled=0/1 to conf
     void loadConf();                                         // read enabled + log_level at boot

protected:
virtual bool onStart() = 0;
virtual void onStop()  = 0;
virtual void onTick()  {}
virtual ServiceState liveState() const { return _state; }
void fail(const char *why);
// _name, _confPath, _state=Stopped, _logLevel=ESP_LOG_INFO, _enabled=true
};

class ServiceManager {
std::vector<Service*> _services;
public:
void registerService(Service *s);   // push_back + s->loadConf()
Service *findByName(const char *n) const;
const std::vector<Service*> &all() const;
void startEnabledAtBoot();           // for each enabled(): start()
int startEnabledNetworkServices();   // self-heal/retry: start enabled non-Running net services; returns count tried
void tickAll();                      // for each Running: tick()
};
inline ServiceManager g_services;
- start() calls esp_log_level_set(_name,_logLevel) first, then onStart(); exceptions caught at the boundary → fail(). Never let an
  exception reach the RT path or net loop (-fexceptions is on).
- loadConf() opens ConfFile{_confPath, /*no_warn*/true}; getByte("enabled", default), getString("log_level","info") → strToLevel
  (clamped to ERROR/WARN/INFO).
- Persistence writes use the existing ConfFile.add({{k,v}}, /*overwrite*/true) idiom (see src/main.cpp:1034). No <sstream>; use
  snprintf/UART_LOG/std::string.

Step 2 — Concrete wrappers

Thin wrappers calling existing code; placed in src/tele/services.h (+ bodies in already-compiled telemetry.cpp), except MQTT which is
MqttService.

Service: mqtt
conf: mqtt.conf
onStart: MQTT.init(conf) + onConnected=haMqttSendDiscovery + mppt.charger.beginMqtt(conf)
onStop: MQTT.close()
onTick: throttled haMqttUpdate(...) (moves main.cpp:837-843)
status note: liveState: Running once started; svc list appends (conn) when isConnected()
────────────────────────────────────────
Service: telemetry
conf: tele.conf
onStart: require WiFi.isConnected()
onStop: (tick skipped)
onTick: mppt.telemetry(); telemetryFlushPointsQ(...) (main.cpp:830-831)
status note: governs core-0 flush only
────────────────────────────────────────
Service: ftp
conf: ftp.conf
onStart: require WiFi; ftpBegin()
onStop: ftpEnd() (new)
onTick: ftpHandle() (new)
status note:
────────────────────────────────────────
Service: telnet
conf: telnet.conf
onStart: require WiFi; setupTelnet()
onStop: telnetEnd() + set_logging_telnet(nullptr)
onTick: telnetLoop() (new)
status note: log-sink teardown is mandatory
────────────────────────────────────────
Service: lcd
conf: lcd.conf
onStart: lcd.init() if !lcd
onStop: flag-skip
onTick: lcd.updateValues(LcdValues{...}) (main.cpp:846-853)
status note: default enabled=0 when no I2C (avoid permanent Failed)
────────────────────────────────────────
Service: scope
conf: scope.conf
onStart: require WiFi; scope begin (channels still added at main.cpp:412)
onStop: scope->end()
onTick: scopeUpdate(); if(scope->connected) scope->netLoop()
status note: created lazily here; remove dup creation in _wifiConnected()

WiFi: precondition. Network services onStart() return false (→ Failed) if WiFi down; the net
loop self-heals them via startEnabledNetworkServices() on the WiFi-up edge.

Step 3 — loopNetwork_task rewrite (src/main.cpp:813-861)

Keep core assert, loopUart, flush_async_uart_log, process_queued_tasks, and the wifi gate
(:823-832, including the RT-state condition at :827). Then:
- on WiFi-up edge: g_services.startEnabledNetworkServices();
- g_services.tickAll();
- keep the loopLF + LED low-frequency block (:835-844) and lcd is now a service tick.
- Preserve the yield: keep vTaskDelay(pdMS_TO_TICKS(1)) unless the scope service is Running
  and scope->connected (its netLoop() blocks). Guard via findByName("scope").

Step 4 — setup() registration (src/main.cpp)

After mountLFS() (:297), WiFi connect attempt (:384-399), and scope channel setup (:412):
- Define wrapper globals at file scope near :46-51 (must outlive setup(); not self-registering in ctors — littlefs isn't mounted at
  static-init).
- registerService(...) each (mqtt, telemetry, ftp, telnet, lcd, scope) → loadConf() runs.
- g_services.startEnabledAtBoot();
- Remove old direct bring-up: MQTT block at :393-398, and scope/ftp/telnet creation in
  _wifiConnected() (telemetry.cpp:241-256); keep MDNS there. lcd.init() early splash at
  :366 may stay (LcdService.onStart no-ops if already inited).

Step 5 — service console commands (handleCommand, before final else at :1073)

Match existing startsWith/substring style; reachable from UART/USB/telnet/MQTT automatically:
- svc / svc list → table: NAME, STATE (stateStr), LOG (levelToStr), (conn) for mqtt.
- svc start <name> → setEnabledPersist(true) + start().
- svc stop <name> → setEnabledPersist(false) + stop().
- svc restart <name> → restart().
- svc log <name> <error|warn|info> → setLogLevel(strToLevel(...), true).
  Unknown name → return false.

Step 6 — Config convention & provisioning

- Each service conf gains enabled = 1 and log_level = info (lcd default enabled = 0).
  Missing file/key → defaults (silent via no_warn). Existing keys (e.g. broker_uri) untouched.
- Add ftp.conf/telnet.conf/lcd.conf/scope.conf to config/lab/wokwi_mock/conf/ (flashed
  with firmware) and config/fmetal/conf/. provision.py needs no code change.
- Update CLAUDE.md's boot-read conf list (documentation only) to mention the new service confs.

Verification

1. Build: . ./idf-export.sh && idf.py build. Watch for -Werror=missing-field-initializers
   (the LcdValues{...} aggregate keeps all 5 fields — unchanged) and any <sstream> link error
   (design introduces none). No new TU required (header-only + edits to existing files).
2. On-target (idf.py -p $ESPPORT flash monitor), in the console:
- svc list → table renders; mqtt shows (conn) once connected.
- svc stop mqtt → publishes stop; mqtt.conf gains enabled=0. svc start mqtt →
  reconnects and charger BMS topics re-subscribe.
- svc log telnet warn → telnet.conf log_level=warn; INFO logs from the telnet tag
  stop. get-config telnet.conf log_level → warn.
- restart → telnet stays warn, mqtt stays stopped (persistence confirmed).
- Stop telnet from a telnet session → client drops cleanly, device stays alive (log-sink
  teardown via set_logging_telnet(nullptr)).

Risks (carry into implementation)

- Telnet log-sink: onStop() must set_logging_telnet(nullptr) or logging writes to a dead
  ESPTelnet.
- Scope: remove duplicate creation in _wifiConnected(); netLoop() blocks → keep the
  vTaskDelay(1) yield fallback.
- MQTT/charger coupling: beginMqtt() inside onStart() so restart re-subscribes; charger
  outlives MQTT (safe).
- WiFi precondition: without the self-heal edge, network services stay Failed until a manual
  svc start.
- Static-init order: wrapper objects are file-scope globals, registered in setup() after
  mountLFS().