// Reuse the actual MQTT/service stubs while exercising the compiled pmini path.
#define main legacy_main
#include "tele-mqtt-test.cpp"
#undef main
bool telePminiConfigured() { return true; }
int main() {
    std::FILE *f=std::fopen(kConf,"w"); assert(f);
    std::fputs("broker_uri=mqtt://127.0.0.1:1883\n",f); std::fclose(f);
    PathPeek::set(MQTT,kConf);setWallClockUs(1000000);
    reset(); assert(!teleMqttLease(30));
    run(900); assert(g_pubs.empty());
    run(200); assert(!g_pubs.empty());
    for(const auto &p:g_pubs) assert(p.second.size()<=1440 && uint8_t(p.second[0])<=2);
    // No producer traffic is needed for the first-point deadline itself.
    const auto before=g_pubs.size();
    teleMqttEnqueue(std::string("\x02\x02\x00",3)); // empty table frame, raw fallback
    advance(1000);teleMqttTick(now());assert(g_pubs.size()>before);
    deliver(MQTT_EVENT_DISCONNECTED);teleMqttTick(now());
    deliver(MQTT_EVENT_CONNECTED);teleMqttTick(now());
    teleMqttLease(0);assert(!packetEncoder);
    assert(!teleMqttLease(30));run(1100);assert(packetEncoder);
    teleMqttLease(0);MQTT.stop();assert(g_live==0);std::remove(kConf);
    puts("tele-mqtt-pmini tests passed");
}
