#include "logging.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#ifdef ESP32
#include <FS.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <WiFi.h>
//#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESP8266mDNS.h>
#endif
//#include <ESPAsyncWebServer.h>

//#include <SPIFFSEditor.h>

//AsyncWebServer server(80);
//AsyncWebSocket ws("/ws");
//AsyncEventSource events("/events");

void webserver_begin(void) {

    MDNS.addService("http","tcp",80);
    // SPIFFS.begin(); // TODO use littleFS

    //server.serveStatic("/", SPIFFS, "/web").setDefaultFile("index.html");
    //server.begin();
}
