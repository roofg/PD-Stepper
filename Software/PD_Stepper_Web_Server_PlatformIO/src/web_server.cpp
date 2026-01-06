#include "web_server.h"
#include "dual_core_scaffold.h"
#include "encoder.h"
#include "index_html.h"
#include "tmc_driver.h"
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <WiFi.h>


// External pin definitions from main.cpp
// (These are #defines in main.cpp, so we'll create simple constexpr
// equivalents)
const int PG = 15;
const int VBUS = 4;
const float DIV_RATIO = 0.1189427313;

// External references to globals in main.cpp (must be in global scope)
extern Preferences preferences;
extern const char *ssid;
extern const char *password;
extern String enabled1;
extern String setVoltage;
extern String microsteps;
extern String current;
extern String stallThreshold;
extern String standstillMode;
extern volatile bool speedUpdatePending;
extern volatile int pendingSpeed;
extern volatile bool posUpdatePending;
extern volatile int pendingPosMode;
extern bool PGState;
extern float VBusVoltage;

// Forward declarations for global functions from main.cpp
void writeSettings();

namespace webserver {

// Local web server instance
static AsyncWebServer server(80);
static int webServerCore = 1;

String processor(const String &var) {
  if (var == "enabled1") {
    if (enabled1 == "enabled") {
      return "checked";
    } else {
      return "";
    }
  }

  if (var == "microsteps") {
    return String(microsteps);
  }

  if (var == "voltage") {
    return String(setVoltage);
  }

  if (var == "current") {
    return String(current);
  }

  if (var == "stall_threshold") {
    return String(stallThreshold);
  }

  if (var == "standstill_mode") {
    return String(standstillMode);
  }
  return String("");
}

String readPGState() {
  PGState = digitalRead(PG);
  if (PGState == 0) {
    return ("Power Good");
  } else {
    return ("Power Bad");
  }
}

String readVoltage() {
  uint32_t mvSum = 0;
  int samples = 10;

  for (int i = 0; i < samples; i++) {
    mvSum += analogReadMilliVolts(VBUS);
  }

  float avgMilliVolts = (float)mvSum / (float)samples;
  VBusVoltage = (avgMilliVolts / 1000.0) / DIV_RATIO;
  return String(VBusVoltage, 2) + "V";
}

String readEncoderPos() {
  encoder::read();
  return String(encoder::getTotalCounts());
}

String readTMCStatus() {
  if (tmc::hardwareDisabled()) {
    return ("Hardware Disabled");
  }
  return tmc::getStatusString();
}

String readStallStatus() { return String(tmc::getStallGuardResult()); }

void initWebServer(int core) {
  webServerCore = core;
  USBSerial.printf("Initializing web server on core %d\\n", core);

  // Set up Wifi and ESP32 network configuration
  IPAddress local_ip(192, 168, 4, 4); // esp32 will be 192.168.4.4
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  WiFi.softAP(ssid, password);
  IPAddress ip = WiFi.softAPIP();
  USBSerial.print("AP IP address: ");
  USBSerial.println(ip);

  // Serve HTML page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html, processor);
  });

  server.on("/powergood", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readPGState().c_str());
  });

  server.on("/voltage", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readVoltage().c_str());
  });

  server.on("/position", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readEncoderPos().c_str());
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readTMCStatus().c_str());
  });

  server.on("/stallguard", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", readStallStatus().c_str());
  });

  // Endpoint to trigger network-task command (Net Cmd)
  server.on("/netcmd", HTTP_POST, [](AsyncWebServerRequest *request) {
    USBSerial.printf("Webserver netcmd running on core %d\\n",
                     xPortGetCoreID());
    USBSerial.println("Net Cmd received");
    request->send(200, "text/plain", "ok");

    // Parse command
    if (request->hasParam("Netcmd", true)) {
      const AsyncWebParameter *p = request->getParam("Netcmd", true);
      int cmdId = 1; // Netcmd
      int cmdValue = p->value().toInt();
      USBSerial.println("Dispatching network command value: " +
                        String(cmdValue));
      bool result = dispatch_network_cmd(cmdId, cmdValue);
      USBSerial.println("Dispatch result: " + String(result));
    }
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("slider", true)) {
      const AsyncWebParameter *p = request->getParam("slider", true);
      pendingSpeed = p->value().toInt();
      speedUpdatePending = true;
    }
    if (request->hasParam("positionControl", true)) {
      const AsyncWebParameter *p = request->getParam("positionControl", true);
      pendingPosMode = p->value().toInt();
      posUpdatePending = true;
    }
    request->send(200);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    String inputMessage;

    if (request->hasParam("enabled1", true)) {
      enabled1 = "enabled";
    } else {
      enabled1 = "disabled";
    }
    if (request->hasParam("setvoltage", true)) {
      inputMessage = request->getParam("setvoltage", true)->value();
      setVoltage = inputMessage;
      tmc::moveAtVelocity(0);
    }
    if (request->hasParam("microsteps", true)) {
      inputMessage = request->getParam("microsteps", true)->value();
      microsteps = inputMessage;
      tmc::moveAtVelocity(0);
    }
    if (request->hasParam("current", true)) {
      inputMessage = request->getParam("current", true)->value();
      current = inputMessage;
    }
    if (request->hasParam("stall_threshold", true)) {
      inputMessage = request->getParam("stall_threshold", true)->value();
      stallThreshold = inputMessage;
    }
    if (request->hasParam("standstill_mode", true)) {
      inputMessage = request->getParam("standstill_mode", true)->value();
      standstillMode = inputMessage;
      ::writeSettings(); // Call from global namespace
    }
    request->redirect("/");
  });
}

void beginWebServer() {
  server.begin();
  USBSerial.printf("Web server started on core %d\\n", xPortGetCoreID());
}

} // namespace webserver
