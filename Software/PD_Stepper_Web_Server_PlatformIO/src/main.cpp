#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#include <Wire.h>

#include "index_html.h"
#include "tmc_driver.h"

Preferences preferences;

// forward declarations (needed because we're compiling as C++ source)
void readEncoder();
void configureSettings();
void readSettings();
void writeSettings();


//access point SSID and password (password = "" for no password)
const char *ssid = "PD Stepper";
const char *password = "";

AsyncWebServer server(80);

//TMC2209 pins/config (kept here; driver uses these constants)
#define TMC_EN  21
#define STEP    5
#define DIR     6
#define MS1     1
#define MS2     2
#define SPREAD  7
#define TMC_TX  17
#define TMC_RX  18
#define DIAG    16
#define INDEX   11

//PD Trigger (CH224K)
#define PG      15  //power good singnal (dont enable stepper untill this is good)
#define CFG1    38
#define CFG2    48
#define CFG3    47

//Other
#define VBUS    4
#define NTC     7
#define LED1    10
#define LED2    12
#define SW1     35
#define SW2     36
#define SW3     37
#define AUX1    14
#define AUX2    13

//Global variables
int set_speed = 0;
bool PGState = 0; //state of the power good signal from PD sink IC
bool enabledState = 0;
bool state = 0; //step state

//AS5600 Hall Effect Encoder
#define AS5600_ADDRESS 0x36 // I2C address of the AS5600 sensor
signed long total_encoder_counts = 0;
unsigned long lastEncRead = 0;

int mainFreq = 10; //Scheduled frequency = 100hz (for slower tasks, encoder reading etc)

//button read and debounce
bool incButtonState = HIGH;
bool decButtonState = HIGH;
bool resetButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int buttonSpeed = 0;

//Voltage reading and calc
float VBusVoltage = 0;
float VREF = 3.3;
const float DIV_RATIO = 0.1189427313;  //20k&2.7K Voltage Divider

//Values received from websever save command
String enabled1 = "enabled";
String setVoltage = "12";
String microsteps = "32";
String current = "30";
String stallThreshold = "10";
String standstillMode = "NORMAL";

//variable updated in callback
volatile bool speedUpdatePending = false;
volatile int pendingSpeed = 0;
volatile bool posUpdatePending = false;
volatile int pendingPosMode = 0;

//Varaiables for position control (open loop)
signed long setPoint = 0;
signed long CurrentPosition = 0;
unsigned long lastStep = 0;

//read state of PG pin to display on webpage
String readPGState(){
  PGState = digitalRead(PG);
  if (PGState == 0){
    return ("Power Good");
  } else {
    return ("Power Bad");
  }
}

// read VBUS voltage to display on webpage
String readVoltage() {
  uint32_t mvSum = 0;
  int samples = 10; // Average 10 readings to smooth out electrical noise

  for (int i = 0; i < samples; i++) {
    mvSum += analogReadMilliVolts(VBUS);
  }

  float avgMilliVolts = (float)mvSum / (float)samples;
  VBusVoltage = (avgMilliVolts / 1000.0) / DIV_RATIO;
  return String(VBusVoltage, 2) + "V";
}

//read encoder pos to display on webpage
String readEncoderPos(){
  readEncoder();
  return String(total_encoder_counts);
}

String readTMCStatus(){
  if (tmc_hardwareDisabled()){
   return ("Hardware Disabled");
  }
  return tmc_getStatusString();
}

String readStallStatus(){
  return String(tmc_getStallGuardResult());
}


//updates placeholder varibles in the HTML code
String processor(const String& var)
{
  if(var == "enabled1"){
    if (enabled1 == "enabled"){
      return "checked";
    } else { return "";}
  }

  if (var == "microsteps"){
    return String(microsteps);
  }

  if(var == "voltage"){
     return String(setVoltage);
  }

  if(var == "current"){
     return String(current);
  }

  if(var == "stall_threshold"){
     return String(stallThreshold);
  }

  if(var == "standstill_mode"){
     return String(standstillMode);
  }
  return String("");
}


void setup() {
  //PD Trigger Setup
  pinMode(PG, INPUT);
  pinMode(CFG1, OUTPUT);
  pinMode(CFG2, OUTPUT);
  pinMode(CFG3, OUTPUT);
  digitalWrite(CFG1, LOW);
  digitalWrite(CFG2, LOW);
  digitalWrite(CFG3, HIGH);

  //General
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);

  //TMC pins
  pinMode(MS1, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(TMC_EN, OUTPUT);
  pinMode(DIAG, INPUT);
  digitalWrite(TMC_EN, LOW);
  digitalWrite(MS2, LOW);

  //AS5600 Hall Encoder Setup
  Wire.begin();

  //ADC Setup
  analogSetPinAttenuation(VBUS, ADC_11db);

  readSettings(); //get saved values from EEPROM

  tmc_init(TMC_RX, TMC_TX);
  tmc_setRunCurrent(20);
  tmc_enableAutomaticCurrentScaling();
  tmc_enableStealthChop();
  tmc_setCoolStepDurationThreshold(5000);
  tmc_disable();

  configureSettings(); //use saved settings

  delay(200);
  Serial.begin(115200);
  Serial.println("Code Starting");
  WiFi.softAP(ssid, password);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(ip);

  // Serve HTML page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html, processor);
  });
  server.on("/powergood", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readPGState().c_str());
  });
  server.on("/voltage", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readVoltage().c_str());
  });
  server.on("/position", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readEncoderPos().c_str());
  });
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readTMCStatus().c_str());
  });
  server.on("/stallguard", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readStallStatus().c_str());
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("slider", true)) {
      const AsyncWebParameter* p = request->getParam("slider", true);
      pendingSpeed = p->value().toInt();
      speedUpdatePending = true;
    }
    if (request->hasParam("positionControl", true)) {
      const AsyncWebParameter* p = request->getParam("positionControl", true);
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
      tmc_moveAtVelocity(0);
    }
    if (request->hasParam("microsteps", true)) {
      inputMessage = request->getParam("microsteps", true)->value();
      microsteps = inputMessage;
      tmc_moveAtVelocity(0);
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
      writeSettings();
    }
    request->redirect("/");
  });

  server.begin();

  digitalWrite(LED1, HIGH);
  delay(200);
  digitalWrite(LED1, LOW);

}

void loop() {
  if (speedUpdatePending) {
    set_speed = pendingSpeed;
    tmc_moveAtVelocity(set_speed * (microsteps.toInt()));
    speedUpdatePending = false;
  }

  if (posUpdatePending) {
    tmc_moveAtVelocity(0);
    if (pendingPosMode == 1)      setPoint -= 25600;
    else if (pendingPosMode == 2) setPoint -= 12800;
    else if (pendingPosMode == 3) setPoint += 12800;
    else if (pendingPosMode == 4) setPoint += 25600;
    posUpdatePending = false;
  }

  if (millis() - lastEncRead >= mainFreq){
    lastEncRead = millis();
    digitalWrite(LED2, digitalRead(DIAG));
    PGState = digitalRead(PG);
    if (PGState == LOW and enabled1 == "enabled" and enabledState == 0){
      tmc_enable();
      enabledState = 1;
    } else if ((PGState == HIGH or enabled1 == "disabled") and enabledState == 1){
      tmc_disable();
      enabledState = 0;
    }
  }

  int delaySpeed = 4500;
  int microSteps = microsteps.toInt();
  int delaySpeedAdjusted = delaySpeed/microSteps;
  if (setPoint > CurrentPosition){
    if (micros()-lastStep > delaySpeedAdjusted){
      digitalWrite(DIR, LOW);
      digitalWrite(STEP, state);
      state = !state;
      CurrentPosition = CurrentPosition + (256/microSteps);
      lastStep = micros();
    }
  } else if (setPoint < CurrentPosition){
    if (micros()-lastStep > delaySpeedAdjusted){
      digitalWrite(DIR, HIGH);
      digitalWrite(STEP, state);
      state = !state;
      CurrentPosition = CurrentPosition - (256/microSteps);
      lastStep = micros();
    }
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    bool currentIncButtonState = digitalRead(SW3);
    bool currentDecButtonState = digitalRead(SW1);
    bool currentResetButtonState = digitalRead(SW2);
  
    if (currentIncButtonState != incButtonState) {
      incButtonState = currentIncButtonState;
      if (incButtonState == LOW) {
        buttonSpeed = buttonSpeed + 30;
        if (buttonSpeed > 330){
          buttonSpeed = 330;
        }
        tmc_moveAtVelocity(buttonSpeed*(microsteps.toInt()));
      }
    }
  
    if (currentDecButtonState != decButtonState) {
      decButtonState = currentDecButtonState;
      if (decButtonState == LOW) {
        buttonSpeed = buttonSpeed -30;
        if (buttonSpeed < -330){
          buttonSpeed = -330;
        }
        tmc_moveAtVelocity(buttonSpeed*(microsteps.toInt()));
      }
    }
  
    if (currentResetButtonState != resetButtonState) {
      resetButtonState = currentResetButtonState;
      if (resetButtonState == LOW) {
        buttonSpeed = 0;
        tmc_moveAtVelocity(0);
      }
    }
  }

}

void readEncoder(){
  int raw_counts;
  static int prev_raw_counts = 0;
  static signed long revolutions = 0;
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDRESS, 2);
  if (Wire.available() >= 2) {
    raw_counts = Wire.read() << 8 | Wire.read();
  }
  if (prev_raw_counts > 3000 && raw_counts < 1000) {
    revolutions++;
  } else if (prev_raw_counts < 1000 && raw_counts > 3000) {
    revolutions--;
  }
  prev_raw_counts = raw_counts;
  total_encoder_counts = raw_counts + (4096 * revolutions);
}

void configureSettings(){
  if (setVoltage == "5"){
      digitalWrite(CFG1, HIGH);
  } else if (setVoltage == "9"){
      digitalWrite(CFG1, LOW);
      digitalWrite(CFG2, LOW);
      digitalWrite(CFG3, LOW);
  } else if (setVoltage == "12"){
      digitalWrite(CFG1, LOW);
      digitalWrite(CFG2, LOW);
      digitalWrite(CFG3, HIGH);
  } else if (setVoltage == "15"){
      digitalWrite(CFG1, LOW);
      digitalWrite(CFG2, HIGH);
      digitalWrite(CFG3, HIGH);
  } else if (setVoltage == "20"){
      digitalWrite(CFG1, LOW);
      digitalWrite(CFG2, HIGH);
      digitalWrite(CFG3, LOW);
  }

  tmc_setRunCurrent(current.toInt());
  tmc_setMicrostepsPerStep(microsteps.toInt());
  tmc_setStallGuardThreshold(stallThreshold.toInt());

  if (standstillMode == "NORMAL"){ tmc_setStandstillMode(0);} // map modes in driver
  else if (standstillMode == "FREEWHEELING"){ tmc_setStandstillMode(1);} 
  else if (standstillMode == "BRAKING"){ tmc_setStandstillMode(2);} 
  else if (standstillMode == "STRONG_BRAKING"){ tmc_setStandstillMode(3);} 
}

void readSettings(){ 
  preferences.begin("settings", false);
  enabled1 = preferences.getString("enable", ""); 
  if (enabled1 == ""){ 
    preferences.end(); 
    enabled1 = "enabled";
    setVoltage = "12";
    microsteps = "32";
    current = "30";
    stallThreshold = "10";
    standstillMode = "NORMAL";
    writeSettings();
  } else {
    Serial.println("Settings found in EEPROM");
    setVoltage = preferences.getString("voltage", ""); 
    microsteps = preferences.getString("microsteps", ""); 
    current = preferences.getString("current", ""); 
    stallThreshold = preferences.getString("stallThreshold", ""); 
    standstillMode = preferences.getString("standstillMode", ""); 
    preferences.end();
  }
}

void writeSettings(){ 
  preferences.begin("settings", false);
  preferences.putString("enable", enabled1); 
  preferences.putString("voltage", setVoltage); 
  preferences.putString("microsteps", microsteps); 
  preferences.putString("current", current); 
  preferences.putString("stallThreshold", stallThreshold); 
  preferences.putString("standstillMode", standstillMode); 
  Serial.println("Saving settings to flash");
  preferences.end();
  configureSettings();
}
