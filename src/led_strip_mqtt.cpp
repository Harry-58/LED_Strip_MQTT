
#define  mitTVSIM

#include <Streaming.h>
#include <myMacros.h>
#include <myMqtt.h>
#include <myUtils.h>

#define DEBUG_EIN  //"Schalter" zum aktivieren von DEBUG-Ausgaben
#include <myDebug.h>


#include <IotWebConf.h>       // https://github.com/prampec/IotWebConf

// UpdateServer includes
#ifdef ESP8266
# include <ESP8266HTTPUpdateServer.h>
#elif defined(ESP32)
// For ESP32 IotWebConf provides a drop-in replacement for UpdateServer.
# include <IotWebConfESP32HTTPUpdateServer.h>
#endif

//#define IR_SEND_PIN 33  //muss vor #include <IRremote.h> definiert werden damit default in private/IRremoteBoardDefs.h überschrieben wird
      // BUG: geht nicht -->  in platformio.ini definieren   build_flags =  -D IR_SEND_PIN=33
#define IR_RECV_PIN  34     //34  15
#include <IRremote.h>

// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
const char thingName[] = HOSTNAME;

// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = WLAN_AP_PASS;

#define STRING_LEN 64
#define NUMBER_LEN 16

// -- Configuration specific key. The value should be modified if config structure was changed.
// maximale Läenge 4 Zeichen
#define CONFIG_VERSION "led1"

// -- When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial
//      password to buld an AP. (E.g. in case of lost password)
#define CONFIG_PIN 27

// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
#define STATUS_PIN  LED_BUILTIN

IRsend irsend;
IRrecv irrecv(IR_RECV_PIN);
// http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html
//decode_results results;

// todo: auf std::map oder std::array umstellen  oder enum class
// https://www.heise.de/developer/artikel/Mehr-besondere-Freunde-mit-std-map-und-std-unordered-map-4435976.html
// https://www.heise.de/developer/artikel/C-Core-Guidelines-Regeln-fuer-Aufzaehlungen-3901624.html
// http://www.willemer.de/informatik/cpp/stl.htm
// C:\Users\Harry\.platformio\lib\Basecamp_ID2077/Webserver.cpp   140
String strMqttTopics[] = {"cmd/power", "cmd/rgb",      "cmd/val", "cmd/val_up",   "cmd/val_down", "cmd/save", "cmd/reload",
                          "cmd/reset", "cmd/farbtemp", "cmd/hsb", "cmd/gradient", "cmd/status",   "cmd/tvsim",
                          "IR-Send/Samsung","IR-Send/NEC"};

enum  mqttTopics { POWER, RGB, VAL, VAL_UP, VAL_DOWN, SAVE, RELOAD, RESET, FARBTEMP, HSB, GRADIENT, GET_STATUS_LED, TVSIM,
                   IR_SAMSUNG, IR_NEC };

String strMqttTopicsPub[] = {"stat/rgb", "stat/led" };
enum  mqttTopicsPub {STATUS_RGB, STATUS_LED};

// LED
#include <NeoPixelBrightnessBus.h>
#include <Kelvin2RGB.h>

#include <Preferences.h>  // this library is used to get access to Non-volatile storage (NVS) of ESP32
// see https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/examples/StartCounter/StartCounter.ino

#ifdef mitTVSIM
#include <tv_sim_data.h>
#endif

#if 0  // zum Testen
#define RGB_PIN 13
#define RGB_PIXELS 8

#define TV_SIM_LEDS 8
#define TV_SIM_START_LED 0

#else

#define RGB_PIN 13
#define RGB_PIXELS 120

#define TV_SIM_LEDS 40
#define TV_SIM_START_LED 21

#endif

const uint16_t  numPixels = (sizeof(colors) / sizeof(colors[0]));
uint32_t pixelNum;
uint16_t pr = 0, pg = 0, pb = 0; // Prev R, G, B
uint8_t gradientDelay = 5;
uint8_t gradientHueDelta = 5;
bool TvSim = false;
bool Gradient=false;

// NeoPixelBus< NeoGrbFeature, Neo800KbpsMethod> rgbstrip(RGB_PIXELS, RGB_PIN);
NeoPixelBrightnessBus<NeoGrbFeature, NeoWs2813Method> rgbstrip(RGB_PIXELS, RGB_PIN);
// NeoGamma<NeoGammaTableMethod> colorGamma;
NeoGamma<NeoGammaEquationMethod> colorGamma;

Kelvin2RGB KRGB;

Preferences preferences;  // we must generate this object of the preference library

int led_R, led_G, led_B;
int led_H;
bool ledStatus = true;

struct colorType{
	int r;
	int g;
	int b;
};

// -- Method declarations.
void handleRoot();
void doRestart();
void mqttMessageReceived(String &topic, String &payload);
void mqttCallback(char *topic, byte *payload, unsigned int length);
bool connectMqtt();
bool connectMqttOptions();
// -- Callback methods.
void wifiConnected();
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper *webRequestWrapper);
void setupMqttTopics();
void handleMqttMessage(char *topic, String sPayLoad, uint16_t len);
// -- LED
void setRGB(uint8_t ledR, uint8_t ledG, uint8_t ledB, uint8_t ledH, bool ledStatus);
void setHSB(uint16_t hue, uint16_t sat, uint16_t val, bool ledStatus);
int getCsvIntAtIndex(String csv, uint8_t index);
void putPreferencesLED();
void getPreferencesLED();
void sendStatRgb();
void TvSimulator();
//colorType kelvin2RGB(int kelvin);
void showGradient();
void LedBeleuchtung(uint32_t irCode);

DNSServer dnsServer;
WebServer server(80);
WiFiClient net;
myMqtt mqttClient(net);

#ifdef ESP8266
ESP8266HTTPUpdateServer httpUpdater;
#elif defined(ESP32)
HTTPUpdateServer httpUpdater(false);   // true = mit Debugausgaben
#endif

char mqttServerValue[STRING_LEN];
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];
char mqttMainTopicValue[STRING_LEN];

char mqttWillMessage[] = "Offline";

String topicIR = "IR-Recv";

IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);
// -- Es kann auch das Alias Format verwendet werden. Bsp.: IotWebConfParameterGroup(...)  (dann ist  #include <IotWebConfUsing.h> notwendig)
iotwebconf::ParameterGroup mqttGroup                = iotwebconf::ParameterGroup("mqttConf", "MQTT");
iotwebconf::TextParameter mqttServerParam           = iotwebconf::TextParameter("Server", "mqttServer", mqttServerValue, STRING_LEN, MQTT_HOST);
iotwebconf::TextParameter mqttUserNameParam         = iotwebconf::TextParameter("User", "mqttUser", mqttUserNameValue, STRING_LEN, MQTT_USER);
iotwebconf::PasswordParameter mqttUserPasswordParam = iotwebconf::PasswordParameter("Password", "mqttPass", mqttUserPasswordValue, STRING_LEN, MQTT_PASS);
iotwebconf::TextParameter mqttMainTopicParam        = iotwebconf::TextParameter("Main Topic", "mqttTopic", mqttMainTopicValue, STRING_LEN,"Gateway1");




char irTimervalue[NUMBER_LEN];
iotwebconf::ParameterGroup irGroup        = iotwebconf::ParameterGroup("irConf", "Infrarot");
iotwebconf::NumberParameter irTimerParam  = iotwebconf::NumberParameter("Timer","irTimer",irTimervalue, NUMBER_LEN, "500", "0..2000", "min='0' max='2000' step='100'");

bool needMqttConnect = false;
bool needReset       = false;
int pinState         = HIGH;

uint32_t lastReport                = 0;
uint32_t lastMqttConnectionAttempt = 0;
uint32_t nextStatus                = 0;

void irSetup() {
  irrecv.enableIRIn();  // Start the receiver
}

uint32_t irTestTimer = 0;
uint32_t irLastValue;
uint32_t irLastTimer = 0;

void irLoop() {
  static char irBuffer[64];
  uint16_t geraet;
#if 1 == 2  // IR-Test Hardware
  pinMode(IR_SEND_PIN, OUTPUT);
  if (millis() > irTestTimer) {
    irTestTimer = millis() + 1000;
    Serial << "IR-Test Send Hardware" << endl;
    digitalWrite(IR_SEND_PIN, true);
    delay(1);
    digitalWrite(IR_SEND_PIN, false);
  }

#endif
#if 1 == 2  // IR-Test Send über IRRemote
  if (millis() > irTestTimer) {
    irTestTimer = millis() + 1000;
    // irsend.sendNEC(0x20DF1AE5, 32);
    // irsend.sendSAMSUNG(0xC2CA807F, 32);
    String sPayLoad = "0xC2CA807F";
    uint32_t irCode = strtoul(sPayLoad.c_str(), NULL, 16);
    Serial.printf("IR-Test-Send -- Samsung Code: 0x%X\n", irCode);
    irrecv.disableIRIn();
    irsend.sendSAMSUNG(irCode, 32);
    irrecv.enableIRIn();
  }
#endif

  if (irrecv.decode()) {
    geraet = irrecv.results.value >> 16;

    Serial << "IR-RECV --- Typ: " <<  irrecv.results.decode_type << " = " << irrecv.getProtocolString() << "\t  Bits: " <<  irrecv.results.bits << "\t Gerät: 0x" << _HEX(geraet) << "\t  Code: 0x" << _HEX( irrecv.results.value) << " = " <<  irrecv.results.value << endl;

    irrecv.resume();  // Receive the next value

    if ( (( irrecv.results.decode_type == NEC) || ( irrecv.results.decode_type == SAMSUNG)) && (! irrecv.results.isRepeat) && (geraet !=  0xF7) ) {  // nur wenn Type bekannt und keine Wiederholung
      if (( irrecv.results.value != irLastValue) || (millis() > irLastTimer)) {  // nur wenn neuer Code oder Timer schon abgelaufen
        irLastTimer = millis() + atoi(irTimervalue);
        irLastValue =  irrecv.results.value;
        snprintf(irBuffer, sizeof(irBuffer), "{\"typ\":%d,\"bits\":%d,\"code\":\"0x%X\"}",  irrecv.results.decode_type,  irrecv.results.bits,  irrecv.results.value);
        mqttClient.publish(topicIR, irBuffer);
      }

    }else{
      LedBeleuchtung( irrecv.results.value);  // E:\Arduino\LedBeleuchtung\IR_Remote1-ESP\IR_Remote1-ESP.ino
    }
  }

  if(Serial2.available())   // E:\Arduino\ATTINY\IRrecv2\IRrecv2.ino
  {
    uint32_t irCode = strtoul(Serial2.readString().c_str(),nullptr,16);
    geraet =irCode >> 16;
    if (geraet == 0xF7){
       Serial << "S2: " << _HEX(irCode) << " = " << irCode << "\t G:" << geraet << endl;
       LedBeleuchtung(irCode);
    }
  }
}



void setup() {
  Serial.begin(BAUD);
  while (!Serial && (millis() < 3000));
  Serial << "\n\n" << ProjektName << " - " << VERSION << "  (" << BUILDDATE << "  " __TIME__ << ")" << endl;

  Serial2.begin(115200);

  mqttGroup.addItem(&mqttServerParam);
  mqttGroup.addItem(&mqttUserNameParam);
  mqttGroup.addItem(&mqttUserPasswordParam);
  mqttGroup.addItem(&mqttMainTopicParam);

  iotWebConf.addParameterGroup(&mqttGroup);

  irGroup.addItem(&irTimerParam);
  iotWebConf.addParameterGroup(&irGroup);

  iotWebConf.setStatusPin(STATUS_PIN,HIGH);
  iotWebConf.setConfigPin(CONFIG_PIN);

  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.setWifiConnectionCallback(&wifiConnected);

  // iotWebConf.forceApMode(true);   // Wenn nur Access-Point gestartet werden soll
  iotWebConf.skipApStartup();  // Ohne Access-Point, sofort Station-Mode starten


  // -- Define how to handle updateServer calls.
  iotWebConf.setupUpdateServer(
    [](const char* updatePath) { httpUpdater.setup(&server, updatePath); },
    [](const char* userName, char* password) { httpUpdater.updateCredentials(userName, password); });

  // -- Initializing the configuration.
  bool validConfig = iotWebConf.init();

  if (!validConfig) {
    iotWebConf.getWifiSsidParameter();
    iotWebConf.getWifiPasswordParameter();

    // nicht notwendig, wird durch lib automatisch auf Default gesetzt
    /*
    mqttServerValue[0]       = '\0';
    mqttUserNameValue[0]     = '\0';
    mqttUserPasswordValue[0] = '\0';
    mqttMainTopicValue[0]    = '\0';

    */
  }

  // -- Set up required URL handlers on the web server.
  server.on("/", handleRoot);
  server.on("/config", [] { iotWebConf.handleConfig(); });
  server.onNotFound([]() { iotWebConf.handleNotFound(); });

  server.on("/restart",  doRestart);

  mqttClient.setBaseTopic(mqttMainTopicValue);
  mqttClient.setServer(mqttServerValue, 1883);
  mqttClient.setCallback(mqttCallback);

  irSetup();

  // Parameter für LED aus EEprom holen
  getPreferencesLED();
  KRGB.begin();
  ledStatus = true;
  rgbstrip.Begin();
  if (led_H < 10) led_H = 80;
  setRGB(led_R, led_G, led_B, led_H, ledStatus);

  Serial.println("Ready.");
}

void loop() {
  // -- doLoop should be called as frequently as possible.
  iotWebConf.doLoop();
  mqttClient.loop();
  irLoop();
  if (TvSim){
    TvSimulator();
  }
  if (Gradient) {
      showGradient();
  }


  if (needMqttConnect) {
    if (connectMqtt()) {
      needMqttConnect = false;
    }
  } else if ((iotWebConf.getState() == IOTWEBCONF_STATE_ONLINE) && (!mqttClient.connected())) {
    // Serial.println("MQTT reconnect");
    connectMqtt();
  }

  if (needReset) {
    Serial.println("Rebooting after 1 second.");
    iotWebConf.delay(1000);
    ESP.restart();
  }

  if (!needMqttConnect && (iotWebConf.getState() == IOTWEBCONF_STATE_ONLINE) && (millis() > nextStatus)) {
    nextStatus = millis() + (1000 * 60 * 10);  // 10min
    mqttClient.publish("IP", WiFi.localIP().toString());
  }
}

/**
 * Handle web requests to "/" path.
 */
void handleRoot() {
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal()) {
    // -- Captive portal request were already served.
    return;
  }
  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\", name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += (String)"<title>" + iotWebConf.getThingName() + "</title></head>";
  s += (String)"<body> <h3  style=\"display: inline;\">" + iotWebConf.getThingName() + "</h3> ";
  s += (String)"( " + ProjektName + "-" + VERSION + " &nbsp;&nbsp;- " + BUILDDATE +  " - "+ __TIME__ + ")<br>" ;
  s += "<ul>";
  s += "<li>MQTT server: ";
  s += mqttServerValue;
  s += "</ul>";
  s += "Gehe zur <a href='config'>Konfigurations Seite</a> um Werte zu ändern.";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}

void doRestart() {
  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\", name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += (String)"<title>" + iotWebConf.getThingName() + "</title></head>";
  s += "<body><br><br>Restart in 5 Sekunden";
  s += "</body></html>\n";
  server.send(200, "text/html", s);
  DEBUG_PRINTLN("Restart in 5 Sekunden");

  iotWebConf.delay(5000);
  WiFi.disconnect();
  delay(100);
  DEBUG_PRINTLN("Restart now");
  ESP.restart();
  delay(1000);
}

void wifiConnected() {
  needMqttConnect     = true;
  // WiFi.setHostname(thingName);
  Serial.print("Hostname:");
  Serial.println(WiFi.getHostname());
}

void configSaved() {
  Serial.println("Configuration was updated.");
  needReset = true;

}

bool formValidator(iotwebconf::WebRequestWrapper *webRequestWrapper) {
  Serial.println("Validating form.");
  bool valid = true;

  int l = webRequestWrapper->arg(mqttServerParam.getId()).length();
  if (l < 3) {
    mqttServerParam.errorMessage = "Please provide at least 3 characters!";
    valid                        = false;
  }
  return valid;
}

void setupMqttTopics() {
  uint16_t topicAnzahl = sizeof(strMqttTopics) / sizeof(strMqttTopics[0]);
  // DEBUG_VAR(topicAnzahl);
  for (size_t i = 0; i < topicAnzahl; i++) {
    mqttClient.subscribe(strMqttTopics[i].c_str(), 0);
  }
}

bool connectMqtt() {
  unsigned long now = millis();
  if (5000 > now - lastMqttConnectionAttempt) {
    // Do not repeat within 5 sec.
    return false;
  }
  Serial.println("Connecting to MQTT server...");
  if (!connectMqttOptions()) {
    lastMqttConnectionAttempt = now;
    return false;
  }
  Serial.println("MQTT Connected!");
  //mqttClient.subscribe("IR-Send/#");

  setupMqttTopics();
  mqttClient.publish("", "Online",true);  // Will-Topic überschreiben
  return true;
}

bool connectMqttOptions() {
  bool result;
  String mqttID = mqttClient.makeClientIDfromMac(iotWebConf.getThingName());
  Serial.print("mqttID:"); Serial.println(mqttID);

  if (mqttUserPasswordValue[0] != '\0') {
    result = mqttClient.connect(mqttID.c_str(), mqttUserNameValue, mqttUserPasswordValue, mqttMainTopicValue, 0, true, mqttWillMessage);
  } else {
    result = mqttClient.connect(mqttID.c_str(), mqttMainTopicValue, 0, true, mqttWillMessage);
  }
  return result;
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  int16_t topicNr = -1;
  String sPayLoad = "";

  if (payload != nullptr) {
    sPayLoad = (String((const char *)payload)).substring(0, length);
  }
  Serial.println("Incoming: " + String(topic) + " >" + sPayLoad + "<");

  for (size_t i = 0; i < (sizeof(strMqttTopics) / sizeof(strMqttTopics[0])); i++) {  // Topic in Liste suchen
    if (strstr(topic, strMqttTopics[i].c_str())) {
      topicNr = i;
      DEBUG_PRINTF("Topic: %s  Nr:%i\n", topic, topicNr);
      break;
    }
  }
  switch (topicNr) {
    case IR_SAMSUNG:
      DEBUG_PRINT("Samsung  ");
      if (sPayLoad.length() > 2) {  // nur wenn payload vorhanden
        uint32_t irCode = strtoul(sPayLoad.c_str(), NULL, 16);
        Serial.printf("IR_Send -- Code: 0x%X\n", irCode);
        irrecv.disableIRIn();
        irsend.sendSAMSUNG(irCode, 32);
        irrecv.enableIRIn();
      }
      break;

    case IR_NEC:
      DEBUG_PRINT("NEC  ");
      if (sPayLoad.length() > 2) {  // nur wenn payload vorhanden
        uint32_t irCode = strtoul(sPayLoad.c_str(), NULL, 16);
        Serial.printf("IR_Send -- Code: 0x%X\n", irCode);
        irrecv.disableIRIn();
        irsend.sendNEC(irCode, 32);
        irrecv.enableIRIn();
      }
      break;
    case RGB:
      ledStatus = true;
      TvSim     = false;
      Gradient  = false;
      if (payload[0] != '#') {
        //   11,22,33
        led_R = getCsvIntAtIndex(sPayLoad, 0);
        led_G = getCsvIntAtIndex(sPayLoad, 1);
        led_B = getCsvIntAtIndex(sPayLoad, 2);
        led_H = getCsvIntAtIndex(sPayLoad, 3);
      } else {
        // #112233
        sscanf(sPayLoad.substring(1, 3).c_str(), "%x", &led_R);
        sscanf(sPayLoad.substring(3, 5).c_str(), "%x", &led_G);
        sscanf(sPayLoad.substring(5, 7).c_str(), "%x", &led_B);
      }
      led_H = ((led_H < 0) | (led_H > 100)) ? 100 : led_H;
      DEBUG_PRINTF("R:%i  G%i  B%i  H%i\n", led_R, led_G, led_B, led_H);
      break;
    case HSB: {  // Klamer wegen "Error - crosses initialization"   https://forum.arduino.cc/index.php?topic=537887.0
      TvSim        = false;
      ledStatus    = true;
      Gradient     = false;
      uint16_t hue = getCsvIntAtIndex(sPayLoad, 0);
      uint16_t sat = getCsvIntAtIndex(sPayLoad, 1);
      uint16_t val = getCsvIntAtIndex(sPayLoad, 2);
      setHSB(hue, sat, val, ledStatus);
    } break;
    case FARBTEMP: {
      TvSim     = false;
      Gradient  = false;
      ledStatus = true;
      int kelvin = sPayLoad.toInt();
      kelvin = (kelvin<650) ? 650:kelvin;

      // colorType color = kelvin2RGB(kelvin);
      // led_R=color.r; led_G=color.g; led_B=color.b;
      // DEBUG_PRINTF("R:%i, G%i, B%i\n",led_R,led_G,led_B);

      KRGB.convert_NB(kelvin, led_H);
      led_R=KRGB.red();led_G=KRGB.green(); led_B=KRGB.blue();
      DEBUG_PRINTF("RGB:%x, R:%.0f, G%.0f, B%.0f\n",KRGB.RGB(),KRGB.red(),KRGB.green(),KRGB.blue());

      //setRGB(led_R, led_G, led_B, led_H, ledStatus);
    } break;
    case POWER:
      TvSim    = false;
      Gradient = false;
      if (sPayLoad == "ON")
        ledStatus = true;
      else if (sPayLoad == "OFF")
        ledStatus = false;
      else
        ledStatus = !ledStatus;
      if (led_H < 10) led_H = 50;
      break;
    case VAL:
      ledStatus = true;
      TvSim     = false;
      Gradient  = false;
      led_H     = sPayLoad.toInt();
      led_H     = ((led_H < 0) | (led_H > 100)) ? 100 : led_H;
      break;
    case VAL_UP:
      ledStatus = true;
      TvSim     = false;
      Gradient  = false;
      led_H     = led_H + 5;
      led_H     = (led_H > 100) ? 100 : led_H;
      break;
    case VAL_DOWN:
      ledStatus = true;
      TvSim     = false;
      Gradient  = false;
      led_H     = led_H - 5;
      led_H     = (led_H < 0) ? 0 : led_H;
      break;
    case SAVE:
      putPreferencesLED();
      break;
    case RELOAD:
      // Parameter aus EEprom holen
      getPreferencesLED();
      if (led_H < 10) led_H = 50;
      ledStatus = true;
      TvSim     = false;
      Gradient  = false;
      break;
    case GET_STATUS_LED:
      char buffer[100];
      snprintf(buffer, sizeof(buffer), "RGBH:%i,%i,%i,%i;TVSIM:%i;POWER:%i", led_R, led_G, led_B, led_H, TvSim, ledStatus);
      DEBUG_PRINTLN(buffer);
      mqttClient.publish(strMqttTopicsPub[STATUS_LED], buffer);
      break;
    case RESET:
      ledStatus = true;
      TvSim     = false;
      Gradient  = false;
      led_R     = 0xfa;
      led_G     = 0xa2;
      led_B     = 0x05;
      led_H     = 80;
      break;
    case GRADIENT:
      TvSim     = false;
      ledStatus = true;
      Gradient  = true;
      if (sPayLoad.substring(0, 6) == "DELAY,") {
        gradientDelay    = getCsvIntAtIndex(sPayLoad, 1);
        gradientDelay    = ((gradientDelay < 2) | (gradientDelay > 500)) ? 5 : gradientDelay;
        gradientHueDelta = getCsvIntAtIndex(sPayLoad, 2);
        gradientHueDelta = ((gradientHueDelta < 2) | (gradientHueDelta > 100)) ? 5 : gradientHueDelta;
        Serial << "Gardient Delay: " << gradientDelay << " HueDelta: " << gradientHueDelta << endl;
      }
      break;
    case TVSIM:
      ledStatus = false;
      TvSim     = true;
      Gradient  = false;
       // randomSeed(analogRead(A0));  // Nicht notwendig weil ESP anhand Wifi und Bluetooth selbst setzt
      pixelNum = random(numPixels);  // Begin at random point
      DEBUG_PRINTF("TV_Sim Startpixel: %i\n", pixelNum);
      break;

    default:
      DEBUG_PRINTF("Topic unbekannt: %s  Nr:%i\n", topic, topicNr);
      break;
  }
  if (topicNr < IR_SAMSUNG) {
    setRGB(led_R, led_G, led_B, led_H, ledStatus);
    sendStatRgb();
  }
}

void setRGB(uint8_t ledR, uint8_t ledG, uint8_t ledB, uint8_t ledH, bool ledStatus) {
  uint8_t lr, lg, lb;
  if (ledStatus) {
    lr = ledR;
    lg = ledG;
    lb = ledB;
  } else {
    lr = 0;
    lg = 0;
    lb = 0;
  }

  // Gammakorrektur und Helligkeit
  RgbColor color = {lr, lg, lb};
  rgbstrip.SetBrightness(ledH * 255 / 100);
  color = colorGamma.Correct(color);
  rgbstrip.ClearTo(color);  // schneller als Schleife mit SetPixelColor

  rgbstrip.Show();

  DEBUG(
    RgbColor colorB = rgbstrip.GetPixelColor(1);
    Serial << "ColorB (" << ledH * 255 / 100 << "): " << colorB.R << "," << colorB.G << "," << colorB.B << "\n";
    Serial << "ColorH (" << ledH << "): " << ledR * ledH / 100 << "," << ledG * ledH / 100 << "," << ledB * ledH / 100 << "\n";
    );
}

void setHSB(uint16_t hue, uint16_t sat, uint16_t val, bool ledStatus) {  // Bereich 0-359, 0-100, 0-100
  float fHue, fSat, fVal;
  fHue = hue / 359.0f;
  fSat = sat / 100.0f;
  fVal = val / 100.0f;

  Serial << _FLOAT(fHue, 3) << "," << _FLOAT(fSat, 2) << "," << _FLOAT(fVal, 2) << endl;
  for (int i = 0; i < RGB_PIXELS; i++) {
    rgbstrip.SetPixelColor(i, HsbColor(fHue, fSat, fVal));  // = HSV
  }
  yield();
  portDISABLE_INTERRUPTS();
  rgbstrip.Show();
  portENABLE_INTERRUPTS();
}

int getCsvIntAtIndex(String csv, uint8_t index) {
  //   0 1 2 3      index  , valueNo 3
  //   01234567     i ,  startchar
  //   a,b,c,d
  //          .              startchar 6
  // http://de.cppreference.com/w/cpp/string/byte/strtok
  int startChar = 0, valueNo = 0;
  for (int i = 0; i <= csv.length(); i++) {
    if ((csv[i] == ',') | (i == csv.length())) {
      if (valueNo == index) {
        return csv.substring(startChar, i).toInt();
      } else {
        startChar = i + 1;
        valueNo++;
      }
    }
  }
  return -1;
}

void sendStatRgb() {
  char tmp[20] = "#112233";  // RGB als Quittung publishen
  // snprintf(tmp, sizeof(tmp), "%i,%i,%i,%i", R, G, B, H);
  snprintf(tmp, sizeof(tmp) - 1, "#%02X%02X%02X", led_R, led_G, led_B);
  mqttClient.publish((strMqttTopicsPub[STATUS_RGB]).c_str(),tmp);

}

void putPreferencesLED() {                 // Parameter im EEprom speichern
  preferences.begin("LED_Status", false);  // see https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/examples/StartCounter/StartCounter.ino
  preferences.putUInt("LED_R", led_R);
  preferences.putUInt("LED_G", led_G);
  preferences.putUInt("LED_B", led_B);
  preferences.putUInt("LED_H", led_H);
  preferences.end();
}
void getPreferencesLED() {                 // Parameter aus EEprom holen, falls nicht vorhanden werden Defaultwerte gesetzt
  preferences.begin("LED_Status", false);  // see https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/examples/StartCounter/StartCounter.ino
  led_R = preferences.getUInt("LED_R", 0xfb);
  led_G = preferences.getUInt("LED_G", 0xa2);
  led_B = preferences.getUInt("LED_B", 0x05);
  led_H = preferences.getUInt("LED_H", 80);
  preferences.end();
}

void TvSimulator() {
#ifdef mitTVSIM
  uint32_t totalTime, fadeTime, holdTime, startTime, elapsed;
  uint16_t nr, ng, nb, r, g, b, i;
  uint8_t hi, lo, r8, g8, b8, frac;

  // Read next 16-bit (5/6/5) color
  hi = pgm_read_byte(&colors[pixelNum * 2]);
  lo = pgm_read_byte(&colors[pixelNum * 2 + 1]);
  if (++pixelNum >= numPixels) pixelNum = 0;

  // Expand to 24-bit (8/8/8)
  r8 = (hi & 0xF8) | (hi >> 5);
  g8 = (hi << 5) | ((lo & 0xE0) >> 3) | ((hi & 0x06) >> 1);
  b8 = (lo << 3) | ((lo & 0x1F) >> 2);
  // Apply gamma correction, further expand to 16/16/16
  nr = (uint8_t)pgm_read_byte(&gamma8[r8]) * 257;  // New R/G/B
  ng = (uint8_t)pgm_read_byte(&gamma8[g8]) * 257;
  nb = (uint8_t)pgm_read_byte(&gamma8[b8]) * 257;

  totalTime = random(250, 2500);     // Semi-random pixel-to-pixel time
  fadeTime  = random(0, totalTime);  // Pixel-to-pixel transition time
  if (random(10) < 3) fadeTime = 0;  // Force scene cut 30% of time
  holdTime = totalTime - fadeTime;   // Non-transition time
#if 1 == 2
  Serial << _FLOAT(millis() / 100, 1) << "\t pixelnum: " << pixelNum << "\t totalTime:" << totalTime << "\t fadeTime:" << fadeTime << "\t holdTime:" << holdTime;
  Serial << "\t Color  R:" << _HEX(nr) << "\t G:" << _HEX(ng) << "\tB:" << _HEX(nb) << endl;
#endif
  startTime = millis();
  for (;;) {
    elapsed = millis() - startTime;
    if (elapsed >= fadeTime) elapsed = fadeTime;
    if (fadeTime) {
      r = map(elapsed, 0, fadeTime, pr, nr);  // 16-bit interp
      g = map(elapsed, 0, fadeTime, pg, ng);
      b = map(elapsed, 0, fadeTime, pb, nb);
    } else {  // Avoid divide-by-zero in map()
      r = nr;
      g = ng;
      b = nb;
    }
    for (i = 0; i < TV_SIM_LEDS; i++) {
      r8   = r >> 8;  // Quantize to 8-bit
      g8   = g >> 8;
      b8   = b >> 8;
      frac = (i << 8) / TV_SIM_LEDS;                 // LED index scaled to 0-255
      if ((r8 < 255) && ((r & 0xFF) >= frac)) r8++;  // Boost some fraction
      if ((g8 < 255) && ((g & 0xFF) >= frac)) g8++;  // of LEDs to handle
      if ((b8 < 255) && ((b & 0xFF) >= frac)) b8++;  // interp > 8bit

      rgbstrip.SetPixelColor(TV_SIM_START_LED + i, RgbColor(r8, g8, b8));
    }

    rgbstrip.Show();

    if (elapsed >= fadeTime) break;
    iotWebConf.delay(1);  // TODO:  Notwendig????
    yield();              // TODO:  Notwendig????
  }
  // delay(holdTime);
  iotWebConf.delay(holdTime);


  pr = nr;  // Prev RGB = new RGB
  pg = ng;
  pb = nb;
#endif
}

/*
//TODO: umstellen auf https://github.com/RobTillaart/Kelvin2RGB
colorType kelvin2RGB(int kelvin) {
	double  temperature=0, red=0, green=0, blue=0;
	colorType color = {0,0,0};
	//color.r = 0; color.g = 0; color.b = 0;

	if (kelvin == 6500 ) {
		// Hard fit at 6500K.
		return {255,255,255};
	}
	temperature = double(kelvin) * 0.01;
	if ( kelvin < 6500 ){
		color.r = 255;
			// a + b x + c Log[x] /.
			// {a -> -155.25485562709179`,
			// b -> -0.44596950469579133`,
			// c -> 104.49216199393888`,
			// x -> (kelvin/100) - 2}
			green = temperature - 2;
			color.g = (uint8_t) (-155.25485562709179 - 0.44596950469579133*green + 104.49216199393888*log(green));

		if (kelvin > 1950) {
			// a + b x + c Log[x] /.
			// {a -> -254.76935184120902`,
			// b -> 0.8274096064007395`,
			// c -> 115.67994401066147`,
			// x -> kelvin/100 - 10}
			blue = temperature - 10;
			color.b = (uint8_t)(-254.76935184120902 + 0.8274096064007395*blue + 115.67994401066147*log(blue));
		}
		return color;
	}
	color.b = 255;
	// a + b x + c Log[x] /.
	// {a -> 351.97690566805693`,
	// b -> 0.114206453784165`,
	// c -> -40.25366309332127
	//x -> (kelvin/100) - 55}
	red = temperature - 55;
	color.r = (uint8_t)(351.97690566805693 + 0.114206453784165*red - 40.25366309332127*log(red));
	// a + b x + c Log[x] /.
	// {a -> 325.4494125711974`,
	// b -> 0.07943456536662342`,
	// c -> -28.0852963507957`,
	// x -> (kelvin/100) - 50}
	green = temperature - 50;
	color.g = (uint8_t)(325.4494125711974 + 0.07943456536662342*green - 28.0852963507957*log(green));
	return color;
}
*/

void showGradient() {
	static uint8_t hue;
	hue++;
	// Use HSV to create nice gradient
	//Serial << "HUE:" << hue << ": ";
	for ( int i = 0; i != RGB_PIXELS; i++ ){
			float  lhue = (hue +  gradientHueDelta * i)/360.0f;
			//Serial << _FLOAT(lhue,3) << "|";
			rgbstrip.SetPixelColor(i,HsbColor( lhue , 1.0f, 0.2f));   // = HSV
	}
   //Serial << endl;
  iotWebConf.delay(gradientDelay);   //TODO: Auf Version ohne Delay umbauen
  yield();
	portDISABLE_INTERRUPTS();
	rgbstrip.Show();
	portENABLE_INTERRUPTS();

	//Serial << "time:" << millis() <<endl;
 }

#if 1==1
#define IR_CODE_LED_ON      0x00F7C03F
#define IR_CODE_LED_OFF     0x00F740BF

#define IR_CODE_LED_UP      0x00F700FF
#define IR_CODE_LED_DOWN    0x00F7807F

 void LedBeleuchtung(uint32_t irCode) {
  TvSim = false;
	ledStatus = true;
	switch (irCode) {
    case IR_CODE_LED_ON:
      Serial << "IR LED_ON" << endl;
      ledStatus = true;
      TvSim = false;
      Gradient = false;
      if (led_H < 10) led_H = 50;
       break;
    case IR_CODE_LED_OFF:
     Serial << "IR LED_OFF" << endl;
      ledStatus = false;
      TvSim = false;
      Gradient = false;
      break;

		case IR_CODE_LED_UP:
			if ((led_H <= 95) & (led_H > 49)) {
				led_H += 5;
			}
			if (led_H <= 50) {
				led_H += 1;
			}
			ledStatus = true;
			break;
		case IR_CODE_LED_DOWN:
			if (led_H > 49) {
				led_H -= 5;
			}
			else {
				if (led_H > 0) {
					led_H -= 1;
				}
			}
			ledStatus = true;
			break;


		case 0x00F720DF:   //Rot 100%
			led_R = 255;
			led_G = 0;
			led_B = 0;
			break;
		case 0x00F7A05F:    //Grün 100%
			led_R = 0;
			led_G = 255;
			led_B = 0;
			break;
		case 0x00F7609F:   //Blau 100%
			led_R = 0;
			led_G = 0;
			led_B = 255;
			break;
		case 0x00F7E01F:   //Weiss 100%
			led_R = 255;
			led_G = 255;
			led_B = 255;
		  led_H = 100;
			break;

		case 0x00F710EF:    //Rot 80%
			led_R = 255;
			led_G = 50;
			led_B = 0;
			break;
		case 0x00F7906F:   //Grün 80%
			led_R = 0;
			led_G = 255;
			led_B = 100;
			break;
		case  0x00F750AF:   //Blau 80%
			led_R = 50;
			led_G = 0;
			led_B = 255;
			break;

		case 0x00F730CF:   //Rot 60%
			led_R = 255;
			led_G = 100;
			led_B = 0;
			break;
		case 0x00F7B04F:   //Grün 60%
			led_R = 0;
			led_G = 255;
			led_B = 200;
			break;
		case 0x00F7708F:   //Blau 60%
			led_R = 100;
			led_G = 0;
			led_B = 255;
			break;

		case  0x00F708F7:    //Rot 40%
			led_R = 255;
			led_G = 150;
			led_B = 0;
			break;
		case 0x00F78877:   //Grün 40%
			led_R = 0;
			led_G = 200;
			led_B = 250;
			break;
		case 0x00F748B7:     //Blau 40%
			led_R = 150;
			led_G = 0;
			led_B = 255;
			break;

		case 0x00F728D7:      //Rot 20%
			led_R = 255;
			led_G = 200;
			led_B = 0;
			break;
		case 0x00F7A857:    //Grün 20%
			led_R = 0;
			led_G = 155;
			led_B = 255;
			break;
		case 0x00F76897:   //Blau 20%
			led_R = 200;
			led_G = 0;
			led_B = 255;
			break;


		case 0x00F7D02F:   //Save        FLASH
			putPreferencesLED();
			break;

		case 0x00F7F00F:   //Recall      STROBE
			getPreferencesLED();
			break;

		case  0x00F7C837:  // TV Simulator einschalten      FADE
			TvSim = true;
			ledStatus = false;
      pixelNum = random(numPixels);  // Begin at random point
      DEBUG_PRINTF("TV_Sim Startpixel: %i\n", pixelNum);
			break;

    case 0x00F7E817:   //Reset       SMOOTH
      led_R     = 0xcc; //0xfa;
      led_G     = 0x7e; //0xa2;
      led_B     = 0x32; //0x05;
      led_H     = 100;
      TvSim = false;
			ledStatus = true;
      break;

		default:
			break;
	}

	//setLedStripColor(led_R, led_G, led_B, led_H, ledStatus);

DEBUG(
	//Serial << _FLOAT(millis() / 100, 1) << " IR:" << results.value << " 0x" << _HEX(results.value) << endl;
	Serial << "RGB#H:" << led_R << "/" << led_G << "/" << led_B << "#" << led_H << " St:" << ledStatus << endl;
);
  setRGB(led_R, led_G, led_B, led_H, ledStatus);
  sendStatRgb();
}

#endif