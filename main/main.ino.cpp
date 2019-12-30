# 1 "/var/folders/7y/gbgqcg811pb8pb_00y83837hzsxdyx/T/tmpFMukFc"
#include <Arduino.h>
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/main.ino"
# 28 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/main.ino"
#include "User_config.h"


#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define array_size 12
unsigned long ReceivedSignal[array_size][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};

unsigned long timer_sys_measures = 0;
#else
#define array_size 4
unsigned long ReceivedSignal[array_size][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
#endif

#include <PubSubClient.h>
#include <ArduinoJson.h>


#if defined(ZgatewayRF) || defined(ZgatewayRF2) || defined(ZgatewayPilight)
#include "config_RF.h"
#endif
#ifdef ZgatewayLORA
#include "config_LORA.h"
#endif
#ifdef ZgatewaySRFB
#include "config_SRFB.h"
#endif
#ifdef ZgatewayBT
#include "config_BT.h"
#endif
#ifdef ZgatewayIR
#include "config_IR.h"
#endif
#ifdef Zgateway2G
#include "config_2G.h"
#endif
#ifdef ZactuatorONOFF
#include "config_ONOFF.h"
#endif
#ifdef ZsensorINA226
#include "config_INA226.h"
#endif
#ifdef ZsensorHCSR501
#include "config_HCSR501.h"
#endif
#ifdef ZsensorADC
#include "config_ADC.h"
#endif
#ifdef ZsensorBH1750
#include "config_BH1750.h"
#endif
#ifdef ZsensorTSL2561
#include "config_TSL2561.h"
#endif
#ifdef ZsensorBME280
#include "config_BME280.h"
#endif
#ifdef ZsensorHCSR04
#include "config_HCSR04.h"
#endif
#ifdef ZsensorDHT
#include "config_DHT.h"
#endif
#ifdef ZgatewayRFM69
#include "config_RFM69.h"
#endif
#ifdef ZsensorGPIOInput
#include "config_GPIOInput.h"
#endif
#ifdef ZsensorGPIOKeyCode
#include "config_GPIOKeyCode.h"
#endif
#ifdef ZmqttDiscovery
#include "config_mqttDiscovery.h"
#endif
#ifdef ZactuatorFASTLED
#include "config_FASTLED.h"
#endif




void callback(char *topic, byte *payload, unsigned int length);

bool connectedOnce = false;

int failure_number = 0;

#ifdef ESP32
  #include <FS.h>
  #include "SPIFFS.h"
  #include <WiFi.h>
  #include <ArduinoOTA.h>
  #include <WiFiUdp.h>
  WiFiClient eClient;
  #include <WiFiManager.h>
#ifdef MDNS_SD
  #include <ESPmDNS.h>
#endif
#elif defined(ESP8266)
  #include <FS.h>
  #include <ESP8266WiFi.h>
  #include <ArduinoOTA.h>
  #include <DNSServer.h>
  #include <ESP8266WebServer.h>
  #include <WiFiManager.h>
  WiFiClient eClient;
#ifdef MDNS_SD
  #include <ESP8266mDNS.h>
#endif
#else
  #include <Ethernet.h>
  EthernetClient eClient;
#endif


PubSubClient client(eClient);


unsigned long lastMQTTReconnectAttempt = 0;
unsigned long lastNTWKReconnectAttempt = 0;
void revert_hex_data(char *in, char *out, int l);
void extract_char(char *token_char, char *subset, int start, int l, bool reverse, bool isNumber);
int strpos(char *haystack, char *needle);
bool to_bool(String const &s);
void trc(String msg);
void trc(int msg);
void trc(unsigned int msg);
void trc(long msg);
void trc(unsigned long msg);
void trc(double msg);
void trc(float msg);
void trc(JsonObject &data);
void pub(char *topicori, char *payload, bool retainFlag);
void pub(char *topicori, JsonObject &data);
void pub(char *topicori, char *payload);
void pub_custom_topic(char *topicori, JsonObject &data, boolean retain);
void pubMQTT(char * topic, char * payload);
void pubMQTT(char * topicori, char * payload, bool retainFlag);
void pubMQTT(String topic, char * payload);
void pubMQTT(char *topic, unsigned long payload);
void pubMQTT(char *topic, String payload);
void pubMQTT(String topic, String payload);
void pubMQTT(String topic, int payload);
void pubMQTT(String topic, float payload);
void pubMQTT(char *topic, float payload);
void pubMQTT(char *topic, int payload);
void pubMQTT(char *topic, unsigned int payload);
void pubMQTT(char *topic, long payload);
void pubMQTT(char *topic, double payload);
void pubMQTT(String topic, unsigned long payload);
bool cmpToMainTopic(char * topicOri, char * toAdd);
void reconnect();
void setup_parameters();
void setup();
void setup_wifi();
void saveConfigCallback();
void checkButton();
void eraseAndRestart();
void setup_wifimanager(bool reset_settings);
void setup_ethernet();
void connectMQTTmdns();
void loop();
void stateMeasures();
void storeValue(unsigned long MQTTvalue);
int getMin();
bool isAduplicate(unsigned long value);
void receivingMQTT(char *topicOri, char *datacallback);
void MQTTtoSYS(char *topicOri, JsonObject &SYSdata);
void setupZsensorBH1750();
void MeasureLightIntensity();
void setupFASTLED();
int animation_step(int duration, int steps);
int animation_step_count(int duration, int steps);
void FASTLEDLoop();
boolean FASTLEDtoMQTT();
void MQTTtoFASTLEDJSON(char *topicOri, JsonObject &jsonData);
void MQTTtoFASTLED(char *topicOri, char *datacallback);
void Fire2012WithPalette();
void MQTTtoONOFF(char *topicOri, JsonObject &ONOFFdata);
void MQTTtoONOFF(char *topicOri, char *datacallback);
void setup2G();
void setupGSM(bool deleteSMS);
void signalStrengthAnalysis();
bool _2GtoMQTT();
void MQTTto2G(char *topicOri, char *datacallback);
void MQTTto2G(char *topicOri, JsonObject &SMSdata);
void setWorBMac(char *mac, bool isWhite);
bool oneWhite();
bool isWhite(char *mac);
bool isBlack(char *mac);
bool isDiscovered(char *mac);
void dumpDevices();
void strupp(char *beg);
void MiFloraDiscovery(char *mac);
void VegTrugDiscovery(char *mac);
void MiJiaDiscovery(char *mac);
void LYWSD02Discovery(char *mac);
void CLEARGRASSTRHDiscovery(char *mac);
void CLEARGRASSTRHKPADiscovery(char *mac);
void MiScaleDiscovery(char *mac);
void MiLampDiscovery(char *mac);
void MiBandDiscovery(char *mac);
void BLEscan();
void coreTask(void *pvParameters);
void setupBT();
bool BTtoMQTT();
void setupBT();
bool BTtoMQTT();
double value_from_service_data(char *service_data, int offset, int data_length);
bool process_sensors(int offset, char *rest_data, char *mac_adress);
bool process_scale_v1(char *rest_data, char *mac_adress);
bool process_scale_v2(char *rest_data, char *mac_adress);
bool process_miband(char *rest_data, char *mac_adress);
bool process_milamp(char *rest_data, char *mac_adress);
bool process_cleargrass_air(char *rest_data, char *mac_adress);
bool process_cleargrass(char *rest_data, char *mac_adress);
void haRoomPresence(JsonObject &HomePresence);
void MQTTtoBT(char *topicOri, JsonObject &BTdata);
void setupIR();
void IRtoMQTT();
void MQTTtoIR(char *topicOri, char *datacallback);
void MQTTtoIR(char *topicOri, JsonObject &IRdata);
bool sendIdentifiedProtocol(const char *protocol_name, unsigned long long data, const char *datastring, unsigned int valueBITS, uint16_t valueRPT);
void setupLORA();
void LORAtoMQTT();
void MQTTtoLORA(char *topicOri, JsonObject &LORAdata);
void MQTTtoLORA(char *topicOri, char *LORAdata);
void pilightCallback(const String &protocol, const String &message, int status,
                     size_t repeats, const String &deviceID);
void setupPilight();
void PilighttoMQTT();
void MQTTtoPilight(char *topicOri, JsonObject &Pilightdata);
void RFtoMQTTdiscovery(unsigned long MQTTvalue);
void setupRF();
void RFtoMQTT();
void MQTTtoRF(char *topicOri, char *datacallback);
void MQTTtoRF(char *topicOri, JsonObject &RFdata);
void setupRF2();
void RF2toMQTT();
void rf2Callback(unsigned int period, unsigned long address, unsigned long groupBit, unsigned long unit, unsigned long switchType);
void MQTTtoRF2(char *topicOri, char *datacallback);
void MQTTtoRF2(char *topicOri, JsonObject &RF2data);
uint32_t gc_checksum();
void eeprom_setup();
void setupRFM69(void);
bool RFM69toMQTT(void);
void MQTTtoRFM69(char *topicOri, char *datacallback);
void MQTTtoRFM69(char *topicOri, JsonObject &RFM69data);
void setupSRFB();
void _rfbSend(byte *message);
void _rfbSend(byte *message, int times);
bool SRFBtoMQTT();
void _rfbDecode();
void _rfbAck();
bool _rfbToArray(const char *in, byte *out);
bool _rfbToChar(byte *in, char *out);
void MQTTtoSRFB(char *topicOri, char *datacallback);
void MQTTtoSRFB(char *topicOri, JsonObject &SRFBdata);
String getMacAddress();
String getUniqueId(String name, String sufix);
void createDiscovery(char *sensor_type,
                     char *st_topic, char *s_name, char *unique_id,
                     char *availability_topic, char *device_class, char *value_template,
                     char *payload_on, char *payload_off, char *unit_of_meas,
                     int off_delay,
                     char *payload_available, char *payload_not_avalaible, bool child_device, char *cmd_topic);
void pubMqttDiscovery();
void MeasureADC();
void setupZsensorBME280();
void MeasureTempHumAndPressure();
void MeasureTempAndHum();
void setupGPIOInput();
void MeasureGPIOInput();
void setupGPIOKeyCode();
void MeasureGPIOKeyCode();
void setupHCSR04();
void MeasureDistance();
void setupHCSR501();
void MeasureHCSR501();
void setupINA226();
void MeasureINA226();
static void writeRegister(byte reg, word value);
static word readRegister(byte reg);
void displaySensorDetails(void);
void setupZsensorTSL2561();
void MeasureLightIntensityTSL2561();
#line 149 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/main.ino"
void revert_hex_data(char *in, char *out, int l)
{

  int i = l - 2, j = 0;
  while (i != -2)
  {
    if (i % 2 == 0)
      out[j] = in[i + 1];
    else
      out[j] = in[i - 1];
    j++;
    i--;
  }
  out[l - 1] = '\0';
}

void extract_char(char *token_char, char *subset, int start, int l, bool reverse, bool isNumber)
{
  char tmp_subset[l + 1];
  memcpy(tmp_subset, &token_char[start], l);
  tmp_subset[l] = '\0';
  if (isNumber)
  {
    char tmp_subset2[l + 1];
    if (reverse)
      revert_hex_data(tmp_subset, tmp_subset2, l + 1);
    else
      strncpy(tmp_subset2, tmp_subset, l + 1);
    long long_value = strtoul(tmp_subset2, NULL, 16);
    sprintf(tmp_subset2, "%ld", long_value);
    strncpy(subset, tmp_subset2, l + 1);
  }
  else
  {
    if (reverse)
      revert_hex_data(tmp_subset, subset, l + 1);
    else
      strncpy(subset, tmp_subset, l + 1);
  }
  subset[l] = '\0';
}

int strpos(char *haystack, char *needle)
{
  char *p = strstr(haystack, needle);
  if (p)
    return p - haystack;
  return -1;
}

char *ip2CharArray(IPAddress ip)
{
  static char a[16];
  sprintf(a, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  return a;
}

bool to_bool(String const &s)
{
  return s != "0";
}


void trc(String msg)
{
  #ifdef TRACE
  Serial.println(msg);
  digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
  pubMQTT(subjectTRACEtoMQTT, msg);
  #endif
}

void trc(int msg)
{
  #ifdef TRACE
  Serial.println(msg);
  digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
  pubMQTT(subjectTRACEtoMQTT, msg);
  #endif
}

void trc(unsigned int msg)
{
  #ifdef TRACE
  Serial.println(msg);
  digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
  pubMQTT(subjectTRACEtoMQTT, msg);
  #endif
}

void trc(long msg)
{
  #ifdef TRACE
  Serial.println(msg);
  digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
  pubMQTT(subjectTRACEtoMQTT, msg);
  #endif
}

void trc(unsigned long msg)
{
  #ifdef TRACE
  Serial.println(msg);
  digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
  pubMQTT(subjectTRACEtoMQTT, msg);
  #endif
}

void trc(double msg)
{
  #ifdef TRACE
  Serial.println(msg);
  digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
  pubMQTT(subjectTRACEtoMQTT, msg);
  #endif
}

void trc(float msg)
{
  #ifdef TRACE
  Serial.println(msg);
  digitalWrite(led_info, HIGH);
  #endif
  #ifdef subjectTRACEtoMQTT
  pubMQTT(subjectTRACEtoMQTT, msg);
  #endif
}

void trc(JsonObject &data)
{
  char JSONmessageBuffer[JSON_MSG_BUFFER];
  data.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  trc(JSONmessageBuffer);
}

void pub(char *topicori, char *payload, bool retainFlag)
{
  String topic = String(mqtt_topic) + String(topicori);
  pubMQTT((char *)topic.c_str(), payload, retainFlag);
}

void pub(char *topicori, JsonObject &data)
{
  if (client.connected())
  {
    digitalWrite(led_receive, HIGH);
    String topic = String(mqtt_topic) + String(topicori);
#ifdef valueAsASubject
    unsigned long value = data["value"];
    if (value != 0)
    {
      topic = topic + "/" + String(value);
    }
#endif

#ifdef jsonPublishing
    char JSONmessageBuffer[JSON_MSG_BUFFER];
    trc(F("Pub json into:"));
    trc(topic);
    data.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
    trc(JSONmessageBuffer);
    pubMQTT(topic, JSONmessageBuffer);
#endif

#ifdef simplePublishing
    trc(F("Pub data per topic"));

    for (JsonPair &p : data)
    {
      #if defined(ESP8266)
      yield();
      #endif
      if (p.value.is<unsigned long>() && strcmp(p.key, "rssi") != 0)
      {
        trc(p.key);
        trc(p.value.as<unsigned long>());
        if (strcmp(p.key, "value") == 0)
        {
          pubMQTT(topic, p.value.as<unsigned long>());
        }
        else
        {
          pubMQTT(topic + "/" + String(p.key), p.value.as<unsigned long>());
        }
      }
      else if (p.value.is<int>())
      {
        trc(p.key);
        trc(p.value.as<int>());
        pubMQTT(topic + "/" + String(p.key), p.value.as<int>());
      }
      else if (p.value.is<float>())
      {
        trc(p.key);
        trc(p.value.as<float>());
        pubMQTT(topic + "/" + String(p.key), p.value.as<float>());
      }
      else if (p.value.is<char *>())
      {
        trc(p.key);
        trc(p.value.as<const char *>());
        pubMQTT(topic + "/" + String(p.key), p.value.as<const char *>());
      }
    }
#endif
  }
  else
  {
    trc("client not connected can't pub");
  }
}

void pub(char *topicori, char *payload)
{
  if (client.connected())
  {
    String topic = String(mqtt_topic) + String(topicori);
    trc(F("Pub ack into:"));
    trc(topic);
    pubMQTT(topic, payload);
  }
  else
  {
    trc("client not connected can't pub");
  }
}

void pub_custom_topic(char *topicori, JsonObject &data, boolean retain)
{
  if (client.connected())
  {
    char JSONmessageBuffer[JSON_MSG_BUFFER];
    trc(F("Pub json discovery into:"));
    trc(topicori);
    data.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
    trc(JSONmessageBuffer);
    pubMQTT(topicori, JSONmessageBuffer, retain);
  }
  else
  {
    trc("client not connected can't pub");
  }
}


void pubMQTT(char * topic, char * payload)
{
    client.publish(topic, payload);
}

void pubMQTT(char * topicori, char * payload, bool retainFlag)
{
  client.publish(topicori, payload, retainFlag);
}

void pubMQTT(String topic, char * payload)
{
    client.publish((char *)topic.c_str(),payload);
}

void pubMQTT(char *topic, unsigned long payload)
{
  char val[11];
  sprintf(val, "%lu", payload);
  client.publish(topic, val);
}

void pubMQTT(char *topic, String payload)
{
  client.publish(topic, (char *)payload.c_str());
}

void pubMQTT(String topic, String payload)
{
  client.publish((char *)topic.c_str(), (char *)payload.c_str());
}

void pubMQTT(String topic, int payload)
{
  char val[12];
  sprintf(val, "%d", payload);
  client.publish((char *)topic.c_str(), val);
}

void pubMQTT(String topic, float payload)
{
  char val[12];
  dtostrf(payload, 3, 1, val);
  client.publish((char *)topic.c_str(), val);
}

void pubMQTT(char *topic, float payload)
{
  char val[12];
  dtostrf(payload, 3, 1, val);
  client.publish(topic, val);
}

void pubMQTT(char *topic, int payload)
{
  char val[6];
  sprintf(val, "%d", payload);
  client.publish(topic, val);
}

void pubMQTT(char *topic, unsigned int payload)
{
  char val[6];
  sprintf(val, "%u", payload);
  client.publish(topic, val);
}

void pubMQTT(char *topic, long payload)
{
  char val[11];
  sprintf(val, "%l", payload);
  client.publish(topic, val);
}

void pubMQTT(char *topic, double payload)
{
  char val[16];
  sprintf(val, "%d", payload);
  client.publish(topic, val);
}

void pubMQTT(String topic, unsigned long payload)
{
  char val[11];
  sprintf(val, "%lu", payload);
  client.publish((char *)topic.c_str(), val);
}

bool cmpToMainTopic(char * topicOri, char * toAdd){
  char topic[mqtt_topic_max_size];
  strcpy(topic,mqtt_topic);
  strcat(topic,toAdd);
  if (strstr(topicOri,topic) != NULL){
    return true;
  }else{
    return false;
  }
}

void reconnect()
{

#ifndef ESPWifiManualSetup
  #if defined(ESP8266) || defined(ESP32)
    checkButton();
  #endif
#endif


  while (!client.connected())
  {
    trc(F("MQTT connection..."));
    char topic[mqtt_topic_max_size];
    strcpy(topic,mqtt_topic);
    strcat(topic,will_Topic);
    if (client.connect(gateway_name, mqtt_user, mqtt_pass, topic, will_QoS, will_Retain, will_Message))
    {
      trc(F("Connected to broker"));
      failure_number = 0;

      pub(will_Topic, Gateway_AnnouncementMsg, will_Retain);

      pub(version_Topic, OMG_VERSION, will_Retain);

      char topic2[mqtt_topic_max_size];
      strcpy(topic2,mqtt_topic);
      strcat(topic2,subjectMQTTtoX);
      if (client.subscribe(topic2))
      {
        #ifdef ZgatewayRF
        client.subscribe(subjectMultiGTWRF);
        #endif
        #ifdef ZgatewayIR
        client.subscribe(subjectMultiGTWIR);
        #endif
        trc(F("Subscription OK to the subjects"));
      }
    }
    else
    {
      failure_number++;
      trc(F("failure_number"));
      trc(failure_number);
      if (failure_number > maxMQTTretry && !connectedOnce)
      {
      #ifndef ESPWifiManualSetup
        #if defined(ESP8266) || defined(ESP32)
        trc(F("failed connecting first time to mqtt, reset wifi manager  & erase network credentials"));
        setup_wifimanager(true);
        #endif
      #endif
      }
      trc(F("failed, rc="));
      trc(client.state());
      delay(5000);

    #if defined(ESP32)
      WiFi.begin();
    #endif
    }
  }
}


void callback(char *topic, byte *payload, unsigned int length)
{



  trc(F("Hey I got a callback "));

  byte *p = (byte *)malloc(length + 1);

  memcpy(p, payload, length);

  p[length] = '\0';

  if ((strstr(topic, subjectMultiGTWKey) != NULL) || (strstr(topic, subjectGTWSendKey) != NULL))
    receivingMQTT(topic, (char *)p);

  free(p);
}

void setup_parameters()
{
  strcat(mqtt_topic, gateway_name);
}

void setup()
{

  Serial.begin(SERIAL_BAUD);

#if defined(ESP8266) || defined(ESP32)
  #ifdef ESP8266
    #ifndef ZgatewaySRFB
    Serial.end();
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_TX_ONLY);
    #endif
  #endif

  #if defined(ESPWifiManualSetup)
  setup_wifi();
  #else
  setup_wifimanager(false);
  #endif

  trc(F("OpenMQTTGateway mac: "));
  trc(WiFi.macAddress());

  trc(F("OpenMQTTGateway ip: "));
  trc(WiFi.localIP().toString());


  ArduinoOTA.setPort(ota_port);


  ArduinoOTA.setHostname(ota_hostname);


  ArduinoOTA.setPassword(ota_password);

  ArduinoOTA.onStart([]() {
    trc(F("Start"));
  });
  ArduinoOTA.onEnd([]() {
    trc(F("\nEnd"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      trc(F("Auth Failed"));
    else if (error == OTA_BEGIN_ERROR)
      trc(F("Begin Failed"));
    else if (error == OTA_CONNECT_ERROR)
      trc(F("Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR)
      trc(F("Receive Failed"));
    else if (error == OTA_END_ERROR)
      trc(F("End Failed"));
  });
  ArduinoOTA.begin();

  #else


  Serial.begin(SERIAL_BAUD);

  setup_ethernet();
  #endif


  pinMode(led_receive, OUTPUT);
  pinMode(led_send, OUTPUT);
  pinMode(led_info, OUTPUT);
  digitalWrite(led_receive, LOW);
  digitalWrite(led_send, LOW);
  digitalWrite(led_info, LOW);

  #if defined(MDNS_SD) && defined(ESP8266)
  trc(F("Connecting to MQTT by mDNS without mqtt hostname"));
  connectMQTTmdns();
  #else
  long port;
  port = strtol(mqtt_port, NULL, 10);
  trc(port);
  #ifdef mqtt_server_name
    trc(F("Connecting to MQTT with mqtt hostname"));
    IPAddress mqtt_server_ip;
    WiFi.hostByName(mqtt_server_name, mqtt_server_ip);
    client.setServer(mqtt_server_ip, port);
    trc(mqtt_server_ip.toString());
  #else
    trc(F("Connecting to MQTT by IP adress"));
    client.setServer(mqtt_server, port);
    trc(mqtt_server);
  #endif
  #endif

  setup_parameters();

  client.setCallback(callback);

  delay(1500);

  lastMQTTReconnectAttempt = 0;
  lastNTWKReconnectAttempt = 0;

  #ifdef ZsensorBME280
  setupZsensorBME280();
  #endif
  #ifdef ZsensorBH1750
  setupZsensorBH1750();
  #endif
  #ifdef ZsensorTSL2561
  setupZsensorTSL2561();
  #endif
  #ifdef Zgateway2G
  setup2G();
  #endif
  #ifdef ZgatewayIR
  setupIR();
  #endif
  #ifdef ZgatewayLORA
  setupLORA();
  #endif
  #ifdef ZgatewayRF
  setupRF();
  #endif
  #ifdef ZgatewayRF2
  setupRF2();
  #endif
  #ifdef ZgatewayPilight
  setupPilight();
  #endif
  #ifdef ZgatewaySRFB
  setupSRFB();
  #endif
  #ifdef ZgatewayBT
  setupBT();
  #endif
  #ifdef ZgatewayRFM69
  setupRFM69();
  #endif
  #ifdef ZsensorINA226
  setupINA226();
  #endif
  #ifdef ZsensorHCSR501
  setupHCSR501();
  #endif
  #ifdef ZsensorHCSR04
  setupHCSR04();
  #endif
  #ifdef ZsensorGPIOInput
  setupGPIOInput();
  #endif
  #ifdef ZsensorGPIOKeyCode
  setupGPIOKeyCode();
  #endif
  #ifdef ZactuatorFASTLED
  setupFASTLED();
  #endif

  trc(F("MQTT_MAX_PACKET_SIZE"));
  trc(MQTT_MAX_PACKET_SIZE);
  #if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  if (MQTT_MAX_PACKET_SIZE == 128)
    trc(F("WRONG PUBSUBCLIENT LIBRARY USED PLEASE INSTALL THE ONE FROM RELEASE PAGE"));
  #endif
  trc(F("Setup OpenMQTTGateway end"));
}

#if defined(ESPWifiManualSetup)
void setup_wifi()
{
  delay(10);
  int failureAttempt = 0;
  WiFi.mode(WIFI_STA);

  trc(F("Connecting to "));
  trc(wifi_ssid);
  #ifdef ESPWifiAdvancedSetup
  IPAddress ip_adress(ip);
  IPAddress gateway_adress(gateway);
  IPAddress subnet_adress(subnet);
  IPAddress dns_adress(Dns);
  if (!WiFi.config(ip_adress, gateway_adress, subnet_adress, dns_adress))
  {
    trc("STA Failed to configure");
  }
  WiFi.begin(wifi_ssid, wifi_password);
  #else
  WiFi.begin(wifi_ssid, wifi_password);
  #endif

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    trc(F("."));
    failureAttempt++;
    if (failureAttempt > 30)
      setup_wifi();
  }
  trc(F("WiFi ok with manual config credentials"));
}

#elif defined(ESP8266) || defined(ESP32)

WiFiManager wifiManager;


bool shouldSaveConfig = true;



void saveConfigCallback()
{
  trc(F("Should save config"));
  shouldSaveConfig = true;
}

void checkButton()
{

  if (digitalRead(TRIGGER_PIN) == LOW)
  {

    delay(50);
    if (digitalRead(TRIGGER_PIN) == LOW)
    {
      trc(F("Trigger button Pressed"));

      delay(3000);
      if (digitalRead(TRIGGER_PIN) == LOW)
      {
        trc(F("Button Held"));
        trc(F("Erasing ESP Config, restarting"));
        setup_wifimanager(true);
      }
    }
  }
}

void eraseAndRestart()
{
  #if defined(ESP8266)
  WiFi.disconnect(true);
  #else
  WiFi.disconnect(true, true);
  #endif

  trc("Formatting requested, result:");
  trc(SPIFFS.format());

  #if defined(ESP8266)
  ESP.reset();
  #else
  ESP.restart();
  #endif
}

void setup_wifimanager(bool reset_settings)
{

  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  if (reset_settings)
    eraseAndRestart();

  WiFi.mode(WIFI_STA);


  trc(F("mounting FS..."));

  if (SPIFFS.begin())
  {
    trc(F("mounted file system"));
  }
  else
  {
    trc(F("failed to mount FS -> formating"));
    SPIFFS.format();
    if (SPIFFS.begin())
      trc(F("mounted file system after formating"));
  }
  if (SPIFFS.exists("/config.json"))
  {

    trc(F("reading config file"));
    File configFile = SPIFFS.open("/config.json", "r");
    if (configFile)
    {
      trc(F("opened config file"));
      size_t size = configFile.size();

      std::unique_ptr<char[]> buf(new char[size]);
      configFile.readBytes(buf.get(), size);
      DynamicJsonBuffer jsonBuffer;
      JsonObject &json = jsonBuffer.parseObject(buf.get());
      json.printTo(Serial);
      if (json.success())
      {
        trc(F("\nparsed json"));
        if (json.containsKey("mqtt_server"))
          strcpy(mqtt_server, json["mqtt_server"]);
        if (json.containsKey("mqtt_port"))
          strcpy(mqtt_port, json["mqtt_port"]);
        if (json.containsKey("mqtt_user"))
          strcpy(mqtt_user, json["mqtt_user"]);
        if (json.containsKey("mqtt_pass"))
          strcpy(mqtt_pass, json["mqtt_pass"]);
        if (json.containsKey("mqtt_topic"))
          strcpy(mqtt_topic, json["mqtt_topic"]);
        if (json.containsKey("gateway_name"))
          strcpy(gateway_name, json["gateway_name"]);
      }
      else
      {
        trc(F("failed to load json config"));
      }
    }
  }




  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, parameters_size);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, parameters_size);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqtt_pass, parameters_size);
  WiFiManagerParameter custom_mqtt_topic("topic", "mqtt base topic", mqtt_topic, mqtt_topic_max_size);
  WiFiManagerParameter custom_gateway_name("name", "gateway name", gateway_name, parameters_size * 2);




  wifiManager.setConnectTimeout(WifiManager_TimeOut);

  wifiManager.setConfigPortalTimeout(WifiManager_ConfigPortalTimeOut);


  wifiManager.setSaveConfigCallback(saveConfigCallback);


  #ifdef NetworkAdvancedSetup
  trc(F("Adv wifi cfg"));
  IPAddress gateway_adress(gateway);
  IPAddress subnet_adress(subnet);
  IPAddress ip_adress(ip);
  wifiManager.setSTAStaticIPConfig(ip_adress, gateway_adress,subnet_adress);
  #endif


  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_gateway_name);
  wifiManager.addParameter(&custom_mqtt_topic);


  wifiManager.setMinimumSignalQuality(MinimumWifiSignalQuality);




  if (!wifiManager.autoConnect(WifiManager_ssid, WifiManager_password))
  {
    trc(F("failed to connect and hit timeout"));
    delay(3000);

    #if defined(ESP8266)
    ESP.reset();
    #else
    ESP.restart();
    #endif
    delay(5000);
  }


  trc(F("connected...yeey :)"));


  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  strcpy(mqtt_topic, custom_mqtt_topic.getValue());
  strcpy(gateway_name, custom_gateway_name.getValue());


  if (shouldSaveConfig)
  {
    trc(F("saving config"));
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;
    json["mqtt_topic"] = mqtt_topic;
    json["gateway_name"] = gateway_name;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      trc(F("failed to open config file for writing"));
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();

  }
}
#else
void setup_ethernet()
{
  #ifdef NetworkAdvancedSetup
  trc(F("Adv eth cfg"));
  Ethernet.begin(mac, ip, Dns, gateway, subnet);
  #else
  trc(F("Spl eth cfg"));
  Ethernet.begin(mac, ip);
  #endif
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    trc("Ethernet shield was not found.");
  }
  else
  {
    trc(F("ip: "));
    Serial.println(Ethernet.localIP());
    trc(F("Eth ok"));
  }
}
#endif

#if defined(MDNS_SD) && defined(ESP8266)
void connectMQTTmdns()
{
  if (!MDNS.begin("ESP_MQTT"))
  {
    trc(F("Error setting up MDNS responder!"));
    while (1)
    {
      delay(1000);
    }
  }
  trc(F("Browsing for service MQTT "));
  int n = MDNS.queryService("mqtt", "tcp");
  if (n == 0)
  {
    trc(F("no services found"));
  }
  else
  {
    trc(n);
    trc(F(" service(s) found"));
    for (int i = 0; i < n; ++i)
    {

      trc(i + 1);
      trc(MDNS.hostname(i));
      trc(MDNS.IP(i).toString());
      trc(MDNS.port(i));
    }
    if (n == 1)
    {
      trc(F("One MQTT server found setting parameters"));
      client.setServer(MDNS.IP(0), int(MDNS.port(0)));
    }
    else
    {
      trc(F("Several MQTT servers found, please deactivate mDNS and set your default server"));
    }
  }
}
#endif

void loop()
{
  #ifndef ESPWifiManualSetup
    #if defined(ESP8266) || defined(ESP32)
      checkButton();
    #endif
  #endif

  digitalWrite(led_receive, LOW);
  digitalWrite(led_info, LOW);
  digitalWrite(led_send, LOW);

  unsigned long now = millis();

  #if defined(ESP8266) || defined(ESP32)
  if (WiFi.status() == WL_CONNECTED)
  {
  #else
  if ((Ethernet.hardwareStatus() != EthernetW5100 && Ethernet.linkStatus() == LinkON) || (Ethernet.hardwareStatus() == EthernetW5100))
  {
  #endif
    lastNTWKReconnectAttempt = 0;
    if (client.connected())
    {

      connectedOnce = true;
      lastMQTTReconnectAttempt = 0;

      #ifdef ZmqttDiscovery
      if(!connectedOnce) pubMqttDiscovery();
      #endif
      client.loop();

      #if defined(ESP8266) || defined(ESP32)
      ArduinoOTA.handle();
      #endif

      #ifdef ZsensorBME280
      MeasureTempHumAndPressure();
      #endif
      #ifdef ZsensorHCSR04
      MeasureDistance();
      #endif
      #ifdef ZsensorBH1750
      MeasureLightIntensity();
      #endif
      #ifdef ZsensorTSL2561
      MeasureLightIntensityTSL2561();
      #endif
      #ifdef ZsensorDHT
      MeasureTempAndHum();
      #endif
      #ifdef ZsensorINA226
      MeasureINA226();
      #endif
      #ifdef ZsensorHCSR501
      MeasureHCSR501();
      #endif
      #ifdef ZsensorGPIOInput
      MeasureGPIOInput();
      #endif
      #ifdef ZsensorGPIOKeyCode
      MeasureGPIOKeyCode();
      #endif
      #ifdef ZsensorADC
      MeasureADC();
      #endif
      #ifdef ZgatewayLORA
      LORAtoMQTT();
      #endif
      #ifdef ZgatewayRF
      RFtoMQTT();
      #endif
      #ifdef ZgatewayRF2
      RF2toMQTT();
      #endif
      #ifdef ZgatewayPilight
      PilighttoMQTT();
      #endif
      #ifdef ZgatewayBT
        #ifndef ESP32
        if (BTtoMQTT())
          trc(F("BTtoMQTT OK"));
        #endif
      #endif
      #ifdef ZgatewaySRFB
      SRFBtoMQTT();
      #endif
      #ifdef ZgatewayIR
      IRtoMQTT();
      #endif
      #ifdef Zgateway2G
      if (_2GtoMQTT())
        trc(F("2GtoMQTT OK"));
      #endif
      #ifdef ZgatewayRFM69
      if (RFM69toMQTT())
        trc(F("RFM69toMQTT OK"));
      #endif
      #if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      stateMeasures();
      #endif
      #ifdef ZactuatorFASTLED
      FASTLEDLoop();
      #endif
    }
    else
    {
      if (now - lastMQTTReconnectAttempt > 5000)
      {
        lastMQTTReconnectAttempt = now;
        reconnect();
      }
    }
  }
  else
  {
    if (now - lastNTWKReconnectAttempt > 10000)
    {
      lastNTWKReconnectAttempt = now;
      #if defined(ESP8266) || defined(ESP32)
        trc("wifi disconnected");
        delay(10000);
        #if defined(ESPWifiManualSetup)
        trc(F("restarting ESP"));
          #ifdef ESP32
          ESP.restart();
          #endif
          #ifdef ESP8266
          ESP.reset();
          #endif
        #else
        if (!connectedOnce)
        {
          trc(F("reseting wifi manager"));
          setup_wifimanager(true);
        }
        #endif
      #else
      trc("eth disconnected");
      #endif
    }
  }
}

#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
void stateMeasures()
{
  unsigned long now = millis();
  if (now > (timer_sys_measures + TimeBetweenReadingSYS))
  {
    timer_sys_measures = millis();
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &SYSdata = jsonBuffer.createObject();
    trc(F("Uptime (s)"));
    unsigned long uptime = millis() / 1000;
    trc(uptime);
    SYSdata["uptime"] = uptime;
    #if defined(ESP8266) || defined(ESP32)
      uint32_t freeMem;
      freeMem = ESP.getFreeHeap();
      SYSdata["freeMem"] = freeMem;
      long rssi = WiFi.RSSI();
      SYSdata["rssi"] = rssi;
      String SSID = WiFi.SSID();
      SYSdata["SSID"] = SSID;
      SYSdata["ip"] = ip2CharArray(WiFi.localIP());
      String mac = WiFi.macAddress();
      SYSdata["mac"] = (char *)mac.c_str();
    #else
      SYSdata["ip"] = ip2CharArray(Ethernet.localIP());
    #endif
    String modules = "";
    #ifdef ZgatewayRF
    modules = modules + ZgatewayRF;
    #endif
    #ifdef ZsensorBME280
    modules = modules + ZsensorBME280;
    #endif
    #ifdef ZsensorHCSR04
    modules = modules + ZsensorHCSR04;
    #endif
    #ifdef ZsensorBH1750
    modules = modules + ZsensorBH1750;
    #endif
    #ifdef ZsensorTSL2561
    modules = modules + ZsensorTSL2561;
    #endif
    #ifdef ZsensorDHT
    modules = modules + ZsensorDHT;
    #endif
    #ifdef ZactuatorONOFF
    modules = modules + ZactuatorONOFF;
    #endif
    #ifdef Zgateway2G
    modules = modules + Zgateway2G;
    #endif
    #ifdef ZgatewayIR
    modules = modules + ZgatewayIR;
    #endif
    #ifdef ZgatewayLORA
    modules = modules + ZgatewayLORA;
    #endif
    #ifdef ZgatewayRF2
    modules = modules + ZgatewayRF2;
    #endif
    #ifdef ZgatewayPilight
    modules = modules + ZgatewayPilight;
    #endif
    #ifdef ZgatewaySRFB
    modules = modules + ZgatewaySRFB;
    #endif
    #ifdef ZgatewayBT
    modules = modules + ZgatewayBT;
    #endif
    #ifdef ZgatewayRFM69
    modules = modules + ZgatewayRFM69;
    #endif
    #ifdef ZsensorINA226
    modules = modules + ZsensorINA226;
    #endif
    #ifdef ZsensorHCSR501
    modules = modules + ZsensorHCSR501;
    #endif
    #ifdef ZsensorGPIOInput
    modules = modules + ZsensorGPIOInput;
    #endif
    #ifdef ZsensorGPIOKeyCode
    modules = modules + ZsensorGPIOKeyCode;
    #endif
    #ifdef ZsensorGPIOKeyCode
    modules = modules + ZsensorGPIOKeyCode;
    #endif
    #ifdef ZmqttDiscovery
    modules = modules + ZmqttDiscovery;
    pubMqttDiscovery();
    #endif
    #ifdef ZactuatorFASTLED
    modules = modules + ZactuatorFASTLED;
    #endif

    SYSdata["modules"] = modules;
    trc(SYSdata);
    pub(subjectSYStoMQTT, SYSdata);
  }
}
#endif

void storeValue(unsigned long MQTTvalue)
{
  unsigned long now = millis();

  int o = getMin();
  trc(F("Min ind: "));
  trc(o);

  ReceivedSignal[o][0] = MQTTvalue;
  ReceivedSignal[o][1] = now;
  trc(F("store code :"));
  trc(String(ReceivedSignal[o][0]) + "/" + String(ReceivedSignal[o][1]));
  trc(F("Col: val/timestamp"));
  for (int i = 0; i < array_size; i++)
  {
    trc(String(i) + ":" + String(ReceivedSignal[i][0]) + "/" + String(ReceivedSignal[i][1]));
  }
}

int getMin()
{
  unsigned int minimum = ReceivedSignal[0][1];
  int minindex = 0;
  for (int i = 0; i < array_size; i++)
  {
    if (ReceivedSignal[i][1] < minimum)
    {
      minimum = ReceivedSignal[i][1];
      minindex = i;
    }
  }
  return minindex;
}

bool isAduplicate(unsigned long value)
{
  trc(F("isAdupl?"));

  for (int i = 0; i < array_size; i++)
  {
    if (ReceivedSignal[i][0] == value)
    {
      unsigned long now = millis();
      if (now - ReceivedSignal[i][1] < time_avoid_duplicate)
      {
        trc(F("no pub. dupl"));
        return true;
      }
    }
  }
  return false;
}

void receivingMQTT(char *topicOri, char *datacallback)
{

  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject &jsondata = jsonBuffer.parseObject(datacallback);

  if (strstr(topicOri, subjectMultiGTWKey) != NULL)
  {
    trc(F("Store str"));
    unsigned long data = 0;
    #ifdef jsonPublishing
    if (jsondata.success())
      data = jsondata["value"];
    #endif

    #ifdef simplePublishing
    data = strtoul(datacallback, NULL, 10);
    #endif

    if (data != 0)
    {
      storeValue(data);
      trc(F("JSON str"));
    }
  }

  if (jsondata.success())
  {
    #ifdef ZgatewayPilight
    MQTTtoPilight(topicOri, jsondata);
    #endif
    #ifdef jsonReceiving
      #ifdef ZgatewayLORA
      MQTTtoLORA(topicOri, jsondata);
      #endif
      #ifdef ZgatewayRF
      MQTTtoRF(topicOri, jsondata);
      #endif
      #ifdef ZgatewayRF2
      MQTTtoRF2(topicOri, jsondata);
      #endif
      #ifdef Zgateway2G
      MQTTto2G(topicOri, jsondata);
      #endif
      #ifdef ZgatewaySRFB
      MQTTtoSRFB(topicOri, jsondata);
      #endif
      #ifdef ZgatewayIR
      MQTTtoIR(topicOri, jsondata);
      #endif
      #ifdef ZgatewayRFM69
      MQTTtoRFM69(topicOri, jsondata);
      #endif
      #ifdef ZgatewayBT
      MQTTtoBT(topicOri, jsondata);
      #endif
    #endif
    #ifdef ZactuatorONOFF
    MQTTtoONOFF(topicOri, jsondata);
    #endif
    digitalWrite(led_send, HIGH);

    #ifdef ZactuatorFASTLED
    MQTTtoFASTLEDJSON(topicOri, jsondata);
    #endif
    MQTTtoSYS(topicOri, jsondata);
  }
  else
  {
    #ifdef simpleReceiving
      #ifdef ZgatewayLORA
      MQTTtoLORA(topicOri, datacallback);
      #endif
      #ifdef ZgatewayRF
      MQTTtoRF(topicOri, datacallback);
      #endif
      #ifdef ZgatewayRF315
      MQTTtoRF315(topicOri, datacallback);
      #endif
      #ifdef ZgatewayRF2
      MQTTtoRF2(topicOri, datacallback);
      #endif
      #ifdef Zgateway2G
      MQTTto2G(topicOri, datacallback);
      #endif
      #ifdef ZgatewaySRFB
      MQTTtoSRFB(topicOri, datacallback);
      #endif
      #ifdef ZgatewayIR
      MQTTtoIR(topicOri, datacallback);
      #endif
      #ifdef ZgatewayRFM69
      MQTTtoRFM69(topicOri, datacallback);
      #endif
    #endif
    #ifdef ZactuatorONOFF
    MQTTtoONOFF(topicOri, datacallback);
    #endif
    #ifdef ZactuatorFASTLED
    MQTTtoFASTLED(topicOri, datacallback);
    #endif
    digitalWrite(led_send, HIGH);
  }

  digitalWrite(led_send, HIGH);
}

void MQTTtoSYS(char *topicOri, JsonObject &SYSdata)
{
if (cmpToMainTopic(topicOri,subjectMQTTtoSYSset))
  {
    trc(F("MQTTtoSYS json set"));
    #if defined(ESP8266) || defined(ESP32)
    if (SYSdata.containsKey("cmd"))
    {
      trc(F("Command"));
      const char *cmd = SYSdata["cmd"];
      trc(cmd);
      if (strstr(cmd, restartCmd) != NULL)
      {
      #if defined(ESP8266)
        ESP.reset();
      #else
        ESP.restart();
      #endif
      }
      else if (strstr(cmd, eraseCmd) != NULL)
      {
        setup_wifimanager(true);
      }
      else
      {
        trc(F("wrong command"));
      }
    }
    #endif
  }
}
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorBH1750.ino"
# 39 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorBH1750.ino"
#include "User_config.h"

#ifdef ZsensorBH1750
#include "math.h"
#include "Wire.h"

void setupZsensorBH1750()
{
  Wire.begin();
  Wire.beginTransmission(BH1750_i2c_addr);
  Wire.write(0x10);
  Wire.endTransmission();
  delay(300);
}


void MeasureLightIntensity()
{
  if (millis() > (timebh1750 + TimeBetweenReadingBH1750))
  {
    trc(F("Creating BH1750 buffer"));
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &BH1750data = jsonBuffer.createObject();

    timebh1750 = millis();
    unsigned int i = 0;
    static float persistedll;
    static float persistedlf;
    static float persistedlw;
    unsigned int Lux;
    float ftcd;
    float Wattsm2;


    Wire.requestFrom(BH1750_i2c_addr, 2);
    if (Wire.available() != 2)
    {
      trc(F("Failed to read from LightSensor BH1750!"));
    }
    else
    {
      i = Wire.read();
      i <<= 8;
      i |= Wire.read();


      Lux = i / 1.2;
      ftcd = Lux / 10.764;
      Wattsm2 = Lux / 683.0;


      if (Lux != persistedll || bh1750_always)
      {
        BH1750data.set("lux", (unsigned int)Lux);
      }
      else
      {
        trc(F("Same lux don't send it"));
      }


      if (ftcd != persistedlf || bh1750_always)
      {
        BH1750data.set("ftcd", (unsigned int)ftcd);
      }
      else
      {
        trc(F("Same ftcd don't send it"));
      }


      if (Wattsm2 != persistedlw || bh1750_always)
      {
        BH1750data.set("wattsm2", (unsigned int)Wattsm2);
      }
      else
      {
        trc(F("Same wattsm2 don't send it"));
      }
      if (BH1750data.size() > 0)
        pub(subjectBH1750toMQTT, BH1750data);
    }
    persistedll = Lux;
    persistedlf = ftcd;
    persistedlw = Wattsm2;
  }
}

#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZactuatorFASTLED.ino"
# 23 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZactuatorFASTLED.ino"
#include "User_config.h"

#ifdef ZactuatorFASTLED

#include <FastLED.h>

enum LEDState
{
  OFF,
  FIRE,
  GENERAL
};
LEDState currentLEDState;
long lastUpdate = 0;
long currentUpdate = 0;
CRGB leds[FASTLED_NUM_LEDS];
CRGB ledColorBlink[FASTLED_NUM_LEDS];
bool blinkLED[FASTLED_NUM_LEDS];
const long blinkInterval = 300;
const long fireUpdate = 10;
CRGBPalette16 gPal;

void setupFASTLED()
{
  trc(F("FASTLED_DATA_PIN "));
  trc(String(FASTLED_DATA_PIN));
  trc(F("FASTLED_NUM_LEDS "));
  trc(String(FASTLED_NUM_LEDS));
  trc(F("ZactuatorFASTLED setup done "));
  FastLED.addLeds<FASTLED_TYPE, FASTLED_DATA_PIN>(leds, FASTLED_NUM_LEDS);
}


int animation_step(int duration, int steps)
{
  int currentStep = ((currentUpdate % duration) / ((float)duration)) * steps;
  return currentStep;
}


int animation_step_count(int duration, int steps)
{
  long lastAnimationNumber = lastUpdate / duration;
  long currentAnimationNumber = currentUpdate / duration;
  int lastStep = ((lastUpdate % duration) / ((float)duration)) * steps;
  int currentStep = ((currentUpdate % duration) / ((float)duration)) * steps;

  return currentStep - lastStep + (currentAnimationNumber - lastAnimationNumber) * steps;
}

void FASTLEDLoop()
{

  lastUpdate = currentUpdate;
  currentUpdate = millis();

  if (currentLEDState == GENERAL)
  {

    for (int i = 0; i < FASTLED_NUM_LEDS; i++)
    {

      int count = animation_step_count(blinkInterval, 2);
      int step = animation_step(blinkInterval, 2);

      if (count > 0)
      {
        if (blinkLED[i])
        {

          if (step == 0)
          {
            leds[i] = ledColorBlink[i];
          }
          else
          {
            ledColorBlink[i] = leds[i];
            leds[i] = CRGB::Black;
          }
        }
      }
    }
  }
  else if (currentLEDState == FIRE)
  {
    int count = animation_step_count(fireUpdate, 1);
    if (count > 0)
    {

      Fire2012WithPalette();
    }
  }
  FastLED.show();
}

boolean FASTLEDtoMQTT()
{
  return false;
}
void MQTTtoFASTLEDJSON(char *topicOri, JsonObject &jsonData)
{
  trc(F("MQTTtoFASTLEDJSON: "));
  currentLEDState = GENERAL;
  trc(topicOri);


  if (cmpToMainTopic(topicOri, subjectMQTTtoFASTLEDsetled))
  {
    trc(F("JSON parsed"));
    int ledNr = jsonData["led"];
    trc(F("led"));
    trc(ledNr);
    const char *color = jsonData["hex"];
    trc(F("hex"));

    long number = (long)strtol(color, NULL, 16);
    trc(number);
    bool blink = jsonData["blink"];
    if (ledNr <= FASTLED_NUM_LEDS)
    {
      trc(F("blink"));
      trc(blink);
      blinkLED[ledNr] = blink;
      leds[ledNr] = number;
    }
  }
}
void MQTTtoFASTLED(char *topicOri, char *datacallback)
{
  trc(F("MQTTtoFASTLED: "));
  currentLEDState = GENERAL;
  long number = 0;
  trc(topicOri);
  if (cmpToMainTopic(topicOri, subjectMQTTtoFASTLED))
  {
    number = (long)strtol(&datacallback[1], NULL, 16);
    trc(number);
    for (int i = 0; i < FASTLED_NUM_LEDS; i++)
    {
      leds[i] = number;
    }
    FastLED.show();
  }
  else if (cmpToMainTopic(topicOri, subjectMQTTtoFASTLEDsetbrightness))
  {
    number = (long)strtol(&datacallback[1], NULL, 16);
    trc(number);
    FastLED.setBrightness(number);
    FastLED.show();
  }
  else if (cmpToMainTopic(topicOri, subjectMQTTtoFASTLEDsetanimation))
  {
    String payload = datacallback;
    trc(payload);
    if (strstr(datacallback, "fire") != NULL)
    {
      currentLEDState = FIRE;
      gPal = HeatColors_p;
    }
    else
    {
      currentLEDState = OFF;
    }
  }
  else
  {
    currentLEDState = OFF;
  }
  if (currentLEDState == OFF)
  {
    for (int i = 0; i < FASTLED_NUM_LEDS; i++)
    {
      leds[i] = CRGB::Black;
    }
  }
}
# 231 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZactuatorFASTLED.ino"
#define COOLING 55




#define SPARKING 120
bool gReverseDirection = false;

void Fire2012WithPalette()
{

  static byte heat[FASTLED_NUM_LEDS];


  for (int i = 0; i < FASTLED_NUM_LEDS; i++)
  {
    heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / FASTLED_NUM_LEDS) + 2));
  }


  for (int k = FASTLED_NUM_LEDS - 1; k >= 2; k--)
  {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }


  if (random8() < SPARKING)
  {
    int y = random8(7);
    heat[y] = qadd8(heat[y], random8(160, 255));
  }


  for (int j = 0; j < FASTLED_NUM_LEDS; j++)
  {


    byte colorindex = scale8(heat[j], 240);
    CRGB color = ColorFromPalette(gPal, colorindex);
    int pixelnumber;
    if (gReverseDirection)
    {
      pixelnumber = (FASTLED_NUM_LEDS - 1) - j;
    }
    else
    {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;
  }
}

#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZactuatorONOFF.ino"
# 29 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZactuatorONOFF.ino"
#include "User_config.h"

#ifdef ZactuatorONOFF

#ifdef jsonReceiving
void MQTTtoONOFF(char *topicOri, JsonObject &ONOFFdata)
{
  if (cmpToMainTopic(topicOri, subjectMQTTtoONOFF))
  {
    trc(F("MQTTtoONOFF json data analysis"));
    int boolSWITCHTYPE = ONOFFdata["cmd"] | 99;
    int pin = ONOFFdata["pin"] | ACTUATOR_ONOFF_PIN;
    if (boolSWITCHTYPE != 99)
    {
      trc(F("MQTTtoONOFF boolSWITCHTYPE ok"));
      trc(boolSWITCHTYPE);
      trc(F("pin number"));
      trc(pin);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, boolSWITCHTYPE);

      pub(subjectGTWONOFFtoMQTT, ONOFFdata);
    }
    else
    {
      trc(F("MQTTtoONOFF failed json read"));
    }
  }
}
#endif

#ifdef simpleReceiving
void MQTTtoONOFF(char *topicOri, char *datacallback)
{
  if ((cmpToMainTopic(topicOri, subjectMQTTtoONOFF)))
  {

    trc(F("MQTTtoONOFF"));
    int pin = strtol(datacallback, NULL, 10);
    trc(F("pin number"));
    trc(pin);
    pinMode(pin, OUTPUT);

    bool ON = false;
    if (strstr(topicOri, ONKey) != NULL)
      ON = true;
    if (strstr(topicOri, OFFKey) != NULL)
      ON = false;

    digitalWrite(pin, ON);

    char b = ON;
    pub(subjectGTWONOFFtoMQTT, &b);
  }
}
#endif

#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/Zgateway2G.ino"
# 28 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/Zgateway2G.ino"
#include "User_config.h"

#ifdef Zgateway2G

#include <ArduinoJson.h>
#include <A6lib.h>


A6lib A6l(_2G_TX_PIN, _2G_RX_PIN);

int unreadSMSLocs[50] = {0};
int unreadSMSNum = 0;
SMSmessage sms;

void setup2G()
{

  trc(F("_2G_TX_PIN "));
  trc(_2G_TX_PIN);
  trc(F("_2G_RX_PIN "));
  trc(_2G_RX_PIN);
  setupGSM(false);
  trc(F("Zgateway2G setup done "));
}

void setupGSM(bool deleteSMS)
{
  trc(F("Init 2G module: "));
  trc(_2G_PWR_PIN);
  delay(1000);

  A6l.powerCycle(_2G_PWR_PIN);
  trc(F("waiting for network connection"));
  A6l.blockUntilReady(_2G_MODULE_BAUDRATE);
  trc(F("A6/A7 gsm ready"));
  signalStrengthAnalysis();
  delay(1000);

  if (deleteSMS)
  {
    if (A6l.deleteSMS(1, 4) == A6_OK)
    {
      trc(F("delete SMS OK"));
    }
    else
    {
      trc(F("delete SMS KO"));
    }
  }
}

void signalStrengthAnalysis()
{
  int signalStrength = 0;
  signalStrength = A6l.getSignalStrength();
  trc(F("Signal strength: "));
  trc(signalStrength);
  if (signalStrength < _2G_MIN_SIGNAL || signalStrength > _2G_MAX_SIGNAL)
  {
    trc(F("Signal too low restart the module"));
    setupGSM(false);
  }
}

bool _2GtoMQTT()
{

  unreadSMSNum = A6l.getUnreadSMSLocs(unreadSMSLocs, 512);
  trc(F("Creating SMS  buffer"));
  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject &SMSdata = jsonBuffer.createObject();
  for (int i = 0; i < unreadSMSNum; i++)
  {
    trc(F("New  message at index: "));
    trc(unreadSMSNum);
    sms = A6l.readSMS(unreadSMSLocs[i]);
    SMSdata.set("message", (char *)sms.message.c_str());
    SMSdata.set("date", (char *)sms.date.c_str());
    SMSdata.set("phone", (char *)sms.number.c_str());
    A6l.deleteSMS(unreadSMSLocs[i]);
    trc(F("Adv data 2GtoMQTT"));
    pub(subject2GtoMQTT, SMSdata);
    return true;
  }
  return false;
}
#ifdef simpleReceiving
void MQTTto2G(char *topicOri, char *datacallback)
{

  String data = datacallback;
  String topic = topicOri;

  if (cmpToMainTopic(topicOri, subjectMQTTto2G))
  {
    trc(F("MQTTto2G data analysis"));

    String phone_number = "";
    int pos0 = topic.lastIndexOf(_2GPhoneKey);
    if (pos0 != -1)
    {
      pos0 = pos0 + strlen(_2GPhoneKey);
      phone_number = topic.substring(pos0);
      trc(F("MQTTto2G phone ok"));
      trc(phone_number);
      trc(F("MQTTto2G sms"));
      trc(data);
      if (A6l.sendSMS(phone_number, data) == A6_OK)
      {
        trc(F("SMS OK"));

        pub(subjectGTW2GtoMQTT, "SMS OK");
      }
      else
      {
        trc(F("SMS KO"));

        pub(subjectGTW2GtoMQTT, "SMS KO");
      }
    }
    else
    {
      trc(F("MQTTto2G Fail reading phone number"));
    }
  }
}
#endif

#ifdef jsonReceiving
void MQTTto2G(char *topicOri, JsonObject &SMSdata)
{

  if (cmpToMainTopic(topicOri, subjectMQTTto2G))
  {
    const char *sms = SMSdata["message"];
    const char *phone = SMSdata["phone"];
    trc(F("MQTTto2G json data analysis"));
    if (sms && phone)
    {
      trc(F("MQTTto2G sms & phone ok"));
      trc(sms);
      if (A6l.sendSMS(String(phone), String(sms)) == A6_OK)
      {
        trc(F("SMS OK"));

        pub(subjectGTW2GtoMQTT, "SMS OK");
      }
      else
      {
        trc(F("SMS KO"));
        pub(subjectGTW2GtoMQTT, "SMS KO");
      }
    }
    else
    {
      trc(F("MQTTto2G failed json read"));
    }
  }
}
#endif
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayBT.ino"
# 31 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayBT.ino"
#include "User_config.h"

#ifdef ZgatewayBT

#include <vector>
using namespace std;
vector<BLEdevice> devices;

void setWorBMac(char *mac, bool isWhite)
{
  bool foundMac = false;
  for (vector<BLEdevice>::iterator p = devices.begin(); p != devices.end(); ++p)
  {
    if ((strcmp(p->macAdr, mac) == 0))
    {
      p->isWhtL = isWhite;
      p->isBlkL = !isWhite;
      foundMac = true;
    }
  }
  if (!foundMac)
  {
    BLEdevice device;
    strcpy(device.macAdr, mac);
    device.isDisc = false;
    device.isWhtL = isWhite;
    device.isBlkL = !isWhite;
    devices.push_back(device);
  }
}

bool oneWhite()
{
  for (vector<BLEdevice>::iterator p = devices.begin(); p != devices.end(); ++p)
  {
    if (p->isWhtL)
      return true;
  }
  return false;
}

bool isWhite(char *mac)
{
  for (vector<BLEdevice>::iterator p = devices.begin(); p != devices.end(); ++p)
  {
    if ((strcmp(p->macAdr, mac) == 0))
    {
      return p->isWhtL;
    }
  }
  return false;
}

bool isBlack(char *mac)
{
  for (vector<BLEdevice>::iterator p = devices.begin(); p != devices.end(); ++p)
  {
    if ((strcmp(p->macAdr, mac) == 0))
    {
      return p->isBlkL;
    }
  }
  return false;
}

bool isDiscovered(char *mac)
{
  for (vector<BLEdevice>::iterator p = devices.begin(); p != devices.end(); ++p)
  {
    if ((strcmp(p->macAdr, mac) == 0))
    {
      return p->isDisc;
    }
  }
  return false;
}

void dumpDevices()
{
  for (vector<BLEdevice>::iterator p = devices.begin(); p != devices.end(); ++p)
  {
    trc(p->macAdr);
    trc(p->isDisc);
    trc(p->isWhtL);
    trc(p->isBlkL);
  }
}

void strupp(char *beg)
{
  while (*beg = toupper(*beg))
    ++beg;
}

#ifdef ZmqttDiscovery
void MiFloraDiscovery(char *mac)
{
#define MiFloraparametersCount 4
  trc(F("MiFloraDiscovery"));
  char *MiFlorasensor[MiFloraparametersCount][8] = {
      {"sensor", "MiFlora-lux", mac, "illuminance", "{{ value_json.lux | is_defined }}", "", "", "lu"},
      {"sensor", "MiFlora-tem", mac, "temperature", "{{ value_json.tem | is_defined }}", "", "", "°C"},
      {"sensor", "MiFlora-fer", mac, "", "{{ value_json.fer | is_defined }}", "", "", "µS/cm"},
      {"sensor", "MiFlora-moi", mac, "", "{{ value_json.moi | is_defined }}", "", "", "%"}

  };

  for (int i = 0; i < MiFloraparametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(MiFlorasensor[i][1]);
    String discovery_topic = String(subjectBTtoMQTT) + "/" + String(mac);
    String unique_id = String(mac) + "-" + MiFlorasensor[i][1];
    createDiscovery(MiFlorasensor[i][0],
                    (char *)discovery_topic.c_str(), MiFlorasensor[i][1], (char *)unique_id.c_str(),
                    will_Topic, MiFlorasensor[i][3], MiFlorasensor[i][4],
                    MiFlorasensor[i][5], MiFlorasensor[i][6], MiFlorasensor[i][7],
                    0, "", "", false, "");
  }
  BLEdevice device;
  strcpy(device.macAdr, mac);
  device.isDisc = true;
  device.isWhtL = false;
  device.isBlkL = false;
  devices.push_back(device);
}

void VegTrugDiscovery(char *mac)
{
#define VegTrugparametersCount 4
  trc(F("VegTrugDiscovery"));
  char *VegTrugsensor[VegTrugparametersCount][8] = {
      {"sensor", "VegTrug-lux", mac, "illuminance", "{{ value_json.lux | is_defined }}", "", "", "lu"},
      {"sensor", "VegTrug-tem", mac, "temperature", "{{ value_json.tem | is_defined }}", "", "", "°C"},
      {"sensor", "VegTrug-fer", mac, "", "{{ value_json.fer | is_defined }}", "", "", "µS/cm"},
      {"sensor", "VegTrug-moi", mac, "", "{{ value_json.moi | is_defined }}", "", "", "%"}

  };

  for (int i = 0; i < VegTrugparametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(VegTrugsensor[i][1]);
    String discovery_topic = String(subjectBTtoMQTT) + "/" + String(mac);
    String unique_id = String(mac) + "-" + VegTrugsensor[i][1];
    createDiscovery(VegTrugsensor[i][0],
                    (char *)discovery_topic.c_str(), VegTrugsensor[i][1], (char *)unique_id.c_str(),
                    will_Topic, VegTrugsensor[i][3], VegTrugsensor[i][4],
                    VegTrugsensor[i][5], VegTrugsensor[i][6], VegTrugsensor[i][7],
                    0, "", "", false, "");
  }
  BLEdevice device;
  strcpy(device.macAdr, mac);
  device.isDisc = true;
  device.isWhtL = false;
  device.isBlkL = false;
  devices.push_back(device);
}

void MiJiaDiscovery(char *mac)
{
#define MiJiaparametersCount 3
  trc(F("MiJiaDiscovery"));
  char *MiJiasensor[MiJiaparametersCount][8] = {
      {"sensor", "MiJia-batt", mac, "battery", "{{ value_json.batt | is_defined }}", "", "", "%"},
      {"sensor", "MiJia-tem", mac, "temperature", "{{ value_json.tem | is_defined }}", "", "", "°C"},
      {"sensor", "MiJia-hum", mac, "humidity", "{{ value_json.hum | is_defined }}", "", "", "%"}

  };

  for (int i = 0; i < MiJiaparametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(MiJiasensor[i][1]);
    String discovery_topic = String(subjectBTtoMQTT) + "/" + String(mac);
    String unique_id = String(mac) + "-" + MiJiasensor[i][1];
    createDiscovery(MiJiasensor[i][0],
                    (char *)discovery_topic.c_str(), MiJiasensor[i][1], (char *)unique_id.c_str(),
                    will_Topic, MiJiasensor[i][3], MiJiasensor[i][4],
                    MiJiasensor[i][5], MiJiasensor[i][6], MiJiasensor[i][7],
                    0, "", "", false, "");
  }
  BLEdevice device;
  strcpy(device.macAdr, mac);
  device.isDisc = true;
  device.isWhtL = false;
  device.isBlkL = false;
  devices.push_back(device);
}

void LYWSD02Discovery(char *mac)
{
#define LYWSD02parametersCount 3
  trc(F("LYWSD02Discovery"));
  char *LYWSD02sensor[LYWSD02parametersCount][8] = {
      {"sensor", "LYWSD02-batt", mac, "battery", "{{ value_json.batt | is_defined }}", "", "", "V"},
      {"sensor", "LYWSD02-tem", mac, "temperature", "{{ value_json.tem | is_defined }}", "", "", "°C"},
      {"sensor", "LYWSD02-hum", mac, "humidity", "{{ value_json.hum | is_defined }}", "", "", "%"}

  };

  for (int i = 0; i < LYWSD02parametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(LYWSD02sensor[i][1]);
    String discovery_topic = String(subjectBTtoMQTT) + "/" + String(mac);
    String unique_id = String(mac) + "-" + LYWSD02sensor[i][1];
    createDiscovery(LYWSD02sensor[i][0],
                    (char *)discovery_topic.c_str(), LYWSD02sensor[i][1], (char *)unique_id.c_str(),
                    will_Topic, LYWSD02sensor[i][3], LYWSD02sensor[i][4],
                    LYWSD02sensor[i][5], LYWSD02sensor[i][6], LYWSD02sensor[i][7],
                    0, "", "", false, "");
  }
  BLEdevice device;
  strcpy(device.macAdr, mac);
  device.isDisc = true;
  device.isWhtL = false;
  device.isBlkL = false;
  devices.push_back(device);
}

void CLEARGRASSTRHDiscovery(char *mac)
{
#define CLEARGRASSTRHparametersCount 3
  trc(F("CLEARGRASSTRHDiscovery"));
  char *CLEARGRASSTRHsensor[CLEARGRASSTRHparametersCount][8] = {
      {"sensor", "CLEARGRASSTRH-batt", mac, "battery", "{{ value_json.batt | is_defined }}", "", "", "V"},
      {"sensor", "CLEARGRASSTRH-tem", mac, "temperature", "{{ value_json.tem | is_defined }}", "", "", "°C"},
      {"sensor", "CLEARGRASSTRH-hum", mac, "humidity", "{{ value_json.hum | is_defined }}", "", "", "%"}

  };

  for (int i = 0; i < CLEARGRASSTRHparametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(CLEARGRASSTRHsensor[i][1]);
    String discovery_topic = String(subjectBTtoMQTT) + "/" + String(mac);
    String unique_id = String(mac) + "-" + CLEARGRASSTRHsensor[i][1];
    createDiscovery(CLEARGRASSTRHsensor[i][0],
                    (char *)discovery_topic.c_str(), CLEARGRASSTRHsensor[i][1], (char *)unique_id.c_str(),
                    will_Topic, CLEARGRASSTRHsensor[i][3], CLEARGRASSTRHsensor[i][4],
                    CLEARGRASSTRHsensor[i][5], CLEARGRASSTRHsensor[i][6], CLEARGRASSTRHsensor[i][7],
                    0, "", "", false, "");
  }
  BLEdevice device;
  strcpy(device.macAdr, mac);
  device.isDisc = true;
  device.isWhtL = false;
  device.isBlkL = false;
  devices.push_back(device);
}

void CLEARGRASSTRHKPADiscovery(char *mac)
{
#define CLEARGRASSTRHKPAparametersCount 3
  trc(F("CLEARGRASSTRHKPADiscovery"));
  char *CLEARGRASSTRHKPAsensor[CLEARGRASSTRHKPAparametersCount][8] = {
      {"sensor", "CLEARGRASSTRHKPA-pres", mac, "pressure", "{{ value_json.pres | is_defined }}", "", "", "kPa"},
      {"sensor", "CLEARGRASSTRHKPA-tem", mac, "temperature", "{{ value_json.tem | is_defined }}", "", "", "°C"},
      {"sensor", "CLEARGRASSTRHKPA-hum", mac, "humidity", "{{ value_json.hum | is_defined }}", "", "", "%"}

  };

  for (int i = 0; i < CLEARGRASSTRHKPAparametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(CLEARGRASSTRHKPAsensor[i][1]);
    String discovery_topic = String(subjectBTtoMQTT) + "/" + String(mac);
    String unique_id = String(mac) + "-" + CLEARGRASSTRHKPAsensor[i][1];
    createDiscovery(CLEARGRASSTRHKPAsensor[i][0],
                    (char *)discovery_topic.c_str(), CLEARGRASSTRHKPAsensor[i][1], (char *)unique_id.c_str(),
                    will_Topic, CLEARGRASSTRHKPAsensor[i][3], CLEARGRASSTRHKPAsensor[i][4],
                    CLEARGRASSTRHKPAsensor[i][5], CLEARGRASSTRHKPAsensor[i][6], CLEARGRASSTRHKPAsensor[i][7],
                    0, "", "", false, "");
  }
  BLEdevice device;
  strcpy(device.macAdr, mac);
  device.isDisc = true;
  device.isWhtL = false;
  device.isBlkL = false;
  devices.push_back(device);
}

void MiScaleDiscovery(char *mac)
{
#define MiScaleparametersCount 1
  trc(F("MiScaleDiscovery"));
  char *MiScalesensor[MiScaleparametersCount][8] = {
      {"sensor", "MiScale-weight", mac, "weight", "{{ value_json.weight | is_defined }}", "", "", "kg"},

  };

  for (int i = 0; i < MiScaleparametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(MiScalesensor[i][1]);
    String discovery_topic = String(subjectBTtoMQTT) + "/" + String(mac);
    String unique_id = String(mac) + "-" + MiScalesensor[i][1];
    createDiscovery(MiScalesensor[i][0],
                    (char *)discovery_topic.c_str(), MiScalesensor[i][1], (char *)unique_id.c_str(),
                    will_Topic, MiScalesensor[i][3], MiScalesensor[i][4],
                    MiScalesensor[i][5], MiScalesensor[i][6], MiScalesensor[i][7],
                    0, "", "", false, "");
  }
  BLEdevice device;
  strcpy(device.macAdr, mac);
  device.isDisc = true;
  device.isWhtL = false;
  device.isBlkL = false;
  devices.push_back(device);
}

void MiLampDiscovery(char *mac)
{
#define MiLampparametersCount 1
  trc(F("MiLampDiscovery"));
  char *MiLampsensor[MiLampparametersCount][8] = {
      {"sensor", "MiLamp-presence", mac, "presence", "{{ value_json.presence}}", "", "", "d"},

  };

  for (int i = 0; i < MiLampparametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(MiLampsensor[i][1]);
    String discovery_topic = String(subjectBTtoMQTT) + "/" + String(mac);
    String unique_id = String(mac) + "-" + MiLampsensor[i][1];
    createDiscovery(MiLampsensor[i][0],
                    (char *)discovery_topic.c_str(), MiLampsensor[i][1], (char *)unique_id.c_str(),
                    will_Topic, MiLampsensor[i][3], MiLampsensor[i][4],
                    MiLampsensor[i][5], MiLampsensor[i][6], MiLampsensor[i][7],
                    0, "", "", false, "");
  }
  BLEdevice device;
  strcpy(device.macAdr, mac);
  device.isDisc = true;
  device.isWhtL = false;
  device.isBlkL = false;
  devices.push_back(device);
}

void MiBandDiscovery(char *mac)
{
#define MiBandparametersCount 1
  trc(F("MiBandDiscovery"));
  char *MiBandsensor[MiBandparametersCount][8] = {
      {"sensor", "MiBand-steps", mac, "steps", "{{ value_json.steps | is_defined }}", "", "", "nb"},

  };

  for (int i = 0; i < MiBandparametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(MiBandsensor[i][1]);
    String discovery_topic = String(subjectBTtoMQTT) + "/" + String(mac);
    String unique_id = String(mac) + "-" + MiBandsensor[i][1];
    createDiscovery(MiBandsensor[i][0],
                    (char *)discovery_topic.c_str(), MiBandsensor[i][1], (char *)unique_id.c_str(),
                    will_Topic, MiBandsensor[i][3], MiBandsensor[i][4],
                    MiBandsensor[i][5], MiBandsensor[i][6], MiBandsensor[i][7],
                    0, "", "", false, "");
  }
  BLEdevice device;
  strcpy(device.macAdr, mac);
  device.isDisc = true;
  device.isWhtL = false;
  device.isBlkL = false;
  devices.push_back(device);
}

#endif

#ifdef ESP32






#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"


static int taskCore = 0;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    trc(F("Creating BLE buffer"));
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &BLEdata = jsonBuffer.createObject();
    String mac_adress = advertisedDevice.getAddress().toString().c_str();
    BLEdata.set("id", (char *)mac_adress.c_str());
    mac_adress.replace(":", "");
    mac_adress.toUpperCase();
    String mactopic = subjectBTtoMQTT + String("/") + mac_adress;
    char mac[mac_adress.length() + 1];
    mac_adress.toCharArray(mac, mac_adress.length() + 1);
    trc("device detected");
    trc(mac);
    if ((!oneWhite() || isWhite(mac)) && !isBlack(mac))
    {
      if (advertisedDevice.haveName())
        BLEdata.set("name", (char *)advertisedDevice.getName().c_str());
      if (advertisedDevice.haveManufacturerData())
        BLEdata.set("manufacturerdata", (char *)advertisedDevice.getManufacturerData().c_str());
      if (advertisedDevice.haveRSSI())
        BLEdata.set("rssi", (int)advertisedDevice.getRSSI());
      if (advertisedDevice.haveTXPower())
        BLEdata.set("txpower", (int8_t)advertisedDevice.getTXPower());
      #ifdef subjectHomePresence
      if (advertisedDevice.haveRSSI())
        haRoomPresence(BLEdata);
      #endif
      if (advertisedDevice.haveServiceData())
      {
        trc(F("Get services data :"));
        int serviceDataCount = advertisedDevice.getServiceDataCount();
        trc(serviceDataCount);
        for (int j = 0; j < serviceDataCount; j++)
        {
          std::string serviceData = advertisedDevice.getServiceData(j);
          int serviceDataLength = serviceData.length();
          String returnedString = "";
          for (int i = 0; i < serviceDataLength; i++)
          {
            int a = serviceData[i];
            if (a < 16)
            {
              returnedString = returnedString + "0";
            }
            returnedString = returnedString + String(a, HEX);
          }
          char service_data[returnedString.length() + 1];
          returnedString.toCharArray(service_data, returnedString.length() + 1);
          service_data[returnedString.length()] = '\0';
          #ifdef pubBLEServiceData
          BLEdata.set("servicedata", service_data);
          #endif
          BLEdata.set("servicedatauuid", (char *)advertisedDevice.getServiceDataUUID(j).toString().c_str());
          if (abs((int)BLEdata["rssi"] | 0) < abs(Minrssi))
          {
            pub((char *)mactopic.c_str(), BLEdata);
            if (strstr(BLEdata["servicedatauuid"].as<char *>(), "fe95") != NULL)
            {
              trc("Processing BLE device data");
              int pos = -1;
              pos = strpos(service_data, "209800");
              if (pos != -1)
              {
                trc(F("mi flora data reading"));

                #ifdef ZmqttDiscovery
                if (!isDiscovered(mac))
                  MiFloraDiscovery(mac);
                #endif
                process_sensors(pos - 24, service_data, mac);
              }
              pos = -1;
              pos = strpos(service_data, "20bc03");
              if (pos != -1)
              {
                trc(F("vegtrug data reading"));

                #ifdef ZmqttDiscovery
                if (!isDiscovered(mac))
                  VegTrugDiscovery(mac);
                #endif
                process_sensors(pos - 24, service_data, mac);
              }
              pos = -1;
              pos = strpos(service_data, "20aa01");
              if (pos != -1)
              {
                trc(F("mi jia data reading"));
                #ifdef ZmqttDiscovery
                if (!isDiscovered(mac))
                  MiJiaDiscovery(mac);
                #endif
                process_sensors(pos - 26, service_data, mac);
              }
              pos = -1;
              pos = strpos(service_data, "205b04");
              if (pos != -1)
              {
                trc(F("LYWSD02 data reading"));

                #ifdef ZmqttDiscovery
                if (!isDiscovered(mac))
                  LYWSD02Discovery(mac);
                #endif
                process_sensors(pos - 24, service_data, mac);
              }
              pos = -1;
              pos = strpos(service_data, "304703");
              if (pos != -1)
              {
                trc(F("ClearGrass T RH data reading method 1"));

                #ifdef ZmqttDiscovery
                if (!isDiscovered(mac))
                  CLEARGRASSTRHDiscovery(mac);
                #endif
                process_sensors(pos - 26, service_data, mac);
              }
              pos = -1;
              pos = strpos(service_data, "4030dd");
              if (pos != -1)
              {
                trc(F("Mi Lamp data reading"));

                #ifdef ZmqttDiscovery
                if (!isDiscovered(mac))
                  MiLampDiscovery(mac);
                #endif
                process_milamp(service_data, mac);
              }
            }
            if (strstr(BLEdata["servicedatauuid"].as<char *>(), "181d") != NULL)
            {
              trc(F("Mi Scale V1 data reading"));

              #ifdef ZmqttDiscovery
              if (!isDiscovered(mac))
                MiScaleDiscovery(mac);
              #endif
              process_scale_v1(service_data, mac);
            }
            if (strstr(BLEdata["servicedatauuid"].as<char *>(), "181b") != NULL)
            {
              trc(F("Mi Scale V2 data reading"));

              #ifdef ZmqttDiscovery
              if (!isDiscovered(mac))
                MiScaleDiscovery(mac);
              #endif
              process_scale_v2(service_data, mac);
            }
            if (strstr(BLEdata["servicedatauuid"].as<char *>(), "fee0") != NULL)
            {
              trc(F("Mi Band data reading"));

              #ifdef ZmqttDiscovery
              if (!isDiscovered(mac))
                MiBandDiscovery(mac);
              #endif
              process_miband(service_data, mac);
            }
            if (strstr(BLEdata["servicedata"].as<char *>(), "08094c") != NULL)
            {
              trc(F("Clear grass data with air pressure reading"));

              #ifdef ZmqttDiscovery
              if (!isDiscovered(mac))
                CLEARGRASSTRHKPADiscovery(mac);
              #endif
              process_cleargrass_air(service_data, mac);
            }
            if (strstr(BLEdata["servicedata"].as<char *>(), "080774") != NULL)
            {
              trc(F("Clear grass data reading method 2"));


              process_cleargrass(service_data, mac);
            }
          }
          else
          {
            trc("Low rssi, device filtered");
          }
        }
      }
      else
      {
        if (abs((int)BLEdata["rssi"] | 0) < abs(Minrssi))
        {
          pub((char *)mactopic.c_str(), BLEdata);
        }
        else
        {
          trc("Low rssi, device filtered");
        }
      }
    }
    else
    {
      trc(F("Filtered mac device"));
    }
  }
};

void BLEscan()
{

  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed = 1;
  TIMERG0.wdt_wprotect = 0;
  trc(F("Scan begin"));
  BLEDevice::init("");
  BLEScan *pBLEScan = BLEDevice::getScan();
  MyAdvertisedDeviceCallbacks myCallbacks;
  pBLEScan->setAdvertisedDeviceCallbacks(&myCallbacks);
  pBLEScan->setActiveScan(true);
  BLEScanResults foundDevices = pBLEScan->start(Scan_duration);
  trc(F("Scan end, deinit controller"));
  esp_bt_controller_deinit();
}

void coreTask(void *pvParameters)
{

  String taskMessage = "BT Task running on core ";
  taskMessage = taskMessage + xPortGetCoreID();

  while (true)
  {
    trc(taskMessage);
    delay(BLEinterval);
    if (client.state() == 0)
    {
      BLEscan();
    }
    else
    {
      trc("MQTT client disconnected no BLE scan");
      delay(1000);
    }
  }
}

void setupBT()
{
  BLEinterval = TimeBtw_Read;
  Minrssi = MinimumRSSI;
  trc(F("BLEinterval btw scans"));
  trc(BLEinterval);
  trc(F("Minrssi"));
  trc(Minrssi);

  xTaskCreatePinnedToCore(
      coreTask,
      "coreTask",
      10000,
      NULL,
      1,
      NULL,
      taskCore);
  trc(F("ZgatewayBT multicore ESP32 setup done "));
}

bool BTtoMQTT()
{
  BLEscan();
}
#else

#include <SoftwareSerial.h>

#define STRING_MSG "OK+DISC:"
#define QUESTION_MSG "AT+DISA?"
#define RESPONSE_MSG "OK+DISIS"
#define RESP_END_MSG "OK+DISCE"
#define SETUP_MSG "OK+RESET"

SoftwareSerial softserial(BT_RX, BT_TX);

String returnedString = "";
unsigned long timebt = 0;


struct decompose d[6] = {{"mac", 16, 12, true}, {"typ", 28, 2, false}, {"rsi", 30, 2, false}, {"rdl", 32, 2, false}, {"sty", 44, 4, true}, {"rda", 34, 60, false}};

void setupBT()
{
  BLEinterval = TimeBtw_Read;
  Minrssi = MinimumRSSI;
  trc(F("BLEinterval btw scans"));
  trc(BLEinterval);
  trc(F("Minrssi"));
  trc(Minrssi);
  softserial.begin(9600);
  softserial.print(F("AT+ROLE1"));
  delay(100);
  softserial.print(F("AT+IMME1"));
  delay(100);
  softserial.print(F("AT+RESET"));
  delay(100);
  #ifdef HM_BLUE_LED_STOP
  softserial.print(F("AT+PIO11"));
  #endif
  delay(100);
  trc(F("ZgatewayBT HM1X setup done "));
}

bool BTtoMQTT()
{


  while (softserial.available() > 0)
  {
    int a = softserial.read();
    if (a < 16)
    {
      returnedString = returnedString + "0";
    }
    returnedString = returnedString + String(a, HEX);
  }

  if (millis() > (timebt + BLEinterval))
  {
    timebt = millis();
    #if defined(ESP8266)
    yield();
    #endif
    if (returnedString != "")
    {
      size_t pos = 0;
      while ((pos = returnedString.lastIndexOf(BLEdelimiter)) != -1)
      {
        #if defined(ESP8266)
        yield();
        #endif
        String token = returnedString.substring(pos);
        returnedString.remove(pos, returnedString.length());
        char token_char[token.length() + 1];
        token.toCharArray(token_char, token.length() + 1);
        trc(token);
        if (token.length() > 60)
        {
          for (int i = 0; i < 6; i++)
          {
            extract_char(token_char, d[i].extract, d[i].start, d[i].len, d[i].reverse, false);
            if (i == 3)
              d[5].len = (int)strtol(d[i].extract, NULL, 16) * 2;
          }

          if ((strlen(d[0].extract)) == 12)
          {
            trc(F("Creating BLE buffer"));
            StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
            JsonObject &BLEdata = jsonBuffer.createObject();
            strupp(d[0].extract);
            if (isBlack(d[0].extract))
              return false;
            if (oneWhite() && !isWhite(d[0].extract))
              return false;
            #ifdef subjectHomePresence
            String HomePresenceId;
            for (int i = 0; i < 12; i++)
            {
              HomePresenceId += String(d[0].extract[i]);
              if (((i - 1) % 2 == 0) && (i != 11))
                HomePresenceId += ":";
            }
            trc(F("HomePresenceId"));
            trc(HomePresenceId);
            BLEdata.set("id", (char *)HomePresenceId.c_str());
            #endif
            String topic = subjectBTtoMQTT + String("/") + String(d[0].extract);
            int rssi = (int)strtol(d[2].extract, NULL, 16) - 256;
            BLEdata.set("rssi", (int)rssi);
            #ifdef subjectHomePresence
            haRoomPresence(BLEdata);
            #endif
            String service_data(d[5].extract);
            service_data = service_data.substring(14);
            #ifdef pubBLEServiceData
            BLEdata.set("servicedata", (char *)service_data.c_str());
            #endif
            if (abs((int)BLEdata["rssi"] | 0) < abs(Minrssi))
            {
              pub((char *)topic.c_str(), BLEdata);
              int pos = -1;
              pos = strpos(d[5].extract, "209800");
              if (pos != -1)
              {
                trc("mi flora data reading");
                #ifdef ZmqttDiscovery
                if (!isDiscovered(d[0].extract))
                  MiFloraDiscovery(d[0].extract);
                #endif
                bool result = process_sensors(pos - 38, (char *)service_data.c_str(), d[0].extract);
              }
              pos = -1;
              pos = strpos(d[5].extract, "20aa01");

              if (pos != -1)
              {
                trc("mi jia data reading");
                #ifdef ZmqttDiscovery
                if (!isDiscovered(d[0].extract))
                  MiJiaDiscovery(d[0].extract);
                #endif
                bool result = process_sensors(pos - 40, (char *)service_data.c_str(), d[0].extract);
              }
              pos = -1;
              pos = strpos(d[5].extract, "205b04");

              if (pos != -1)
              {
                trc("LYWSD02 data reading");
                #ifdef ZmqttDiscovery
                if (!isDiscovered(d[0].extract))
                  LYWSD02Discovery(d[0].extract);
                #endif
                bool result = process_sensors(pos - 38, (char *)service_data.c_str(), d[0].extract);
              }
              pos = -1;
              pos = strpos(d[5].extract, "304703");
              if (pos != -1)
              {
                trc("CLEARGRASSTRH data reading method 1");
                #ifdef ZmqttDiscovery
                if (!isDiscovered(d[0].extract))
                  CLEARGRASSTRHDiscovery(d[0].extract);
                #endif
                bool result = process_sensors(pos - 40, (char *)service_data.c_str(), d[0].extract);
              }
              pos = -1;
              pos = strpos(d[5].extract, "e30706");
              if (pos != -1)
              {
                trc("Mi Scale data reading");

                #ifdef ZmqttDiscovery
                if (!isDiscovered(d[0].extract))
                  MiScaleDiscovery(d[0].extract);
                #endif
                bool result = process_scale_v1((char *)service_data.c_str(), d[0].extract);
              }
              pos = -1;
              pos = strpos(d[5].extract, "4030dd");
              if (pos != -1)
              {
                trc(F("Mi Lamp data reading"));

                #ifdef ZmqttDiscovery
                if (!isDiscovered(d[0].extract))
                  MiLampDiscovery(d[0].extract);
                #endif
                process_milamp((char *)service_data.c_str(), d[0].extract);
              }
              pos = -1;
              pos = strpos(d[5].extract, "08094c");
              if (pos != -1)
              {
                trc(F("Clear grass data with air pressure reading"));

                #ifdef ZmqttDiscovery
                if (!isDiscovered(d[0].extract))
                  CLEARGRASSTRHKPADiscovery(d[0].extract);
                #endif
                process_cleargrass_air((char *)service_data.c_str(), d[0].extract);
              }
              pos = -1;
              pos = strpos(d[5].extract, "080774");
              if (pos != -1)
              {
                trc(F("Clear grass data reading method 2"));


                process_cleargrass((char *)service_data.c_str(), d[0].extract);
              }
              return true;
            }
            else
            {
              trc("Low rssi, device filtered");
            }
          }
        }
      }
      returnedString = "";
      return false;
    }
    softserial.print(F(QUESTION_MSG));
    return false;
  }
  else
  {
    return false;
  }
}
#endif

double value_from_service_data(char *service_data, int offset, int data_length)
{
  char rev_data[data_length + 1];
  char data[data_length + 1];
  memcpy(rev_data, &service_data[offset], data_length);
  rev_data[data_length] = '\0';


  revert_hex_data(rev_data, data, data_length + 1);
  double value = strtol(data, NULL, 16);
  if (value > 65000 && data_length <= 4)
    value = value - 65535;
  trc(value);
  return value;
}

bool process_sensors(int offset, char *rest_data, char *mac_adress)
{

  trc(F("Creating BLE buffer"));
  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject &BLEdata = jsonBuffer.createObject();
  trc("rest_data");
  trc(rest_data);
  int data_length = 0;
  trc("data_length");
  trc(rest_data[51 + offset]);
  switch (rest_data[51 + offset])
  {
  case '1':
  case '2':
  case '3':
  case '4':
    data_length = ((rest_data[51 + offset] - '0') * 2);
    trc("valid data_length");
    break;
  default:
    trc("invalid data_length");
    return false;
  }

  double value = 9999;
  value = value_from_service_data(rest_data, 52 + offset, data_length);




  switch (rest_data[47 + offset])
  {
  case '9':
    BLEdata.set("fer", (double)value);
    break;
  case '4':
    BLEdata.set("tem", (double)value / 10);
    break;
  case '6':
    BLEdata.set("hum", (double)value / 10);
    break;
  case '7':
    BLEdata.set("lux", (double)value);
    break;
  case '8':
    BLEdata.set("moi", (double)value);
    break;

  case 'a':
    BLEdata.set("batt", (double)value);
    break;

  case 'd':

    value = value_from_service_data(rest_data, 52 + offset, 4);
    BLEdata.set("tem", (double)value / 10);

    value = value_from_service_data(rest_data, 56 + offset, 4);
    BLEdata.set("hum", (double)value / 10);
    break;
  default:
    trc("can't read values");
    return false;
  }
  String mactopic(mac_adress);
  mactopic = subjectBTtoMQTT + String("/") + mactopic;
  pub((char *)mactopic.c_str(), BLEdata);
  return true;
}

bool process_scale_v1(char *rest_data, char *mac_adress)
{

  trc("rest_data");
  trc(rest_data);

  trc(F("Creating BLE buffer"));
  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject &BLEdata = jsonBuffer.createObject();

  double weight = value_from_service_data(rest_data, 2, 4) / 200;


  BLEdata.set("weight", (double)weight);


  String mactopic(mac_adress);
  mactopic = subjectBTtoMQTT + String("/") + mactopic;
  pub((char *)mactopic.c_str(), BLEdata);
  return true;
}

bool process_scale_v2(char *rest_data, char *mac_adress)
{

  trc("rest_data");
  trc(rest_data);

  trc(F("Creating BLE buffer"));
  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject &BLEdata = jsonBuffer.createObject();

  double weight = value_from_service_data(rest_data, 22, 4) / 200;
  double impedance = value_from_service_data(rest_data, 18, 4);


  BLEdata.set("weight", (double)weight);
  BLEdata.set("impedance", (double)impedance);


  String mactopic(mac_adress);
  mactopic = subjectBTtoMQTT + String("/") + mactopic;
  pub((char *)mactopic.c_str(), BLEdata);
  return true;
}

bool process_miband(char *rest_data, char *mac_adress)
{

  trc("rest_data");
  trc(rest_data);

  trc(F("Creating BLE buffer"));
  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject &BLEdata = jsonBuffer.createObject();

  double steps = value_from_service_data(rest_data, 0, 4);


  BLEdata.set("steps", (double)steps);


  String mactopic(mac_adress);
  mactopic = subjectBTtoMQTT + String("/") + mactopic;
  pub((char *)mactopic.c_str(), BLEdata);
  return true;
}

bool process_milamp(char *rest_data, char *mac_adress)
{

  trc("rest_data");
  trc(rest_data);

  trc(F("Creating BLE buffer"));
  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject &BLEdata = jsonBuffer.createObject();

  long darkness = value_from_service_data(rest_data, 8, 2);


  BLEdata.set("presence", (bool)"true");
  BLEdata.set("darkness", (long)darkness);


  String mactopic(mac_adress);
  mactopic = subjectBTtoMQTT + String("/") + mactopic;
  pub((char *)mactopic.c_str(), BLEdata);
  return true;
}

bool process_cleargrass_air(char *rest_data, char *mac_adress)
{


  trc("rest_data");
  trc(rest_data);

  trc(F("Creating BLE buffer"));
  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject &BLEdata = jsonBuffer.createObject();

  double value = 9999;

  value = value_from_service_data(rest_data, 20, 4);
  BLEdata.set("tem", (double)value / 10);

  value = value_from_service_data(rest_data, 24, 4);
  BLEdata.set("hum", (double)value / 10);

  value = value_from_service_data(rest_data, 32, 4);
  BLEdata.set("pres", (double)value / 100);

  String mactopic(mac_adress);
  mactopic = subjectBTtoMQTT + String("/") + mactopic;
  pub((char *)mactopic.c_str(), BLEdata);
  return true;
}

bool process_cleargrass(char *rest_data, char *mac_adress)
{

  trc("rest_data");
  trc(rest_data);

  trc(F("Creating BLE buffer"));
  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
  JsonObject &BLEdata = jsonBuffer.createObject();

  double value = 9999;

  value = value_from_service_data(rest_data, 20, 4);
  BLEdata.set("tem", (double)value / 10);

  value = value_from_service_data(rest_data, 24, 4);
  BLEdata.set("hum", (double)value / 10);

  String mactopic(mac_adress);
  mactopic = subjectBTtoMQTT + String("/") + mactopic;
  pub((char *)mactopic.c_str(), BLEdata);
  return true;
}

#ifdef subjectHomePresence
void haRoomPresence(JsonObject &HomePresence)
{
  int BLErssi = HomePresence["rssi"];
  trc(F("BLErssi"));
  trc(BLErssi);
  int txPower = HomePresence["txpower"] | 0;
  if (txPower >= 0)
    txPower = -59;
  trc(F("txPower"));
  trc(txPower);
  double ratio = BLErssi * 1.0 / txPower;
  double distance;
  if (ratio < 1.0)
  {
    distance = pow(ratio, 10);
  }
  else
  {
    distance = (0.89976) * pow(ratio, 7.7095) + 0.111;
  }
  HomePresence["distance"] = distance;
  trc(F("BLE DISTANCE :"));
  trc(distance);
  String topic = String(Base_Topic) + "home_presence/" + String(gateway_name);
  pub_custom_topic((char *)topic.c_str(), HomePresence, false);
}
#endif

void MQTTtoBT(char *topicOri, JsonObject &BTdata)
{
  if (cmpToMainTopic(topicOri, subjectMQTTtoBTset))
  {
    trc(F("MQTTtoBT json set"));


    int WLsize = BTdata["white-list"].size();
    if (WLsize > 0)
    {
      trc(F("WL set"));
      for (int i = 0; i < WLsize; i++)
      {
        const char *whiteMac = BTdata["white-list"][i];
        setWorBMac((char *)whiteMac, true);
      }
    }
    int BLsize = BTdata["black-list"].size();
    if (BLsize > 0)
    {
      trc(F("BL set"));
      for (int i = 0; i < BLsize; i++)
      {
        const char *blackMac = BTdata["black-list"][i];
        setWorBMac((char *)blackMac, false);
      }
    }
    if (BLsize > 0 || WLsize > 0)
      dumpDevices();


    if (BTdata.containsKey("interval"))
    {
      trc(F("BLE interval"));

      unsigned int prevBLEinterval = BLEinterval;
      trc("previous interval");
      trc(BLEinterval);

      BLEinterval = (unsigned int)BTdata["interval"];
      trc("new interval");
      trc(BLEinterval);
      if (BLEinterval == 0)
      {
        if (BTtoMQTT())
          trc(F("Scan done"));
        BLEinterval = prevBLEinterval;
      }
    }

    if (BTdata.containsKey("minrssi"))
    {
      trc(F("Min RSSI"));

      trc("previous Minrssi");
      trc(Minrssi);

      Minrssi = (unsigned int)BTdata["minrssi"];
      trc("new Minrssi");
      trc(Minrssi);
    }
  }
}

#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayIR.ino"
# 28 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayIR.ino"
#include "User_config.h"

#ifdef ZgatewayIR

#if defined(ESP8266) || defined(ESP32)
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#ifdef DumpMode
IRrecv irrecv(IR_RECEIVER_PIN, 1024, 15U, true);
#else
IRrecv irrecv(IR_RECEIVER_PIN);
#endif
IRsend irsend(IR_EMITTER_PIN);
#else
#include <IRremote.h>
IRrecv irrecv(IR_RECEIVER_PIN);
IRsend irsend;
#endif


#ifndef NEC_BITS
#define NEC_BITS 32U
#endif
#ifndef SAMSUNG_BITS
#define SAMSUNG_BITS 32U
#endif
#ifndef SHARP_BITS
#define SHARP_ADDRESS_BITS 5U
#define SHARP_COMMAND_BITS 8U
#define SHARP_BITS (SHARP_ADDRESS_BITS + SHARP_COMMAND_BITS + 2)
#endif
#ifndef RC5_BITS
#define RC5_RAW_BITS 14U
#define RC5_BITS RC5_RAW_BITS - 2U
#endif
#ifndef DISH_BITS
#define DISH_BITS 16U
#endif
#ifndef SONY_12_BITS
#define SONY_12_BITS 12U
#endif
#ifndef LG_BITS
#define LG_BITS 28U
#endif
#ifndef WHYNTER_BITS
#define WHYNTER_BITS 32U
#endif

void setupIR()
{

#if defined(ESP8266) || defined(ESP32)
  irsend.begin();
#endif

  irrecv.enableIRIn();

  trc(F("IR_EMITTER_PIN "));
  trc(IR_EMITTER_PIN);
  trc(F("IR_RECEIVER_PIN "));
  trc(IR_RECEIVER_PIN);
  trc(F("ZgatewayIR setup done "));
}

void IRtoMQTT()
{
  decode_results results;

  if (irrecv.decode(&results))
  {
    trc(F("Creating IR buffer"));
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &IRdata = jsonBuffer.createObject();

    trc(F("Rcv. IR"));
#ifdef ESP32
    String taskMessage = "Task running on core ";
    taskMessage = taskMessage + xPortGetCoreID();
    trc(taskMessage);
#endif
    IRdata.set("value", (unsigned long)(results.value));
    IRdata.set("protocol", (int)(results.decode_type));
    IRdata.set("bits", (int)(results.bits));
#if defined(ESP8266) || defined(ESP32)
    String hex = resultToHexidecimal(&results);
    IRdata.set("hex", (char *)hex.c_str());
    IRdata.set("protocol_name", (char *)(typeToString(results.decode_type, false)).c_str());
#endif
    String rawCode = "";

    for (uint16_t i = 1; i < results.rawlen; i++)
    {
#if defined(ESP8266) || defined(ESP32)
      if (i % 100 == 0)
        yield();
      rawCode = rawCode + (results.rawbuf[i] * RAWTICK);
#else
      rawCode = rawCode + (results.rawbuf[i] * USECPERTICK);
#endif
      if (i < results.rawlen - 1)
        rawCode = rawCode + ",";
    }
    trc(rawCode);
    IRdata.set("raw", rawCode);

#ifdef RawDirectForward
#if defined(ESP8266) || defined(ESP32)
    uint16_t rawsend[results.rawlen];
    for (uint16_t i = 1; i < results.rawlen; i++)
    {
      if (i % 100 == 0)
        yield();
#else
    unsigned int rawsend[results.rawlen];
    for (int i = 1; i < results.rawlen; i++)
    {
#endif
      rawsend[i] = results.rawbuf[i];
    }
    irsend.sendRaw(rawsend, results.rawlen, RawFrequency);
    trc(F("raw redirected"));
#endif
    irrecv.resume();
    unsigned long MQTTvalue = IRdata.get<unsigned long long>("value");
    trc(MQTTvalue);
    if ((pubIRunknownPrtcl == false && IRdata.get<int>("protocol") == -1) )
    {
      trc(F("--no pub unknwn prt or data to high--"));
    }
    else if (!isAduplicate(MQTTvalue) && MQTTvalue != 0)
    {
      trc(F("Adv data IRtoMQTT"));
      pub(subjectIRtoMQTT, IRdata);
      trc(F("Store val"));
      storeValue(MQTTvalue);
      if (repeatIRwMQTT)
      {
        trc(F("Pub. IR for rpt"));
        pubMQTT(subjectMQTTtoIR, MQTTvalue);
      }
    }
  }
}

#ifdef simpleReceiving
void MQTTtoIR(char *topicOri, char *datacallback)
{



  bool signalSent = false;
  uint64_t data = 0;
  String strcallback = String(datacallback);
  trc(datacallback);
  unsigned int s = strcallback.length();

  int count = 0;
  for (int i = 0; i < s; i++)
  {
    if (datacallback[i] == ',')
    {
      count++;
    }
  }
  if (count == 0)
  {
    data = strtoul(datacallback, NULL, 10);
  }
#ifdef IR_GC
  else if (strstr(topicOri, "GC") != NULL)
  {
    trc(F("GC"));

    uint16_t GC[count + 1];
    String value = "";
    int j = 0;
    for (int i = 0; i < s; i++)
    {
      if (datacallback[i] != ',')
      {
        value = value + String(datacallback[i]);
      }
      if ((datacallback[i] == ',') || (i == s - 1))
      {
        GC[j] = value.toInt();
        value = "";
        j++;
      }
    }
    irsend.sendGC(GC, j);
    signalSent = true;
  }
#endif
#ifdef IR_RAW
  else if (strstr(topicOri, "Raw") != NULL)
  {
    trc(F("Raw"));

#if defined(ESP8266) || defined(ESP32)
    uint16_t Raw[count + 1];
#else
    unsigned int Raw[count + 1];
#endif
    String value = "";
    int j = 0;
    for (int i = 0; i < s; i++)
    {
      if (datacallback[i] != ',')
      {
        value = value + String(datacallback[i]);
      }
      if ((datacallback[i] == ',') || (i == s - 1))
      {
        Raw[j] = value.toInt();
        value = "";
        j++;
      }
    }
    irsend.sendRaw(Raw, j, RawFrequency);
    signalSent = true;
  }
#endif


  String topic = topicOri;
  unsigned int valueBITS = 0;
  int pos = topic.lastIndexOf(IRbitsKey);
  if (pos != -1)
  {
    pos = pos + +strlen(IRbitsKey);
    valueBITS = (topic.substring(pos, pos + 2)).toInt();
    trc(F("Bits nb:"));
    trc(valueBITS);
  }

  uint16_t valueRPT = 0;
  int pos2 = topic.lastIndexOf(IRRptKey);
  if (pos2 != -1)
  {
    pos2 = pos2 + strlen(IRRptKey);
    valueRPT = (topic.substring(pos2, pos2 + 1)).toInt();
    trc(F("IR repeat:"));
    trc(valueRPT);
  }

  if (topicOri && (strstr(topicOri, "NEC") == NULL))
  {
    trc("SendId prt");
    signalSent = sendIdentifiedProtocol(topicOri, data, datacallback, valueBITS, valueRPT);
  }
  else
  {
    trc(F("Not identified prt using NEC"));
    if (valueBITS == 0)
      valueBITS = NEC_BITS;
#if defined(ESP8266) || defined(ESP32)
    irsend.sendNEC(data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendNEC(data, valueBITS);
#endif
    signalSent = true;
  }

  if (signalSent)
  {
    pub(subjectGTWIRtoMQTT, datacallback);
  }
  irrecv.enableIRIn();
}
#endif

#ifdef jsonReceiving
void MQTTtoIR(char *topicOri, JsonObject &IRdata)
{

  if (cmpToMainTopic(topicOri, subjectMQTTtoIR))
  {
    trc(F("MQTTtoIR json"));
    uint64_t data = IRdata["value"];
    const char *raw = IRdata["raw"];
    const char *datastring = IRdata["datastring"];
    if (data != 0 || raw || datastring)
    {
      trc(F("MQTTtoIR value || raw || datasring ok"));
      bool signalSent = false;
      if (datastring)
      {
        trc(F("datastring"));
        trc(datastring);
      }
      else
      {
        datastring = "";
      }
      const char *protocol_name = IRdata["protocol_name"];
      unsigned int valueBITS = IRdata["bits"] | 0;
      uint16_t valueRPT = IRdata["repeat"] | repeatIRwNumber;

      if (raw)
      {
        trc(F("raw"));
        trc(raw);
        unsigned int s = strlen(raw);

        int count = 0;
        for (int i = 0; i < s; i++)
        {
          if (raw[i] == ',')
          {
            count++;
          }
        }
#ifdef IR_GC
        if (strstr(protocol_name, "GC") != NULL)
        {
          trc(F("GC"));

          uint16_t GC[count + 1];
          String value = "";
          int j = 0;
          for (int i = 0; i < s; i++)
          {
            if (raw[i] != ',')
            {
              value = value + String(raw[i]);
            }
            if ((raw[i] == ',') || (i == s - 1))
            {
              GC[j] = value.toInt();
              value = "";
              j++;
            }
          }
          irsend.sendGC(GC, j);
          signalSent = true;
        }
#endif
#ifdef IR_RAW
        if (strstr(protocol_name, "Raw") != NULL)
        {
          trc(F("Raw"));

#if defined(ESP8266) || defined(ESP32)
          uint16_t Raw[count + 1];
#else
          unsigned int Raw[count + 1];
#endif
          String value = "";
          int j = 0;
          for (int i = 0; i < s; i++)
          {
            if (raw[i] != ',')
            {
              value = value + String(raw[i]);
            }
            if ((raw[i] == ',') || (i == s - 1))
            {
              Raw[j] = value.toInt();
              value = "";
              j++;
            }
          }
          irsend.sendRaw(Raw, j, RawFrequency);
          signalSent = true;
        }
#endif
      }
      else if (protocol_name && (strstr(protocol_name, "NEC") == NULL))
      {
        signalSent = sendIdentifiedProtocol(protocol_name, data, datastring, valueBITS, valueRPT);
      }
      else
      {
        trc(F("Using NEC protocol"));
        if (valueBITS == 0)
          valueBITS = NEC_BITS;
#if defined(ESP8266) || defined(ESP32)
        irsend.sendNEC(data, valueBITS, valueRPT);
#else
        for (int i = 0; i <= valueRPT; i++)
          irsend.sendNEC(data, valueBITS);
#endif
        signalSent = true;
      }
      if (signalSent)
      {
        trc(F("MQTTtoIR OK"));
        pub(subjectGTWIRtoMQTT, IRdata);
      }
      irrecv.enableIRIn();
    }
    else
    {
      trc(F("MQTTtoIR failed json read"));
    }
  }
}
#endif

bool sendIdentifiedProtocol(const char *protocol_name, unsigned long long data, const char *datastring, unsigned int valueBITS, uint16_t valueRPT)
{
  unsigned char dataarray[valueBITS];
  const char *pointer = datastring;
  int i = 0;
  while (strlen(pointer) > 0 && i < valueBITS)
  {
    if (pointer[0] == ',')
    {
      pointer++;
    }
    else
    {
      dataarray[i] = strtol(pointer, (char **)&pointer, 16);
      i++;
    }
  }

#ifdef IR_WHYNTER
  if (strstr(protocol_name, "WHYNTER") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = WHYNTER_BITS;
#if defined(ESP8266) || defined(ESP32)
    irsend.sendWhynter(data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendWhynter(data, valueBITS);
#endif
    return true;
  }
#endif
#ifdef IR_LG
  if (strstr(protocol_name, "LG") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = LG_BITS;
#if defined(ESP8266) || defined(ESP32)
    irsend.sendLG(data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendLG(data, valueBITS);
#endif
    return true;
  }
#endif
#ifdef IR_SONY
  if (strstr(protocol_name, "SONY") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = SONY_12_BITS;
#if defined(ESP8266) || defined(ESP32)
    irsend.sendSony(data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendSony(data, valueBITS);
#endif
    return true;
  }
#endif
#ifdef IR_DISH
  if (strstr(protocol_name, "DISH") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = DISH_BITS;
#if defined(ESP8266) || defined(ESP32)
    irsend.sendDISH(data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendDISH(data, valueBITS);
#endif
    return true;
  }
#endif
#ifdef IR_RC5
  if (strstr(protocol_name, "RC5") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = RC5_BITS;
#if defined(ESP8266) || defined(ESP32)
    irsend.sendRC5(data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendRC5(data, valueBITS);
#endif
    return true;
  }
#endif
#ifdef IR_RC6
  if (strstr(protocol_name, "RC6") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = RC6_MODE0_BITS;
#if defined(ESP8266) || defined(ESP32)
    irsend.sendRC6(data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendRC6(data, valueBITS);
#endif
    return true;
  }
#endif
#ifdef IR_SHARP
  if (strstr(protocol_name, "SHARP") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = SHARP_BITS;
#if defined(ESP8266) || defined(ESP32)
    irsend.sendSharpRaw(data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendSharpRaw(data, valueBITS);
#endif
    return true;
  }
#endif
#ifdef IR_SAMSUNG
  if (strstr(protocol_name, "SAMSUNG") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = SAMSUNG_BITS;
#if defined(ESP8266) || defined(ESP32)
    irsend.sendSAMSUNG(data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendSAMSUNG(data, valueBITS);
#endif
    return true;
  }
#endif
#ifdef IR_JVC
  if (strstr(protocol_name, "JVC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = JVC_BITS;
#if defined(ESP8266) || defined(ESP32)
    irsend.sendJVC(data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendJVC(data, valueBITS);
#endif
    return true;
  }
#endif
#ifdef IR_PANASONIC
  if (strstr(protocol_name, "PANASONIC") != NULL)
  {
#if defined(ESP8266) || defined(ESP32)
    if (valueBITS == 0)
      valueBITS = PANASONIC_BITS;
    irsend.sendPanasonic(PanasonicAddress, data, valueBITS, valueRPT);
#else
    for (int i = 0; i <= valueRPT; i++)
      irsend.sendPanasonic(PanasonicAddress, data);
#endif
    return true;
  }
#endif

#if defined(ESP8266) || defined(ESP32)
#ifdef IR_COOLIX
  if (strstr(protocol_name, "COOLIX") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kCoolixBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kCoolixDefaultRepeat);
    irsend.sendCOOLIX(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_RCMM
  if (strstr(protocol_name, "RCMM") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kRCMMBits;
    irsend.sendRCMM(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_DENON
  if (strstr(protocol_name, "DENON") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = DENON_BITS;
    irsend.sendDenon(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_GICABLE
  if (strstr(protocol_name, "GICABLE") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kGicableBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kGicableMinRepeat);
    irsend.sendGICable(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_SHERWOOD
  if (strstr(protocol_name, "SHERWOOD") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kSherwoodBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kSherwoodMinRepeat);
    irsend.sendSherwood(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_MITSUBISHI
  if (strstr(protocol_name, "MITSUBISHI") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kMitsubishiBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishiMinRepeat);
    irsend.sendMitsubishi(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_NIKAI
  if (strstr(protocol_name, "NIKAI") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kNikaiBits;
    irsend.sendNikai(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_MIDEA
  if (strstr(protocol_name, "MIDEA") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kMideaBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMideaMinRepeat);
    irsend.sendMidea(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_MAGIQUEST
  if (strstr(protocol_name, "MAGIQUEST") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kMagiquestBits;
    irsend.sendMagiQuest(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_LASERTAG
  if (strstr(protocol_name, "LASERTAG") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kLasertagBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kLasertagMinRepeat);
    irsend.sendLasertag(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_CARRIER_AC
  if (strstr(protocol_name, "CARRIER_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kCarrierAcBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kCarrierAcMinRepeat);
    irsend.sendCarrierAC(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_MITSUBISHI2
  if (strstr(protocol_name, "MITSUBISHI2") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kMitsubishiBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishiMinRepeat);
    irsend.sendMitsubishi2(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_AIWA_RC_T501
  if (strstr(protocol_name, "AIWA_RC_T501") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kAiwaRcT501Bits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kAiwaRcT501MinRepeats);
    irsend.sendAiwaRCT501(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_DAIKIN
  if (strstr(protocol_name, "DAIKIN") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kDaikinStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikinDefaultRepeat);
    irsend.sendDaikin(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_KELVINATOR
  if (strstr(protocol_name, "KELVINATOR") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kKelvinatorStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kKelvinatorDefaultRepeat);
    irsend.sendKelvinator(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_MITSUBISHI_AC
  if (strstr(protocol_name, "MITSUBISHI_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kMitsubishiACStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishiACMinRepeat);
    irsend.sendMitsubishiAC(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_SANYO
  if (strstr(protocol_name, "SANYOLC7461") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kSanyoLC7461Bits;
    irsend.sendSanyoLC7461(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_GREE
  if (strstr(protocol_name, "GREE") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kGreeStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kGreeDefaultRepeat);
    irsend.sendGree(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_ARGO
  if (strstr(protocol_name, "ARGO") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kArgoStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kArgoDefaultRepeat);
    irsend.sendArgo(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_TROTEC
  if (strstr(protocol_name, "TROTEC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kTrotecStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kTrotecDefaultRepeat);
    irsend.sendTrotec(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_TOSHIBA_AC
  if (strstr(protocol_name, "TOSHIBA_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kToshibaACBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kToshibaACMinRepeat);
    irsend.sendToshibaAC(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_FUJITSU_AC
  if (strstr(protocol_name, "FUJITSU_AC") != NULL)
  {
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kFujitsuAcMinRepeat);
    irsend.sendFujitsuAC(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_HAIER_AC
  if (strstr(protocol_name, "HAIER_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kHaierACStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHaierAcDefaultRepeat);
    irsend.sendHaierAC(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_HITACHI_AC
  if (strstr(protocol_name, "HITACHI_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kHitachiAcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHitachiAcDefaultRepeat);
    irsend.sendHitachiAC(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_HITACHI_AC1
  if (strstr(protocol_name, "HITACHI_AC1") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kHitachiAc1StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHitachiAcDefaultRepeat);
    irsend.sendHitachiAC1(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_HITACHI_AC2
  if (strstr(protocol_name, "HITACHI_AC2") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kHitachiAc2StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHitachiAcDefaultRepeat);
    irsend.sendHitachiAC2(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_HAIER_AC_YRW02
  if (strstr(protocol_name, "HAIER_AC_YRW02") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kHaierACYRW02StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHaierAcYrw02DefaultRepeat);
    irsend.sendHaierACYRW02(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_WHIRLPOOL_AC
  if (strstr(protocol_name, "WHIRLPOOL_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kWhirlpoolAcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kWhirlpoolAcDefaultRepeat);
    irsend.sendWhirlpoolAC(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_SAMSUNG_AC
  if (strstr(protocol_name, "SAMSUNG_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kSamsungAcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kSamsungAcDefaultRepeat);
    irsend.sendSamsungAC(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_LUTRON
  if (strstr(protocol_name, "LUTRON") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kLutronBits;
    irsend.sendLutron(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_ELECTRA_AC
  if (strstr(protocol_name, "ELECTRA_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kElectraAcStateLength;
    irsend.sendElectraAC(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_PANASONIC_AC
  if (strstr(protocol_name, "PANASONIC_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kPanasonicAcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kPanasonicAcDefaultRepeat);
    irsend.sendPanasonicAC(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_PIONEER
  if (strstr(protocol_name, "PIONEER") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kPioneerBits;
    irsend.sendPioneer(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_LG2
  if (strstr(protocol_name, "LG2") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kLgBits;
    irsend.sendLG2(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_MWM
  if (strstr(protocol_name, "MWM") != NULL)
  {
    irsend.sendMWM(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_DAIKIN2
  if (strstr(protocol_name, "DAIKIN2") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kDaikin2StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin2DefaultRepeat);
    irsend.sendDaikin2(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_VESTEL_AC
  if (strstr(protocol_name, "VESTEL_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kVestelAcBits;
    irsend.sendVestelAc(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_SAMSUNG36
  if (strstr(protocol_name, "SAMSUNG36") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kSamsung36Bits;
    irsend.sendSamsung36(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_TCL112AC
  if (strstr(protocol_name, "TCL112AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kTcl112AcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kTcl112AcDefaultRepeat);
    irsend.sendTcl112Ac(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_TECO
  if (strstr(protocol_name, "TECO") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kTecoBits;
    irsend.sendTeco(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_LEGOPF
  if (strstr(protocol_name, "LEGOPF") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kLegoPfBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kLegoPfMinRepeat);
    irsend.sendLegoPf(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_MITSUBISHIHEAVY88
  if (strstr(protocol_name, "MITSUBISHIHEAVY88") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kMitsubishiHeavy88StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishiHeavy88MinRepeat);
    irsend.sendMitsubishiHeavy88(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_MITSUBISHIHEAVY152
  if (strstr(protocol_name, "MITSUBISHIHEAVY152") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kMitsubishiHeavy152StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishiHeavy152MinRepeat);
    irsend.sendMitsubishiHeavy152(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_DAIKIN216
  if (strstr(protocol_name, "DAIKIN216") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kDaikin216StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin216DefaultRepeat);
    irsend.sendDaikin216(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_SHARP_AC
  if (strstr(protocol_name, "SHARP_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kSharpAcStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kSharpAcDefaultRepeat);
    irsend.sendSharpAc(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_GOODWEATHER
  if (strstr(protocol_name, "GOODWEATHER_AC") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kGoodweatherBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kGoodweatherMinRepeat);
    irsend.sendGoodweather(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_INAX
  if (strstr(protocol_name, "INAX") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kInaxBits;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kInaxMinRepeat);
    irsend.sendInax(data, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_DAIKIN160
  if (strstr(protocol_name, "DAIKIN160") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kDaikin160StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin160DefaultRepeat);
    irsend.sendDaikin160(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_NEOCLIMA
  if (strstr(protocol_name, "NEOCLIMA") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kNeoclimaStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kNeoclimaMinRepeat);
    irsend.sendNeoclima(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_DAIKIN176
  if (strstr(protocol_name, "DAIKIN176") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kDaikin176StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin176DefaultRepeat);
    irsend.sendDaikin176(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_DAIKIN128
  if (strstr(protocol_name, "DAIKIN128") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kDaikin128StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin128DefaultRepeat);
    irsend.sendDaikin128(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_AMCOR
  if (strstr(protocol_name, "AMCOR") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kAmcorStateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kAmcorDefaultRepeat);
    irsend.sendAmcor(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_DAIKIN152
  if (strstr(protocol_name, "DAIKIN152") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kDaikin152StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kDaikin152DefaultRepeat);
    irsend.sendDaikin152(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_MITSUBISHI136
  if (strstr(protocol_name, "MITSUBISHI136") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kMitsubishi136StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishi136MinRepeat);
    irsend.sendMitsubishi136(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_MITSUBISHI112
  if (strstr(protocol_name, "MITSUBISHI112") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kMitsubishi112StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kMitsubishi112MinRepeat);
    irsend.sendMitsubishi112(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
#ifdef IR_HITACHI_AC424
  if (strstr(protocol_name, "HITACHI_AC424") != NULL)
  {
    if (valueBITS == 0)
      valueBITS = kHitachiAc424StateLength;
    if (valueRPT == repeatIRwNumber)
      valueRPT = std::max(valueRPT, kHitachiAcDefaultRepeat);
    irsend.sendHitachiAc424(dataarray, valueBITS, valueRPT);
    return true;
  }
#endif
  return false;
#endif
}
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayLORA.ino"
# 28 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayLORA.ino"
#include "User_config.h"

#ifdef ZgatewayLORA

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

void setupLORA()
{
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);

  if (!LoRa.begin(LORA_BAND))
  {
    trc(F("ZgatewayLORA setup failed!"));
    while (1)
      ;
  }
  LoRa.receive();
  trc(F("LORA_SCK"));
  trc(LORA_SCK);
  trc(F("LORA_MISO"));
  trc(LORA_MISO);
  trc(F("LORA_MOSI"));
  trc(LORA_MOSI);
  trc(F("LORA_SS"));
  trc(LORA_SS);
  trc(F("LORA_RST"));
  trc(LORA_RST);
  trc(F("LORA_DI0"));
  trc(LORA_DI0);
  trc(F("ZgatewayLORA setup done"));
}

void LORAtoMQTT()
{
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &LORAdata = jsonBuffer.createObject();
    trc(F("Rcv. LORA"));
#ifdef ESP32
    String taskMessage = "LORA Task running on core ";
    taskMessage = taskMessage + xPortGetCoreID();
    trc(taskMessage);
#endif
    String packet;
    packet = "";
    for (int i = 0; i < packetSize; i++)
    {
      packet += (char)LoRa.read();
    }
    LORAdata.set("rssi", (int)LoRa.packetRssi());
    LORAdata.set("snr", (float)LoRa.packetSnr());
    LORAdata.set("pferror", (float)LoRa.packetFrequencyError());
    LORAdata.set("packetSize", (int)packetSize);
    LORAdata.set("message", (char *)packet.c_str());
    pub(subjectLORAtoMQTT, LORAdata);
    if (repeatLORAwMQTT)
    {
      trc(F("Pub LORA for rpt"));
      pub(subjectMQTTtoLORA, LORAdata);
    }
  }
}

#ifdef jsonReceiving
void MQTTtoLORA(char *topicOri, JsonObject &LORAdata)
{
  if (cmpToMainTopic(topicOri, subjectMQTTtoLORA))
  {
    trc(F("MQTTtoLORA json"));
    const char *message = LORAdata["message"];
    int txPower = LORAdata["txpower"] | LORA_TX_POWER;
    int spreadingFactor = LORAdata["spreadingfactor"] | LORA_SPREADING_FACTOR;
    long int frequency = LORAdata["frequency "] | LORA_BAND;
    long int signalBandwidth = LORAdata["signalbandwidth"] | LORA_SIGNAL_BANDWIDTH;
    int codingRateDenominator = LORAdata["codingrate"] | LORA_CODING_RATE;
    int preambleLength = LORAdata["preamblelength"] | LORA_PREAMBLE_LENGTH;
    byte syncWord = LORAdata["syncword"] | LORA_SYNC_WORD;
    bool Crc = LORAdata["enablecrc"] | DEFAULT_CRC;
    if (message)
    {
      LoRa.setTxPower(txPower);
      LoRa.setFrequency(frequency);
      LoRa.setSpreadingFactor(spreadingFactor);
      LoRa.setSignalBandwidth(signalBandwidth);
      LoRa.setCodingRate4(codingRateDenominator);
      LoRa.setPreambleLength(preambleLength);
      LoRa.setSyncWord(syncWord);
      if (Crc)
        LoRa.enableCrc();
      LoRa.beginPacket();
      LoRa.print(message);
      LoRa.endPacket();
      trc(F("MQTTtoLORA OK"));
      pub(subjectGTWLORAtoMQTT, LORAdata);
    }
    else
    {
      trc(F("MQTTtoLORA Fail json"));
    }
  }
}
#endif
#ifdef simpleReceiving
void MQTTtoLORA(char *topicOri, char *LORAdata)
{
  if (cmpToMainTopic(topicOri, subjectMQTTtoLORA))
  {
    LoRa.beginPacket();
    LoRa.print(LORAdata);
    LoRa.endPacket();
    trc(F("MQTTtoLORA OK"));
    pub(subjectGTWLORAtoMQTT, LORAdata);
  }
}
#endif
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayPilight.ino"
# 28 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayPilight.ino"
#include "User_config.h"

#ifdef ZgatewayPilight
#include <ESPiLight.h>
ESPiLight rf(RF_EMITTER_PIN);

void pilightCallback(const String &protocol, const String &message, int status,
                     size_t repeats, const String &deviceID)
{
  if (status == VALID)
  {
    trc(F("Creating RF PiLight buffer"));
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &RFPiLightdata = jsonBuffer.createObject();
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer2;
    JsonObject &msg = jsonBuffer2.parseObject(message);
    RFPiLightdata.set("message", msg);
    RFPiLightdata.set("protocol", (char *)protocol.c_str());
    RFPiLightdata.set("length", (char *)deviceID.c_str());
    RFPiLightdata.set("repeats", (int)repeats);
    RFPiLightdata.set("status", (int)status);
    pub(subjectPilighttoMQTT, RFPiLightdata);
    if (repeatPilightwMQTT)
    {
      trc(F("Pub Pilight for rpt"));
      pub(subjectMQTTtoPilight, RFPiLightdata);
    }
  }
}

void setupPilight()
{
#ifndef ZgatewayRF &&ZgatewayRF2 &&ZgatewayRF315
  rf.setCallback(pilightCallback);
  rf.initReceiver(RF_RECEIVER_PIN);
  trc(F("RF_EMITTER_PIN "));
  trc(String(RF_EMITTER_PIN));
  trc(F("RF_RECEIVER_PIN "));
  trc(String(RF_RECEIVER_PIN));
  trc(F("ZgatewayPilight setup done "));
#else
  trc(F("ZgatewayPilight setup cannot be done, comment first ZgatewayRF && ZgatewayRF2 && ZgatewayRF315"));
#endif
}

void PilighttoMQTT()
{
  rf.loop();
}

void MQTTtoPilight(char *topicOri, JsonObject &Pilightdata)
{

  int result = 0;

  if (cmpToMainTopic(topicOri, subjectMQTTtoPilight))
  {
    trc(F("MQTTtoPilight json data analysis"));
    const char *message = Pilightdata["message"];
    const char *protocol = Pilightdata["protocol"];
    const char *raw = Pilightdata["raw"];
    if (raw)
    {
      int msgLength = 0;
      uint16_t codes[MAXPULSESTREAMLENGTH];
      msgLength = rf.stringToPulseTrain(
          raw,
          codes, MAXPULSESTREAMLENGTH);
      if (msgLength > 0)
      {
        trc(F("MQTTtoPilight raw ok"));
        rf.sendPulseTrain(codes, msgLength);
        result = msgLength;
      }
      else
      {
        trc(F("MQTTtoPilight raw KO"));
        switch (result)
        {
        case ESPiLight::ERROR_INVALID_PULSETRAIN_MSG_C:
          trc(F("'c' not found in string, or has no data"));
          break;
        case ESPiLight::ERROR_INVALID_PULSETRAIN_MSG_P:
          trc(F("'p' not found in string, or has no data"));
          break;
        case ESPiLight::ERROR_INVALID_PULSETRAIN_MSG_END:
          trc(F("';' or '@' not found in data string"));
          break;
        case ESPiLight::ERROR_INVALID_PULSETRAIN_MSG_TYPE:
          trc(F("pulse type not defined"));
          break;
        }
      }
    }
    else if (message && protocol)
    {
      trc(F("MQTTtoPilight msg & protocol ok"));
      result = rf.send(protocol, message);
    }
    else
    {
      trc(F("MQTTtoPilight failed json read"));
    }

    if (result > 0)
    {
      trc(F("Adv data MQTTtoPilight push state via PilighttoMQTT"));
      pub(subjectGTWPilighttoMQTT, Pilightdata);
    }
    else
    {
      switch (result)
      {
      case ESPiLight::ERROR_UNAVAILABLE_PROTOCOL:
        trc(F("protocol is not available"));
        break;
      case ESPiLight::ERROR_INVALID_PILIGHT_MSG:
        trc(F("message is invalid"));
        break;
      case ESPiLight::ERROR_INVALID_JSON:
        trc(F("message is not a proper json object"));
        break;
      case ESPiLight::ERROR_NO_OUTPUT_PIN:
        trc(F("no transmitter pin"));
        break;
      default:
        trc(F("invalid json data, can't read raw or message/protocol"));
        break;
      }
    }
  }
}
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayRF.ino"
# 28 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayRF.ino"
#include "User_config.h"

#ifdef ZgatewayRF

#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

#ifdef ZmqttDiscovery
void RFtoMQTTdiscovery(unsigned long MQTTvalue)
{
  char val[11];
  sprintf(val, "%lu", MQTTvalue);
  trc(F("switchRFDiscovery"));
  char *switchRF[8] = {"switch", val, "", "", "", val, "", ""};


  trc(F("CreateDiscoverySwitch"));
  trc(switchRF[1]);
  createDiscovery(switchRF[0],
                  subjectRFtoMQTT, switchRF[1], (char *)getUniqueId(switchRF[1], switchRF[2]).c_str(),
                  will_Topic, switchRF[3], switchRF[4],
                  switchRF[5], switchRF[6], switchRF[7],
                  0, "", "", true, subjectMQTTtoRF);
}
#endif

void setupRF()
{


  mySwitch.enableTransmit(RF_EMITTER_PIN);
  trc(F("RF_EMITTER_PIN "));
  trc(RF_EMITTER_PIN);
  mySwitch.setRepeatTransmit(RF_EMITTER_REPEAT);
  mySwitch.enableReceive(RF_RECEIVER_PIN);
  trc(F("RF_RECEIVER_PIN "));
  trc(RF_RECEIVER_PIN);
  trc(F("RF setup ok"));
}

void RFtoMQTT()
{

  if (mySwitch.available())
  {
    const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(4);
    StaticJsonBuffer<JSON_MSG_CALC_BUFFER> jsonBuffer;
    JsonObject &RFdata = jsonBuffer.createObject();
    trc(F("Rcv. RF"));
#ifdef ESP32
    String taskMessage = "RF Task running on core ";
    taskMessage = taskMessage + xPortGetCoreID();
    trc(taskMessage);
#endif
    RFdata.set("value", (unsigned long)mySwitch.getReceivedValue());
    RFdata.set("protocol", (int)mySwitch.getReceivedProtocol());
    RFdata.set("length", (int)mySwitch.getReceivedBitlength());
    RFdata.set("delay", (int)mySwitch.getReceivedDelay());
    mySwitch.resetAvailable();

    unsigned long MQTTvalue = RFdata.get<unsigned long>("value");
    if (!isAduplicate(MQTTvalue) && MQTTvalue != 0)
    {
#ifdef ZmqttDiscovery
      RFtoMQTTdiscovery(MQTTvalue);
#endif
      pub(subjectRFtoMQTT, RFdata);
      trc(F("Store val"));
      storeValue(MQTTvalue);
      if (repeatRFwMQTT)
      {
        trc(F("Pub RF for rpt"));
        pub(subjectMQTTtoRF, RFdata);
      }
    }
  }
}

#ifdef simpleReceiving
void MQTTtoRF(char *topicOri, char *datacallback)
{

  unsigned long data = strtoul(datacallback, NULL, 10);



  String topic = topicOri;
  int valuePRT = 0;
  int valuePLSL = 0;
  int valueBITS = 0;
  int pos = topic.lastIndexOf(RFprotocolKey);
  if (pos != -1)
  {
    pos = pos + +strlen(RFprotocolKey);
    valuePRT = (topic.substring(pos, pos + 1)).toInt();
    trc(F("RF Protocol:"));
    trc(valuePRT);
  }

  int pos2 = topic.lastIndexOf(RFpulselengthKey);
  if (pos2 != -1)
  {
    pos2 = pos2 + strlen(RFpulselengthKey);
    valuePLSL = (topic.substring(pos2, pos2 + 3)).toInt();
    trc(F("RF Pulse Lgth:"));
    trc(valuePLSL);
  }
  int pos3 = topic.lastIndexOf(RFbitsKey);
  if (pos3 != -1)
  {
    pos3 = pos3 + strlen(RFbitsKey);
    valueBITS = (topic.substring(pos3, pos3 + 2)).toInt();
    trc(F("Bits nb:"));
    trc(valueBITS);
  }

  if ((cmpToMainTopic(topicOri, subjectMQTTtoRF)) && (valuePRT == 0) && (valuePLSL == 0) && (valueBITS == 0))
  {
    trc(F("MQTTtoRF dflt"));
    mySwitch.setProtocol(1, 350);
    mySwitch.send(data, 24);

    pub(subjectGTWRFtoMQTT, datacallback);
  }
  else if ((valuePRT != 0) || (valuePLSL != 0) || (valueBITS != 0))
  {
    trc(F("MQTTtoRF usr par."));
    if (valuePRT == 0)
      valuePRT = 1;
    if (valuePLSL == 0)
      valuePLSL = 350;
    if (valueBITS == 0)
      valueBITS = 24;
    trc(valuePRT);
    trc(valuePLSL);
    trc(valueBITS);
    mySwitch.setProtocol(valuePRT, valuePLSL);
    mySwitch.send(data, valueBITS);

    pub(subjectGTWRFtoMQTT, datacallback);
  }
}
#endif

#ifdef jsonReceiving
void MQTTtoRF(char *topicOri, JsonObject &RFdata)
{
  if (cmpToMainTopic(topicOri, subjectMQTTtoRF))
  {
    trc(F("MQTTtoRF json"));
    unsigned long data = RFdata["value"];
    if (data != 0)
    {
      int valuePRT = RFdata["protocol"] | 1;
      int valuePLSL = RFdata["delay"] | 350;
      int valueBITS = RFdata["length"] | 24;
      int valueRPT = RFdata["repeat"] | RF_EMITTER_REPEAT;
      mySwitch.setRepeatTransmit(valueRPT);
      mySwitch.setProtocol(valuePRT, valuePLSL);
      mySwitch.send(data, valueBITS);
      trc(F("MQTTtoRF OK"));
      pub(subjectGTWRFtoMQTT, RFdata);
      mySwitch.setRepeatTransmit(RF_EMITTER_REPEAT);
    }
    else
    {
      trc(F("MQTTtoRF Fail json"));
    }
  }
}
#endif
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayRF2.ino"
# 36 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayRF2.ino"
#include "User_config.h"

#ifdef ZgatewayRF2

#include <NewRemoteTransmitter.h>
#include <NewRemoteReceiver.h>

struct RF2rxd
{
  unsigned int period;
  unsigned long address;
  unsigned long groupBit;
  unsigned long unit;
  unsigned long switchType;
  bool hasNewData;
};

RF2rxd rf2rd;

void setupRF2()
{
#ifndef ZgatewayRF
  NewRemoteReceiver::init(RF_RECEIVER_PIN, 2, rf2Callback);
  trc(F("RF_EMITTER_PIN "));
  trc(RF_EMITTER_PIN);
  trc(F("RF_RECEIVER_PIN "));
  trc(RF_RECEIVER_PIN);
  trc(F("ZgatewayRF2 setup done "));
#endif
  pinMode(RF_EMITTER_PIN, OUTPUT);
  digitalWrite(RF_EMITTER_PIN, LOW);
}

void RF2toMQTT()
{

  if (rf2rd.hasNewData)
  {
    trc(F("Creating RF2 buffer"));
    const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(5);
    StaticJsonBuffer<JSON_MSG_CALC_BUFFER> jsonBuffer;
    JsonObject &RF2data = jsonBuffer.createObject();

    rf2rd.hasNewData = false;

    trc(F("Rcv. RF2"));
    RF2data.set("unit", (int)rf2rd.unit);
    RF2data.set("groupBit", (int)rf2rd.groupBit);
    RF2data.set("period", (int)rf2rd.period);
    RF2data.set("address", (unsigned long)rf2rd.address);
    RF2data.set("switchType", (int)rf2rd.switchType);

    trc(F("Adv data RF2toMQTT"));
    pub(subjectRF2toMQTT, RF2data);
  }
}

void rf2Callback(unsigned int period, unsigned long address, unsigned long groupBit, unsigned long unit, unsigned long switchType)
{

  rf2rd.period = period;
  rf2rd.address = address;
  rf2rd.groupBit = groupBit;
  rf2rd.unit = unit;
  rf2rd.switchType = switchType;
  rf2rd.hasNewData = true;
}

#ifdef simpleReceiving
void MQTTtoRF2(char *topicOri, char *datacallback)
{



  String topic = topicOri;
  bool boolSWITCHTYPE;
  boolSWITCHTYPE = to_bool(datacallback);
  bool isDimCommand = false;

  long valueCODE = 0;
  int valueUNIT = -1;
  int valuePERIOD = 0;
  int valueGROUP = 0;
  int valueDIM = -1;

  int pos = topic.lastIndexOf(RF2codeKey);
  if (pos != -1)
  {
    pos = pos + +strlen(RF2codeKey);
    valueCODE = (topic.substring(pos, pos + 8)).toInt();
    trc(F("RF2 code:"));
    trc(valueCODE);
  }
  int pos2 = topic.lastIndexOf(RF2periodKey);
  if (pos2 != -1)
  {
    pos2 = pos2 + strlen(RF2periodKey);
    valuePERIOD = (topic.substring(pos2, pos2 + 3)).toInt();
    trc(F("RF2 Period:"));
    trc(valuePERIOD);
  }
  int pos3 = topic.lastIndexOf(RF2unitKey);
  if (pos3 != -1)
  {
    pos3 = pos3 + strlen(RF2unitKey);
    valueUNIT = (topic.substring(pos3, topic.indexOf("/", pos3))).toInt();
    trc(F("Unit:"));
    trc(valueUNIT);
  }
  int pos4 = topic.lastIndexOf(RF2groupKey);
  if (pos4 != -1)
  {
    pos4 = pos4 + strlen(RF2groupKey);
    valueGROUP = (topic.substring(pos4, pos4 + 1)).toInt();
    trc(F("RF2 Group:"));
    trc(valueGROUP);
  }
  int pos5 = topic.lastIndexOf(RF2dimKey);
  if (pos5 != -1)
  {
    isDimCommand = true;
    valueDIM = atoi(datacallback);
    trc(F("RF2 Dim:"));
    trc(valueDIM);
  }

  if ((topic == subjectMQTTtoRF2) || (valueCODE != 0) || (valueUNIT != -1) || (valuePERIOD != 0))
  {
    trc(F("MQTTtoRF2"));
    if (valueCODE == 0)
      valueCODE = 8233378;
    if (valueUNIT == -1)
      valueUNIT = 0;
    if (valuePERIOD == 0)
      valuePERIOD = 272;
    trc(valueCODE);
    trc(valueUNIT);
    trc(valuePERIOD);
    trc(valueGROUP);
    trc(boolSWITCHTYPE);
    trc(valueDIM);
    NewRemoteReceiver::disable();
    trc(F("Creating transmitter"));
    NewRemoteTransmitter transmitter(valueCODE, RF_EMITTER_PIN, valuePERIOD);
    trc(F("Sending data"));
    if (valueGROUP)
    {
      if (isDimCommand)
      {
        transmitter.sendGroupDim(valueDIM);
      }
      else
      {
        transmitter.sendGroup(boolSWITCHTYPE);
      }
    }
    else
    {
      if (isDimCommand)
      {
        transmitter.sendDim(valueUNIT, valueDIM);
      }
      else
      {
        transmitter.sendUnit(valueUNIT, boolSWITCHTYPE);
      }
    }
    trc(F("Data sent"));
    NewRemoteReceiver::enable();


    String MQTTAddress;
    String MQTTperiod;
    String MQTTunit;
    String MQTTgroupBit;
    String MQTTswitchType;
    String MQTTdimLevel;

    MQTTAddress = String(valueCODE);
    MQTTperiod = String(valuePERIOD);
    MQTTunit = String(valueUNIT);
    MQTTgroupBit = String(rf2rd.groupBit);
    MQTTswitchType = String(boolSWITCHTYPE);
    MQTTdimLevel = String(valueDIM);
    String MQTTRF2string;
    trc(F("Adv data MQTTtoRF2 push state via RF2toMQTT"));
    if (isDimCommand)
    {
      MQTTRF2string = subjectRF2toMQTT + String("/") + RF2codeKey + MQTTAddress + String("/") + RF2unitKey + MQTTunit + String("/") + RF2groupKey + MQTTgroupBit + String("/") + RF2dimKey + String("/") + RF2periodKey + MQTTperiod;
      pub((char *)MQTTRF2string.c_str(), (char *)MQTTdimLevel.c_str());
    }
    else
    {
      MQTTRF2string = subjectRF2toMQTT + String("/") + RF2codeKey + MQTTAddress + String("/") + RF2unitKey + MQTTunit + String("/") + RF2groupKey + MQTTgroupBit + String("/") + RF2periodKey + MQTTperiod;
      pub((char *)MQTTRF2string.c_str(), (char *)MQTTswitchType.c_str());
    }
  }
}
#endif

#ifdef jsonReceiving
void MQTTtoRF2(char *topicOri, JsonObject &RF2data)
{

  if (cmpToMainTopic(topicOri, subjectMQTTtoRF2))
  {
    trc(F("MQTTtoRF2 json"));
    int boolSWITCHTYPE = RF2data["switchType"] | 99;
    if (boolSWITCHTYPE != 99)
    {
      trc(F("MQTTtoRF2 switch type ok"));
      bool isDimCommand = boolSWITCHTYPE == 2;
      unsigned long valueCODE = RF2data["address"];
      int valueUNIT = RF2data["unit"] | -1;
      int valuePERIOD = RF2data["period"];
      int valueGROUP = RF2data["group"];
      int valueDIM = RF2data["dim"] | -1;
      if ((valueCODE != 0) || (valueUNIT != -1) || (valuePERIOD != 0))
      {
        trc(F("MQTTtoRF2"));
        if (valueCODE == 0)
          valueCODE = 8233378;
        if (valueUNIT == -1)
          valueUNIT = 0;
        if (valuePERIOD == 0)
          valuePERIOD = 272;
        trc(valueCODE);
        trc(valueUNIT);
        trc(valuePERIOD);
        trc(valueGROUP);
        trc(boolSWITCHTYPE);
        trc(valueDIM);
        NewRemoteReceiver::disable();
        NewRemoteTransmitter transmitter(valueCODE, RF_EMITTER_PIN, valuePERIOD);
        trc(F("Sending"));
        if (valueGROUP)
        {
          if (isDimCommand)
          {
            transmitter.sendGroupDim(valueDIM);
          }
          else
          {
            transmitter.sendGroup(boolSWITCHTYPE);
          }
        }
        else
        {
          if (isDimCommand)
          {
            transmitter.sendDim(valueUNIT, valueDIM);
          }
          else
          {
            transmitter.sendUnit(valueUNIT, boolSWITCHTYPE);
          }
        }
        trc(F("MQTTtoRF2 OK"));
        NewRemoteReceiver::enable();


        pub(subjectGTWRF2toMQTT, RF2data);
      }
    }
    else
    {
      trc(F("MQTTtoRF2 failed json read"));
    }
  }
}
#endif
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayRFM69.ino"
# 31 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewayRFM69.ino"
#include "User_config.h"

#ifdef ZgatewayRFM69

#include <RFM69.h>
#include <SPI.h>
#include <EEPROM.h>

char RadioConfig[128];



struct _GLOBAL_CONFIG
{
  uint32_t checksum;
  char rfmapname[32];
  char encryptkey[16 + 1];
  uint8_t networkid;
  uint8_t nodeid;
  uint8_t powerlevel;
  uint8_t rfmfrequency;
};

#define GC_POWER_LEVEL (pGC->powerlevel & 0x1F)
#define GC_IS_RFM69HCW ((pGC->powerlevel & 0x80) != 0)
#define SELECTED_FREQ(f) ((pGC->rfmfrequency == f) ? "selected" : "")

struct _GLOBAL_CONFIG *pGC;


uint32_t gc_checksum()
{
  uint8_t *p = (uint8_t *)pGC;
  uint32_t checksum = 0;
  p += sizeof(pGC->checksum);
  for (size_t i = 0; i < (sizeof(*pGC) - 4); i++)
  {
    checksum += *p++;
  }
  return checksum;
}

#if defined(ESP8266) || defined(ESP32)
void eeprom_setup()
{
  EEPROM.begin(4096);
  pGC = (struct _GLOBAL_CONFIG *)EEPROM.getDataPtr();

  if (gc_checksum() != pGC->checksum)
  {
    trc(F("Factory reset"));
    memset(pGC, 0, sizeof(*pGC));
    strcpy_P(pGC->encryptkey, ENCRYPTKEY);
    strcpy_P(pGC->rfmapname, RFM69AP_NAME);
    pGC->networkid = NETWORKID;
    pGC->nodeid = NODEID;
    pGC->powerlevel = ((IS_RFM69HCW) ? 0x80 : 0x00) | POWER_LEVEL;
    pGC->rfmfrequency = FREQUENCY;
    pGC->checksum = gc_checksum();
    EEPROM.commit();
  }
}
#endif

RFM69 radio;

void setupRFM69(void)
{
#if defined(ESP8266) || defined(ESP32)
  eeprom_setup();
#endif
  int freq;
  static const char PROGMEM JSONtemplate[] =
      R"({"msgType":"config","freq":%d,"rfm69hcw":%d,"netid":%d,"power":%d})";
  char payload[128];

  radio = RFM69(RFM69_CS, RFM69_IRQ, GC_IS_RFM69HCW, RFM69_IRQN);


  if (!radio.initialize(pGC->rfmfrequency, pGC->nodeid, pGC->networkid))
  {
    trc(F("ZgatewayRFM69 initialization failed"));
  }

  if (GC_IS_RFM69HCW)
  {
    radio.setHighPower();
  }
  radio.setPowerLevel(GC_POWER_LEVEL);

  if (pGC->encryptkey[0] != '\0')
    radio.encrypt(pGC->encryptkey);

  trc(F("ZgatewayRFM69 Listening and transmitting at"));
  switch (pGC->rfmfrequency)
  {
  case RF69_433MHZ:
    freq = 433;
    break;
  case RF69_868MHZ:
    freq = 868;
    break;
  case RF69_915MHZ:
    freq = 915;
    break;
  case RF69_315MHZ:
    freq = 315;
    break;
  default:
    freq = -1;
    break;
  }
  trc(freq);

  size_t len = snprintf_P(RadioConfig, sizeof(RadioConfig), JSONtemplate,
                          freq, GC_IS_RFM69HCW, pGC->networkid, GC_POWER_LEVEL);
  if (len >= sizeof(RadioConfig))
  {
    trc(F("\n\n*** RFM69 config truncated ***\n"));
  }
}

bool RFM69toMQTT(void)
{

  if (radio.receiveDone())
  {
    trc(F("Creating RFM69 buffer"));
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &RFM69data = jsonBuffer.createObject();
    uint8_t data[RF69_MAX_DATA_LEN + 1];
    uint8_t SENDERID = radio.SENDERID;
    uint8_t DATALEN = radio.DATALEN;
    uint16_t RSSI = radio.RSSI;


    memcpy(data, (void *)radio.DATA, DATALEN);
    data[DATALEN] = '\0';



    if (radio.ACKRequested())
    {
      radio.sendACK();
    }


    trc(F("Data received"));
    trc((const char *)data);

    char buff[sizeof(subjectRFM69toMQTT) + 4];
    sprintf(buff, "%s/%d", subjectRFM69toMQTT, SENDERID);
    RFM69data.set("data", (char *)data);
    RFM69data.set("rssi", (int)radio.RSSI);
    RFM69data.set("senderid", (int)radio.SENDERID);
    pub(buff, RFM69data);

    return true;
  }
  else
  {
    return false;
  }
}

#ifdef simpleReceiving
void MQTTtoRFM69(char *topicOri, char *datacallback)
{

  if (cmpToMainTopic(topicOri, subjectMQTTtoRFM69))
  {
    trc(F("MQTTtoRFM69 data analysis"));
    char data[RF69_MAX_DATA_LEN + 1];
    memcpy(data, (void *)datacallback, RF69_MAX_DATA_LEN);
    data[RF69_MAX_DATA_LEN] = '\0';


    String topic = topicOri;
    int valueRCV = defaultRFM69ReceiverId;
    int pos = topic.lastIndexOf(RFM69receiverKey);
    if (pos != -1)
    {
      pos = pos + +strlen(RFM69receiverKey);
      valueRCV = (topic.substring(pos, pos + 3)).toInt();
      trc(F("RFM69 receiver ID:"));
      trc(valueRCV);
    }
    if (radio.sendWithRetry(valueRCV, data, strlen(data), 10))
    {
      trc(F(" OK "));

      char buff[sizeof(subjectGTWRFM69toMQTT) + 4];
      sprintf(buff, "%s/%d", subjectGTWRFM69toMQTT, radio.SENDERID);
      pub(buff, data);
    }
    else
    {
      trc(F("RFM69 sending failed"));
    }
  }
}
#endif
#ifdef jsonReceiving
void MQTTtoRFM69(char *topicOri, JsonObject &RFM69data)
{

  if (cmpToMainTopic(topicOri, subjectMQTTtoRFM69))
  {
    const char *data = RFM69data["data"];
    trc(F("MQTTtoRFM69 json data analysis"));
    if (data)
    {
      trc(F("MQTTtoRFM69 data ok"));
      int valueRCV = RFM69data["receiverid"] | defaultRFM69ReceiverId;
      trc(F("RFM69 receiver ID:"));
      trc(valueRCV);
      if (radio.sendWithRetry(valueRCV, data, strlen(data), 10))
      {
        trc(F(" OK "));

        pub(subjectGTWRFM69toMQTT, RFM69data);
      }
      else
      {
        trc(F("MQTTtoRFM69 sending failed"));
      }
    }
    else
    {
      trc(F("MQTTtoRFM69 failed json read"));
    }
  }
}
#endif
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewaySRFB.ino"
# 31 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZgatewaySRFB.ino"
#include "User_config.h"

#ifdef ZgatewaySRFB

unsigned char _uartbuf[RF_MESSAGE_SIZE + 3] = {0};
unsigned char _uartpos = 0;

void setupSRFB()
{
  trc(F("ZgatewaySRFB setup done "));
  trc("Serial Baud" + String(SERIAL_BAUD));
}

void _rfbSend(byte *message)
{
  Serial.println();
  Serial.write(RF_CODE_START);
  Serial.write(RF_CODE_RFOUT);
  for (unsigned char j = 0; j < RF_MESSAGE_SIZE; j++)
  {
    Serial.write(message[j]);
  }
  Serial.write(RF_CODE_STOP);
  Serial.flush();
  Serial.println();
}

void _rfbSend(byte *message, int times)
{

  char buffer[RF_MESSAGE_SIZE];
  _rfbToChar(message, buffer);
  trc(F("[RFBRIDGE] Sending MESSAGE '%s' %d time(s)\n"));

  for (int i = 0; i < times; i++)
  {
    if (i > 0)
    {
      unsigned long start = millis();
      while (millis() - start < RF_SEND_DELAY)
        delay(1);
    }
    _rfbSend(message);
  }
}

bool SRFBtoMQTT()
{

  static bool receiving = false;

  while (Serial.available())
  {
    yield();
    byte c = Serial.read();

    if (receiving)
    {
      if (c == RF_CODE_STOP)
      {
        _rfbDecode();
        receiving = false;
      }
      else
      {
        _uartbuf[_uartpos++] = c;
      }
    }
    else if (c == RF_CODE_START)
    {
      _uartpos = 0;
      receiving = true;
    }
  }
  return receiving;
}

void _rfbDecode()
{

  static unsigned long last = 0;
  if (millis() - last < RF_RECEIVE_DELAY)
    return;
  last = millis();

  byte action = _uartbuf[0];
  char buffer[RF_MESSAGE_SIZE * 2 + 1] = {0};

  if (action == RF_CODE_RFIN)
  {
    _rfbToChar(&_uartbuf[1], buffer);

    trc(F("Creating SRFB buffer"));
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &SRFBdata = jsonBuffer.createObject();
    SRFBdata.set("raw", (char *)buffer);

    char Tsyn[4] = {0};
    extract_char(buffer, Tsyn, 0, 4, false, true);
    int val_Tsyn = (int)strtol(Tsyn, NULL, 10);
    SRFBdata.set("delay", (int)val_Tsyn);

    char Tlow[4] = {0};
    extract_char(buffer, Tlow, 4, 4, false, true);
    int val_Tlow = (int)strtol(Tlow, NULL, 10);
    SRFBdata.set("val_Tlow", (int)val_Tlow);

    char Thigh[4] = {0};
    extract_char(buffer, Thigh, 8, 4, false, true);
    int val_Thigh = (int)strtol(Thigh, NULL, 10);
    SRFBdata.set("val_Thigh", (int)val_Thigh);

    char val[8] = {0};
    extract_char(buffer, val, 12, 8, false, true);
    unsigned long MQTTvalue = (unsigned long)strtoul(val, NULL, 10);
    SRFBdata.set("value", (unsigned long)MQTTvalue);

    if (!isAduplicate(MQTTvalue) && MQTTvalue != 0)
    {
      trc(F("Adv data SRFBtoMQTT"));
      pub(subjectSRFBtoMQTT, SRFBdata);
      trc(F("Store val"));
      storeValue(MQTTvalue);
      if (repeatSRFBwMQTT)
      {
        trc(F("Publish SRFB for rpt"));
        pub(subjectMQTTtoSRFB, SRFBdata);
      }
    }
    _rfbAck();
  }
}

void _rfbAck()
{
  trc(F("[RFBRIDGE] Sending ACK\n"));
  Serial.println();
  Serial.write(RF_CODE_START);
  Serial.write(RF_CODE_ACK);
  Serial.write(RF_CODE_STOP);
  Serial.flush();
  Serial.println();
}




bool _rfbToArray(const char *in, byte *out)
{
  if (strlen(in) != RF_MESSAGE_SIZE * 2)
    return false;
  char tmp[3] = {0};
  for (unsigned char p = 0; p < RF_MESSAGE_SIZE; p++)
  {
    memcpy(tmp, &in[p * 2], 2);
    out[p] = strtol(tmp, NULL, 16);
  }
  return true;
}




bool _rfbToChar(byte *in, char *out)
{
  for (unsigned char p = 0; p < RF_MESSAGE_SIZE; p++)
  {
    sprintf_P(&out[p * 2], PSTR("%02X"), in[p]);
  }
  return true;
}

#ifdef simpleReceiving
void MQTTtoSRFB(char *topicOri, char *datacallback)
{


  String topic = topicOri;
  int valueRPT = 0;

  if (topic == subjectMQTTtoSRFB)
  {

    int valueMiniPLSL = 0;
    int valueMaxiPLSL = 0;
    int valueSYNC = 0;

    int pos = topic.lastIndexOf(SRFBRptKey);
    if (pos != -1)
    {
      pos = pos + +strlen(SRFBRptKey);
      valueRPT = (topic.substring(pos, pos + 1)).toInt();
      trc(F("SRFB Repeat:"));
      trc(valueRPT);
    }

    int pos2 = topic.lastIndexOf(SRFBminipulselengthKey);
    if (pos2 != -1)
    {
      pos2 = pos2 + strlen(SRFBminipulselengthKey);
      valueMiniPLSL = (topic.substring(pos2, pos2 + 3)).toInt();
      trc(F("RF Mini Pulse Lgth:"));
      trc(valueMiniPLSL);
    }

    int pos3 = topic.lastIndexOf(SRFBmaxipulselengthKey);
    if (pos3 != -1)
    {
      pos3 = pos3 + strlen(SRFBmaxipulselengthKey);
      valueMaxiPLSL = (topic.substring(pos3, pos3 + 2)).toInt();
      trc(F("RF Maxi Pulse Lgth:"));
      trc(valueMaxiPLSL);
    }

    int pos4 = topic.lastIndexOf(SRFBsyncKey);
    if (pos4 != -1)
    {
      pos4 = pos4 + strlen(SRFBsyncKey);
      valueSYNC = (topic.substring(pos4, pos4 + 2)).toInt();
      trc(F("RF sync:"));
      trc(valueSYNC);
    }

    trc(F("MQTTtoSRFB prts"));
    if (valueRPT == 0)
      valueRPT = 1;
    if (valueMiniPLSL == 0)
      valueMiniPLSL = 320;
    if (valueMaxiPLSL == 0)
      valueMaxiPLSL = 900;
    if (valueSYNC == 0)
      valueSYNC = 9500;

    byte hex_valueMiniPLSL[2];
    hex_valueMiniPLSL[0] = (int)((valueMiniPLSL >> 8) & 0xFF);
    hex_valueMiniPLSL[1] = (int)(valueMiniPLSL & 0xFF);

    byte hex_valueMaxiPLSL[2];
    hex_valueMaxiPLSL[0] = (int)((valueMaxiPLSL >> 8) & 0xFF);
    hex_valueMaxiPLSL[1] = (int)(valueMaxiPLSL & 0xFF);

    byte hex_valueSYNC[2];
    hex_valueSYNC[0] = (int)((valueSYNC >> 8) & 0xFF);
    hex_valueSYNC[1] = (int)(valueSYNC & 0xFF);

    unsigned long data = strtoul(datacallback, NULL, 10);
    byte hex_data[3];
    hex_data[0] = (unsigned long)((data >> 16) & 0xFF);
    hex_data[1] = (unsigned long)((data >> 8) & 0xFF);
    hex_data[2] = (unsigned long)(data & 0xFF);

    byte message_b[RF_MESSAGE_SIZE];

    memcpy(message_b, hex_valueSYNC, 2);
    memcpy(message_b + 2, hex_valueMiniPLSL, 2);
    memcpy(message_b + 4, hex_valueMaxiPLSL, 2);
    memcpy(message_b + 6, hex_data, 3);

    _rfbSend(message_b, valueRPT);

    pub(subjectGTWSRFBtoMQTT, datacallback);
  }
  if (topic == subjectMQTTtoSRFBRaw)
  {

    int pos = topic.lastIndexOf(SRFBRptKey);
    if (pos != -1)
    {
      pos = pos + +strlen(SRFBRptKey);
      valueRPT = (topic.substring(pos, pos + 1)).toInt();
      trc(F("SRFB Repeat:"));
      trc(valueRPT);
    }
    if (valueRPT == 0)
      valueRPT = 1;

    byte message_b[RF_MESSAGE_SIZE];
    _rfbToArray(datacallback, message_b);
    _rfbSend(message_b, valueRPT);

    pub(subjectGTWSRFBtoMQTT, datacallback);
  }
}
#endif
#ifdef jsonReceiving
void MQTTtoSRFB(char *topicOri, JsonObject &SRFBdata)
{


  const char *raw = SRFBdata["raw"];
  int valueRPT = SRFBdata["repeat"] | 1;
  if (cmpToMainTopic(topicOri, subjectMQTTtoSRFB))
  {
    trc(F("MQTTtoSRFB json"));
    if (raw)
    {
      trc(F("MQTTtoSRFB raw ok"));
      byte message_b[RF_MESSAGE_SIZE];
      _rfbToArray(raw, message_b);
      _rfbSend(message_b, valueRPT);
    }
    else
    {
      unsigned long data = SRFBdata["value"];
      if (data != 0)
      {
        trc(F("MQTTtoSRFB data ok"));
        int valueMiniPLSL = SRFBdata["val_Tlow"];
        int valueMaxiPLSL = SRFBdata["val_Thigh"];
        int valueSYNC = SRFBdata["delay"];

        if (valueRPT == 0)
          valueRPT = 1;
        if (valueMiniPLSL == 0)
          valueMiniPLSL = 320;
        if (valueMaxiPLSL == 0)
          valueMaxiPLSL = 900;
        if (valueSYNC == 0)
          valueSYNC = 9500;

        byte hex_valueMiniPLSL[2];
        hex_valueMiniPLSL[0] = (int)((valueMiniPLSL >> 8) & 0xFF);
        hex_valueMiniPLSL[1] = (int)(valueMiniPLSL & 0xFF);

        byte hex_valueMaxiPLSL[2];
        hex_valueMaxiPLSL[0] = (int)((valueMaxiPLSL >> 8) & 0xFF);
        hex_valueMaxiPLSL[1] = (int)(valueMaxiPLSL & 0xFF);

        byte hex_valueSYNC[2];
        hex_valueSYNC[0] = (int)((valueSYNC >> 8) & 0xFF);
        hex_valueSYNC[1] = (int)(valueSYNC & 0xFF);

        byte hex_data[3];
        hex_data[0] = (unsigned long)((data >> 16) & 0xFF);
        hex_data[1] = (unsigned long)((data >> 8) & 0xFF);
        hex_data[2] = (unsigned long)(data & 0xFF);

        byte message_b[RF_MESSAGE_SIZE];

        memcpy(message_b, hex_valueSYNC, 2);
        memcpy(message_b + 2, hex_valueMiniPLSL, 2);
        memcpy(message_b + 4, hex_valueMaxiPLSL, 2);
        memcpy(message_b + 6, hex_data, 3);

        trc(F("MQTTtoSRFB OK"));
        _rfbSend(message_b, valueRPT);

        pub(subjectGTWSRFBtoMQTT, SRFBdata);
      }
      else
      {
        trc(F("MQTTtoSRFB error decoding value"));
      }
    }
  }
}
#endif
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZmqttDiscovery.ino"
# 26 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZmqttDiscovery.ino"
#include "User_config.h"

#ifdef ZmqttDiscovery

String getMacAddress()
{
  uint8_t baseMac[6];
  char baseMacChr[13] = {0};
#if defined(ESP8266)
  WiFi.macAddress(baseMac);
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
#elif defined(ESP32)
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
#else
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
  return String(baseMacChr);
}

String getUniqueId(String name, String sufix)
{
  String uniqueId = (String)getMacAddress() + name + sufix;
  return String(uniqueId);
}

void createDiscovery(char *sensor_type,
                     char *st_topic, char *s_name, char *unique_id,
                     char *availability_topic, char *device_class, char *value_template,
                     char *payload_on, char *payload_off, char *unit_of_meas,
                     int off_delay,
                     char *payload_available, char *payload_not_avalaible, bool child_device, char *cmd_topic)
{
  const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(14) + JSON_OBJECT_SIZE(5) + JSON_ARRAY_SIZE(1);
  StaticJsonBuffer<JSON_MSG_CALC_BUFFER> jsonBuffer;
  JsonObject &sensor = jsonBuffer.createObject();

  char state_topic[mqtt_topic_max_size];
  strcpy(state_topic, mqtt_topic);
  strcat(state_topic, st_topic);
  sensor.set("stat_t", state_topic);

  sensor.set("name", s_name);
  sensor.set("uniq_id", unique_id);
  if (device_class[0])
    sensor.set("dev_cla", device_class);
  if (value_template[0])
    sensor.set("val_tpl", value_template);
  if (payload_on[0])
    sensor.set("pl_on", payload_on);
  if (payload_off[0])
    sensor.set("pl_off", payload_off);
  if (unit_of_meas[0])
    sensor.set("unit_of_meas", unit_of_meas);
  if (off_delay != 0)
    sensor.set("off_delay", off_delay);
  if (payload_available[0])
    sensor.set("pl_avail", payload_available);
  if (payload_not_avalaible[0])
    sensor.set("pl_not_avail", payload_not_avalaible);

  if (cmd_topic[0])
  {
    char command_topic[mqtt_topic_max_size];
    strcpy(command_topic, mqtt_topic);
    strcat(command_topic, cmd_topic);
    sensor.set("cmd_t", command_topic);
  }
# 103 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZmqttDiscovery.ino"
  if (child_device)
  {
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonDeviceBuffer;
    JsonObject &device = jsonDeviceBuffer.createObject();
    device.set("name", gateway_name);
    device.set("manufacturer", DEVICEMANUFACTURER);
    device.set("sw_version", OMG_VERSION);
    JsonArray &identifiers = device.createNestedArray("identifiers");
    identifiers.add(getMacAddress());
    sensor.set("device", device);
  }
  String topic = String(discovery_Topic) + "/" + String(sensor_type) + "/" + String(unique_id) + "/config";
  pub_custom_topic((char *)topic.c_str(), sensor, true);
}

void pubMqttDiscovery()
{
  trc(F("omgStatusDiscovery"));
  createDiscovery("binary_sensor",
                  will_Topic, Gateway_Name, (char *)getUniqueId("", "").c_str(),
                  will_Topic, "connectivity", "",
                  Gateway_AnnouncementMsg, will_Message, "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, false, ""
  );
  createDiscovery("switch",
                  will_Topic, "restart OMG", (char *)getUniqueId("restart", "").c_str(),
                  will_Topic, "", "",
                  "{\"cmd\":\"restart\"}", "", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoSYSset
  );
  createDiscovery("switch",
                  will_Topic, "erase OMG", (char *)getUniqueId("erase", "").c_str(),
                  will_Topic, "", "",
                  "{\"cmd\":\"erase\"}", "", "",
                  0,
                  Gateway_AnnouncementMsg, will_Message, true, subjectMQTTtoSYSset
  );

#ifdef ZsensorBME280
#define BMEparametersCount 6
  trc(F("bme280Discovery"));
  char *BMEsensor[BMEparametersCount][8] = {
      {"sensor", "tempc", "bme", "temperature", "{{ value_json.tempc }}", "", "", "°C"},
      {"sensor", "tempf", "bme", "temperature", "{{ value_json.tempf }}", "", "", "°F"},
      {"sensor", "pa", "bme", "", "{{ float(value_json.pa) * 0.01 }}", "", "", "hPa"},
      {"sensor", "hum", "bme", "humidity", "{{ value_json.hum }}", "", "", "%"},
      {"sensor", "altim", "bme", "", "{{ value_json.altim }}", "", "", "m"},
      {"sensor", "altift", "bme", "", "{{ value_json.altift }}", "", "", "ft"}

  };

  for (int i = 0; i < BMEparametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(BMEsensor[i][1]);
    createDiscovery(BMEsensor[i][0],
                    BMETOPIC, BMEsensor[i][1], (char *)getUniqueId(BMEsensor[i][1], BMEsensor[i][2]).c_str(),
                    will_Topic, BMEsensor[i][3], BMEsensor[i][4],
                    BMEsensor[i][5], BMEsensor[i][6], BMEsensor[i][7],
                    0, "", "", true, "");
  }
#endif

#ifdef ZsensorDHT
#define DHTparametersCount 2
  trc(F("DHTDiscovery"));
  char *DHTsensor[DHTparametersCount][8] = {
      {"sensor", "tempc", "dht", "temperature", "{{ value_json.temp }}", "", "", "°C"},
      {"sensor", "hum", "dht", "humidity", "{{ value_json.hum }}", "", "", "%"}};

  for (int i = 0; i < DHTparametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(DHTsensor[i][1]);
    createDiscovery(DHTsensor[i][0],
                    DHTTOPIC, DHTsensor[i][1], (char *)getUniqueId(DHTsensor[i][1], DHTsensor[i][2]).c_str(),
                    will_Topic, DHTsensor[i][3], DHTsensor[i][4],
                    DHTsensor[i][5], DHTsensor[i][6], DHTsensor[i][7],
                    0, "", "", true, "");
  }
#endif

#ifdef ZsensorADC
  trc(F("ADCDiscovery"));
  char *ADCsensor[8] = {"sensor", "adc", "", "", "{{ value_json.adc }}", "", "", ""};


  trc(F("CreateDiscoverySensor"));
  trc(ADCsensor[1]);
  createDiscovery(ADCsensor[0],
                  ADCTOPIC, ADCsensor[1], (char *)getUniqueId(ADCsensor[1], ADCsensor[2]).c_str(),
                  will_Topic, ADCsensor[3], ADCsensor[4],
                  ADCsensor[5], ADCsensor[6], ADCsensor[7],
                  0, "", "", true, "");
#endif

#ifdef ZsensorBH1750
#define BH1750parametersCount 3
  trc(F("BH1750Discovery"));
  char *BH1750sensor[BH1750parametersCount][8] = {
      {"sensor", "lux", "BH1750", "illuminance", "{{ value_json.lux }}", "", "", "lu"},
      {"sensor", "ftCd", "BH1750", "", "{{ value_json.ftCd }}", "", "", ""},
      {"sensor", "wattsm2", "BH1750", "", "{{ value_json.wattsm2 }}", "", "", "wm²"}

  };

  for (int i = 0; i < BH1750parametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(BH1750sensor[i][1]);
    createDiscovery(BH1750sensor[i][0],
                    subjectBH1750toMQTT, BH1750sensor[i][1], (char *)getUniqueId(BH1750sensor[i][1], BH1750sensor[i][2]).c_str(),
                    will_Topic, BH1750sensor[i][3], BH1750sensor[i][4],
                    BH1750sensor[i][5], BH1750sensor[i][6], BH1750sensor[i][7],
                    0, "", "", true, "");
  }
#endif

#ifdef ZsensorTSL2561
#define TSL2561parametersCount 3
  trc(F("TSL2561Discovery"));
  char *TSL2561sensor[TSL2561parametersCount][8] = {
      {"sensor", "lux", "TSL2561", "illuminance", "{{ value_json.lux }}", "", "", "lu"},
      {"sensor", "ftcd", "TSL2561", "", "{{ value_json.ftcd }}", "", "", ""},
      {"sensor", "wattsm2", "TSL2561", "", "{{ value_json.wattsm2 }}", "", "", "wm²"}

  };

  for (int i = 0; i < TSL2561parametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(TSL2561sensor[i][1]);
    createDiscovery(TSL2561sensor[i][0],
                    subjectTSL12561toMQTT, TSL2561sensor[i][1], (char *)getUniqueId(TSL2561sensor[i][1], TSL2561sensor[i][2]).c_str(),
                    will_Topic, TSL2561sensor[i][3], TSL2561sensor[i][4],
                    TSL2561sensor[i][5], TSL2561sensor[i][6], TSL2561sensor[i][7],
                    0, "", "", true, "");
  }
#endif

#ifdef ZsensorHCSR501
  trc(F("HCSR501Discovery"));
  char *HCSR501sensor[8] = {"binary_sensor", "hcsr501", "", "", "{{value_json.hcsr501}}", "true", "false", ""};


  trc(F("CreateDiscoverySensor"));
  trc(HCSR501sensor[1]);
  createDiscovery(HCSR501sensor[0],
                  subjectHCSR501toMQTT, HCSR501sensor[1], (char *)getUniqueId(HCSR501sensor[1], HCSR501sensor[2]).c_str(),
                  will_Topic, HCSR501sensor[3], HCSR501sensor[4],
                  HCSR501sensor[5], HCSR501sensor[6], HCSR501sensor[7],
                  0, "", "", true, "");
#endif

#ifdef ZsensorGPIOInput
  trc(F("GPIOInputDiscovery"));
  char *GPIOInputsensor[8] = {"binary_sensor", "GPIOInput", "", "", "{{value_json.gpio}}", "HIGH", "LOW", ""};


  trc(F("CreateDiscoverySensor"));
  trc(GPIOInputsensor[1]);
  createDiscovery(GPIOInputsensor[0],
                  subjectGPIOInputtoMQTT, GPIOInputsensor[1], (char *)getUniqueId(GPIOInputsensor[1], GPIOInputsensor[2]).c_str(),
                  will_Topic, GPIOInputsensor[3], GPIOInputsensor[4],
                  GPIOInputsensor[5], GPIOInputsensor[6], GPIOInputsensor[7],
                  0, "", "", true, "");
#endif

#ifdef ZsensorINA226
#define INA226parametersCount 3
  trc(F("INA226Discovery"));
  char *INA226sensor[INA226parametersCount][8] = {
      {"sensor", "volt", "INA226", "", "{{ value_json.volt }}", "", "", "V"},
      {"sensor", "current", "INA226", "", "{{ value_json.current }}", "", "", "A"},
      {"sensor", "power", "INA226", "", "{{ value_json.power }}", "", "", "W"}

  };

  for (int i = 0; i < INA226parametersCount; i++)
  {
    trc(F("CreateDiscoverySensor"));
    trc(INA226sensor[i][1]);
    createDiscovery(INA226sensor[i][0],
                    subjectINA226toMQTT, INA226sensor[i][1], (char *)getUniqueId(INA226sensor[i][1], INA226sensor[i][2]).c_str(),
                    will_Topic, INA226sensor[i][3], INA226sensor[i][4],
                    INA226sensor[i][5], INA226sensor[i][6], INA226sensor[i][7],
                    0, "", "", true, "");
  }
#endif

#ifdef ZactuatorONOFF
  trc(F("actuatorONOFFDiscovery"));
  char *actuatorONOFF[8] = {"switch", "actuatorONOFF", "", "", "", "{\"cmd\":1}", "{\"cmd\":0}", ""};


  trc(F("CreateDiscoverySensor"));
  trc(actuatorONOFF[1]);
  createDiscovery(actuatorONOFF[0],
                  subjectGTWONOFFtoMQTT, actuatorONOFF[1], (char *)getUniqueId(actuatorONOFF[1], actuatorONOFF[2]).c_str(),
                  will_Topic, actuatorONOFF[3], actuatorONOFF[4],
                  actuatorONOFF[5], actuatorONOFF[6], actuatorONOFF[7],
                  0, "", "", true, subjectMQTTtoONOFF);
#endif

#ifdef ZgatewayRF

  trc(F("gatewayRFDiscovery"));
  char *gatewayRF[8] = {"sensor", "gatewayRF", "", "", "{{ value_json.value }}", "", "", ""};


  trc(F("CreateDiscoverySensor"));
  trc(gatewayRF[1]);
  createDiscovery(gatewayRF[0],
                  subjectRFtoMQTT, gatewayRF[1], (char *)getUniqueId(gatewayRF[1], gatewayRF[2]).c_str(),
                  will_Topic, gatewayRF[3], gatewayRF[4],
                  gatewayRF[5], gatewayRF[6], gatewayRF[7],
                  0, "", "", true, "");
#endif

#ifdef ZgatewayRF2

  trc(F("gatewayRF2Discovery"));
  char *gatewayRF2[8] = {"sensor", "gatewayRF2", "", "", "{{ value_json.value }}", "", "", ""};


  trc(F("CreateDiscoverySensor"));
  trc(gatewayRF2[1]);
  createDiscovery(gatewayRF2[0],
                  subjectRF2toMQTT, gatewayRF2[1], (char *)getUniqueId(gatewayRF2[1], gatewayRF2[2]).c_str(),
                  will_Topic, gatewayRF2[3], gatewayRF2[4],
                  gatewayRF2[5], gatewayRF2[6], gatewayRF2[7],
                  0, "", "", true, "");
#endif

#ifdef ZgatewayRFM69

  trc(F("gatewayRFM69Discovery"));
  char *gatewayRFM69[8] = {"sensor", "gatewayRFM69", "", "", "{{ value_json.value }}", "", "", ""};


  trc(F("CreateDiscoverySensor"));
  trc(gatewayRFM69[1]);
  createDiscovery(gatewayRFM69[0],
                  subjectRFM69toMQTT, gatewayRFM69[1], (char *)getUniqueId(gatewayRFM69[1], gatewayRFM69[2]).c_str(),
                  will_Topic, gatewayRFM69[3], gatewayRFM69[4],
                  gatewayRFM69[5], gatewayRFM69[6], gatewayRFM69[7],
                  0, "", "", true, "");
#endif

#ifdef ZgatewayLORA

  trc(F("gatewayLORADiscovery"));
  char *gatewayLORA[8] = {"sensor", "gatewayLORA", "", "", "{{ value_json.message }}", "", "", ""};


  trc(F("CreateDiscoverySensor"));
  trc(gatewayLORA[1]);
  createDiscovery(gatewayLORA[0],
                  subjectLORAtoMQTT, gatewayLORA[1], (char *)getUniqueId(gatewayLORA[1], gatewayLORA[2]).c_str(),
                  will_Topic, gatewayLORA[3], gatewayLORA[4],
                  gatewayLORA[5], gatewayLORA[6], gatewayLORA[7],
                  0, "", "", true, "");
#endif

#ifdef ZgatewaySRFB

  trc(F("gatewaySRFBDiscovery"));
  char *gatewaySRFB[8] = {"sensor", "gatewaySRFB", "", "", "{{ value_json.value }}", "", "", ""};


  trc(F("CreateDiscoverySensor"));
  trc(gatewaySRFB[1]);
  createDiscovery(gatewaySRFB[0],
                  subjectSRFBtoMQTT, gatewaySRFB[1], (char *)getUniqueId(gatewaySRFB[1], gatewaySRFB[2]).c_str(),
                  will_Topic, gatewaySRFB[3], gatewaySRFB[4],
                  gatewaySRFB[5], gatewaySRFB[6], gatewaySRFB[7],
                  0, "", "", true, "");
#endif

#ifdef ZgatewayPilight

  trc(F("gatewayPilightDiscovery"));
  char *gatewayPilight[8] = {"sensor", "gatewayPilight", "", "", "{{ value_json.message }}", "", "", ""};


  trc(F("CreateDiscoverySensor"));
  trc(gatewayPilight[1]);
  createDiscovery(gatewayPilight[0],
                  subjectPilighttoMQTT, gatewayPilight[1], (char *)getUniqueId(gatewayPilight[1], gatewayPilight[2]).c_str(),
                  will_Topic, gatewayPilight[3], gatewayPilight[4],
                  gatewayPilight[5], gatewayPilight[6], gatewayPilight[7],
                  0, "", "", true, "");
#endif

#ifdef ZgatewayIR

  trc(F("gatewayIRDiscovery"));
  char *gatewayIR[8] = {"sensor", "gatewayIR", "", "", "{{ value_json.value }}", "", "", ""};


  trc(F("CreateDiscoverySensor"));
  trc(gatewayIR[1]);
  createDiscovery(gatewayIR[0],
                  subjectIRtoMQTT, gatewayIR[1], (char *)getUniqueId(gatewayIR[1], gatewayIR[2]).c_str(),
                  will_Topic, gatewayIR[3], gatewayIR[4],
                  gatewayIR[5], gatewayIR[6], gatewayIR[7],
                  0, "", "", true, "");
#endif

#ifdef Zgateway2G

  trc(F("gateway2GDiscovery"));
  char *gateway2G[8] = {"sensor", "gateway2G", "", "", "{{ value_json.message }}", "", "", ""};


  trc(F("CreateDiscoverySensor"));
  trc(gateway2G[1]);
  createDiscovery(gateway2G[0],
                  subject2GtoMQTT, gateway2G[1], (char *)getUniqueId(gateway2G[1], gateway2G[2]).c_str(),
                  will_Topic, gateway2G[3], gateway2G[4],
                  gateway2G[5], gateway2G[6], gateway2G[7],
                  0, "", "", true, "");
#endif

#ifdef ZgatewayBT

  trc(F("gatewayBTDiscovery"));
  char *gatewayBT[8] = {"sensor", "gatewayBT", "", "", "{{ value_json.id }}", "", "", ""};


  trc(F("CreateDiscoverySensor"));
  trc(gatewayBT[1]);
  createDiscovery(gatewayBT[0],
                  subjectBTtoMQTT, gatewayBT[1], (char *)getUniqueId(gatewayBT[1], gatewayBT[2]).c_str(),
                  will_Topic, gatewayBT[3], gatewayBT[4],
                  gatewayBT[5], gatewayBT[6], gatewayBT[7],
                  0, "", "", true, "");
#endif
}

#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorADC.ino"
# 29 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorADC.ino"
#include "User_config.h"

#ifdef ZsensorADC

#if defined(ESP8266)
ADC_MODE(ADC_TOUT);
#endif


unsigned long timeadc = 0;

void MeasureADC()
{
  if (millis() > (timeadc + TimeBetweenReadingADC))
  {
#if defined(ESP8266)
    yield();
#endif
    timeadc = millis();
    static int persistedadc;
    int val = analogRead(ADC_PIN);
    if (isnan(val))
    {
      trc(F("Failed to read from ADC !"));
    }
    else
    {
      if (val >= persistedadc + ThresholdReadingADC || val <= persistedadc - ThresholdReadingADC)
      {
        trc(F("Creating ADC buffer"));
        const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(1);
        StaticJsonBuffer<JSON_MSG_CALC_BUFFER> jsonBuffer;
        JsonObject &ADCdata = jsonBuffer.createObject();
        ADCdata.set("adc", (int)val);
        pub(ADCTOPIC, ADCdata);
        persistedadc = val;
      }
    }
  }
}
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorBME280.ino"
# 40 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorBME280.ino"
#include "User_config.h"

#ifdef ZsensorBME280
#include "Wire.h"
#include <stdint.h>
#include "SparkFunBME280.h"


BME280 mySensor;

void setupZsensorBME280()
{
  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x76;
# 62 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorBME280.ino"
  mySensor.settings.runMode = 3;
# 74 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorBME280.ino"
  mySensor.settings.tStandby = 1;
# 83 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorBME280.ino"
  mySensor.settings.filter = 4;





  mySensor.settings.tempOverSample = 1;





  mySensor.settings.pressOverSample = 1;





  mySensor.settings.humidOverSample = 1;

  delay(10);
  Serial.print("Bosch BME280 Initialized - Result of .begin(): 0x");
  Serial.println(mySensor.begin(), HEX);
}

void MeasureTempHumAndPressure()
{

  if (millis() > (timebme280 + TimeBetweenReadingbme280))
  {

    timebme280 = millis();
    static float persisted_bme_tempc;
    static float persisted_bme_tempf;
    static float persisted_bme_hum;
    static float persisted_bme_pa;
    static float persisted_bme_altim;
    static float persisted_bme_altift;

    float BmeTempC = mySensor.readTempC();
    float BmeTempF = mySensor.readTempF();
    float BmeHum = mySensor.readFloatHumidity();
    float BmePa = mySensor.readFloatPressure();
    float BmeAltiM = mySensor.readFloatAltitudeMeters();
    float BmeAltiFt = mySensor.readFloatAltitudeFeet();


    if (isnan(BmeTempC) || isnan(BmeTempF) || isnan(BmeHum) || isnan(BmePa) || isnan(BmeAltiM) || isnan(BmeAltiFt))
    {
      trc(F("Failed to read from Weather Sensor BME280!"));
    }
    else
    {
      trc(F("Creating BME280 buffer"));
      StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
      JsonObject &BME280data = jsonBuffer.createObject();

      if (BmeTempC != persisted_bme_tempc || bme280_always)
      {
        BME280data.set("tempc", (float)BmeTempC);
      }
      else
      {
        trc(F("Same Degrees C don't send it"));
      }


      if (BmeTempF != persisted_bme_tempf || bme280_always)
      {
        BME280data.set("tempf", (float)BmeTempF);
      }
      else
      {
        trc(F("Same Degrees F don't send it"));
      }


      if (BmeHum != persisted_bme_hum || bme280_always)
      {
        BME280data.set("hum", (float)BmeHum);
      }
      else
      {
        trc(F("Same Humidity don't send it"));
      }


      if (BmePa != persisted_bme_pa || bme280_always)
      {
        BME280data.set("pa", (float)BmePa);
      }
      else
      {
        trc(F("Same Pressure don't send it"));
      }


      if (BmeAltiM != persisted_bme_altim || bme280_always)
      {
        trc(F("Sending Altitude Meter to MQTT"));
        BME280data.set("altim", (float)BmeAltiM);
      }
      else
      {
        trc(F("Same Altitude Meter don't send it"));
      }


      if (BmeAltiFt != persisted_bme_altift || bme280_always)
      {
        BME280data.set("altift", (float)BmeAltiFt);
      }
      else
      {
        trc(F("Same Altitude Feet don't send it"));
      }
      if (BME280data.size() > 0)
        pub(BMETOPIC, BME280data);
    }
    persisted_bme_tempc = BmeTempC;
    persisted_bme_tempf = BmeTempF;
    persisted_bme_hum = BmeHum;
    persisted_bme_pa = BmePa;
    persisted_bme_altim = BmeAltiM;
    persisted_bme_altift = BmeAltiFt;
  }
}

#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorDHT.ino"
# 30 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorDHT.ino"
#include "User_config.h"

#ifdef ZsensorDHT
#include <DHT.h>
#include <DHT_U.h>

DHT dht(DHT_RECEIVER_PIN, DHT_SENSOR_TYPE);


unsigned long timedht = 0;

void MeasureTempAndHum()
{
  if (millis() > (timedht + TimeBetweenReadingDHT))
  {
    timedht = millis();
    static float persistedh;
    static float persistedt;
    float h = dht.readHumidity();

    float t = dht.readTemperature();

    if (isnan(h) || isnan(t))
    {
      trc(F("Failed to read from DHT sensor!"));
    }
    else
    {
      trc(F("Creating DHT buffer"));
      StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
      JsonObject &DHTdata = jsonBuffer.createObject();
      if (h != persistedh || dht_always)
      {
        DHTdata.set("hum", (float)h);
      }
      else
      {
        trc(F("Same hum don't send it"));
      }
      if (t != persistedt || dht_always)
      {
        DHTdata.set("temp", (float)t);
      }
      else
      {
        trc(F("Same temp don't send it"));
      }
      if (DHTdata.size() > 0)
        pub(DHTTOPIC, DHTdata);
    }
    persistedh = h;
    persistedt = t;
  }
}
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorGPIOInput.ino"
# 29 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorGPIOInput.ino"
#include "User_config.h"

#ifdef ZsensorGPIOInput

unsigned long lastDebounceTime = 0;
int InputState = 3;
int lastInputState = 3;

void setupGPIOInput()
{
  pinMode(GPIOInput_PIN, INPUT_PULLUP);
}

void MeasureGPIOInput()
{
  int reading = digitalRead(GPIOInput_PIN);






  if (reading != lastInputState)
  {

    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > GPIOInputDebounceDelay)
  {


#if defined(ESP8266) || defined(ESP32)
    yield();
#endif

    if (reading != InputState)
    {
      InputState = reading;
      trc(F("Creating GPIOInput buffer"));
      const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(1);
      StaticJsonBuffer<JSON_MSG_CALC_BUFFER> jsonBuffer;
      JsonObject &GPIOdata = jsonBuffer.createObject();
      if (InputState == HIGH)
      {
        trc(F("GPIO HIGH"));
        GPIOdata.set("gpio", "HIGH");
      }
      if (InputState == LOW)
      {
        trc(F("GPIO LOW"));
        GPIOdata.set("gpio", "LOW");
      }
      if (GPIOdata.size() > 0)
        pub(subjectGPIOInputtoMQTT, GPIOdata);
    }
  }


  lastInputState = reading;
}
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorGPIOKeyCode.ino"
# 26 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorGPIOKeyCode.ino"
#include "User_config.h"

#ifdef ZsensorGPIOKeyCode

int InputStateGPIOKeyCode = 0x0f;
int lastInputStateGPIOKeyCode = 0x0f;
int lastLatchStateGPIOKeyCode = 0;

void setupGPIOKeyCode()
{
  pinMode(GPIOKeyCode_LATCH_PIN, INPUT_PULLUP);
  pinMode(GPIOKeyCode_D0_PIN, INPUT_PULLUP);
  pinMode(GPIOKeyCode_D1_PIN, INPUT_PULLUP);
  pinMode(GPIOKeyCode_D2_PIN, INPUT_PULLUP);

}

void MeasureGPIOKeyCode()
{

  int latch = digitalRead(GPIOKeyCode_LATCH_PIN);





  {


#if defined(ESP8266) || defined(ESP32)
    yield();
#endif

    if (latch > 0 && lastLatchStateGPIOKeyCode != latch)
    {
      int reading = digitalRead(GPIOKeyCode_D0_PIN) | (digitalRead(GPIOKeyCode_D1_PIN) << 1) | (digitalRead(GPIOKeyCode_D2_PIN) << 2);


      char hex[3];

      InputStateGPIOKeyCode = reading;
      sprintf(hex, "%02x", InputStateGPIOKeyCode);
      hex[2] = 0;
      Serial.printf("GPIOKeyCode %s\n", hex);
      pub(subjectGPIOKeyCodetoMQTT, hex);
      lastLatchStateGPIOKeyCode = latch;
    }

    if (latch != lastLatchStateGPIOKeyCode)
    {
      lastLatchStateGPIOKeyCode = latch;
      Serial.printf("GPIOKeyCode latch %d\n", latch);
      if (latch == 0)
        pub(subjectGPIOKeyCodeStatetoMQTT, "done");
    }


    lastInputStateGPIOKeyCode = InputStateGPIOKeyCode;
  }
}
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorHCSR04.ino"
# 30 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorHCSR04.ino"
#include "User_config.h"

#ifdef ZsensorHCSR04

unsigned long timeHCSR04 = 0;

void setupHCSR04()
{
  pinMode(HCSR04_TRI_PIN, OUTPUT);
  pinMode(HCSR04_ECH_PIN, INPUT);
}

void MeasureDistance()
{
  if (millis() > (timeHCSR04 + TimeBetweenReadingHCSR04))
  {
    timeHCSR04 = millis();
    trc(F("Creating HCSR04 buffer"));
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &HCSR04data = jsonBuffer.createObject();
    digitalWrite(HCSR04_TRI_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(HCSR04_TRI_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(HCSR04_TRI_PIN, LOW);
    unsigned long duration = pulseIn(HCSR04_ECH_PIN, HIGH);
    if (isnan(duration))
    {
      trc(F("Failed to read from HC SR04 sensor!"));
    }
    else
    {
      static unsigned int distance = 99999;
      unsigned int d = duration / 58.2;
      HCSR04data.set("distance", (int)d);
      if (d > distance)
      {
        HCSR04data.set("direction", "away");
        trc(F("HC SR04 Distance changed"));
      }
      else if (d < distance)
      {
        HCSR04data.set("direction", "towards");
        trc(F("HC SR04 Distance changed"));
      }
      else if (HCSR04_always)
      {
        HCSR04data.set("direction", "static");
        trc(F("HC SR04 Distance hasn't changed"));
      }
      distance = d;
      if (HCSR04data.size() > 0)
        pub(subjectHCSR04, HCSR04data);
    }
  }
}
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorHCSR501.ino"
# 29 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorHCSR501.ino"
#include "User_config.h"

#ifdef ZsensorHCSR501

void setupHCSR501()
{
  pinMode(HCSR501_PIN, INPUT);
}

void MeasureHCSR501()
{
  if (millis() > TimeBeforeStartHCSR501)
  {
    const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(1);
    StaticJsonBuffer<JSON_MSG_CALC_BUFFER> jsonBuffer;
    JsonObject &HCSR501data = jsonBuffer.createObject();
    static int pirState = LOW;
    int PresenceValue = digitalRead(HCSR501_PIN);
    #if defined(ESP8266) || defined(ESP32)
    yield();
    #endif
    if (PresenceValue == HIGH)
    {
      if (pirState == LOW)
      {

        HCSR501data.set("hcsr501", "true");
        trc(F("HC SR501 Motion started"));
        pirState = HIGH;
      }
    }
    else
    {
      if (pirState == HIGH)
      {

        HCSR501data.set("hcsr501", "false");
        trc(F("HC SR501 Motion ended"));
        pirState = LOW;
      }
    }
    if (HCSR501data.size() > 0)
      pub(subjectHCSR501toMQTT, HCSR501data);
  }
}
#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorINA226.ino"
# 35 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorINA226.ino"
#include "User_config.h"

#ifdef ZsensorINA226
#include <Wire.h>

float rShunt = 0.1;
const int INA226_ADDR = 0x40;


unsigned long timeINA226 = 0;

void setupINA226()
{
  Wire.begin();

  writeRegister(0x00, 0x4427);
}

void MeasureINA226()
{
  if (millis() > (timeINA226 + TimeBetweenReadingINA226))
  {
    timeINA226 = millis();
    trc(F("Creating INA226 buffer"));
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &INA226data = jsonBuffer.createObject();

    trc(F("Retrieving electrical data"));

    trc(F(" Volt: "));
    float volt = readRegister(0x02) * 0.00125;
    trc(volt);
    trc(F(" V, Current: "));

    int shuntvolt = readRegister(0x01);
    if (shuntvolt && 0x8000)
    {
      shuntvolt = ~shuntvolt;
      shuntvolt += 1;
      shuntvolt *= -1;
    }
    float current = shuntvolt * 0.0000025 / rShunt;
    trc(current);
    trc(F(" A, Power: "));
    float power = abs(volt * current);
    trc(power);
    trc(F(" W"));

    char volt_c[7];
    char current_c[7];
    char power_c[7];
    dtostrf(volt, 6, 3, volt_c);
    dtostrf(current, 6, 3, current_c);
    dtostrf(power, 6, 3, power_c);
    INA226data.set("volt", (char *)volt_c);
    INA226data.set("current", (char *)current_c);
    INA226data.set("power", (char *)power_c);
    pub(subjectINA226toMQTT, INA226data);
  }
}

static void writeRegister(byte reg, word value)
{
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

static word readRegister(byte reg)
{
  word res = 0x0000;
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() == 0)
  {
    if (Wire.requestFrom(INA226_ADDR, 2) >= 2)
    {
      res = Wire.read() * 256;
      res += Wire.read();
    }
  }
  return res;
}

#endif
# 1 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorTSL2561.ino"
# 40 "/Volumes/Daten/Users/matthiasberner/github/OpenMQTTGateway/main/ZsensorTSL2561.ino"
#include "User_config.h"

#ifdef ZsensorTSL2561
#include "math.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  trc(F("------------------------------------"));
  trc("Sensor:       " + String(sensor.name));
  trc("Driver Ver:   " + String(sensor.version));
  trc("Unique ID:    " + String(sensor.sensor_id));
  trc("Max Value:    " + String(sensor.max_value) + " lux");
  trc("Min Value:    " + String(sensor.min_value) + " lux");
  trc("Resolution:   " + String(sensor.resolution) + " lux");
  trc(F("------------------------------------"));
  trc(F(""));
  delay(500);
}

void setupZsensorTSL2561()
{
  Wire.begin();
  Wire.beginTransmission(TSL2561_ADDR_FLOAT);

  if (!tsl.begin())
  {
    trc(F("No TSL2561 detected"));
  }




  tsl.enableAutoRange(true);



  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);

  trc(F("TSL2561 Initialized. Printing detials now."));
  displaySensorDetails();
}

void MeasureLightIntensityTSL2561()
{
  if (millis() > (timetsl2561 + TimeBetweenReadingtsl2561))
  {
    static uint32_t persisted_lux;
    timetsl2561 = millis();

    trc(F("Creating TSL2561 buffer"));
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &TSL2561data = jsonBuffer.createObject();

    sensors_event_t event;
    tsl.getEvent(&event);
    if (event.light)

    {
      if (persisted_lux != event.light || tsl2561_always)
      {
        persisted_lux = event.light;

        TSL2561data.set("lux", (float)event.light);
        TSL2561data.set("ftcd", (float)(event.light) / 10.764);
        TSL2561data.set("wattsm2", (float)(event.light) / 683.0);

        pub(subjectTSL12561toMQTT, TSL2561data);
      }
      else
      {
        trc(F("Same lux value, do not send"));
      }
    }
    else
    {
      trc(F("Failed to read from TSL2561"));
    }
  }
}
#endif