#include <Arduino.h>
#include <ESP8266WiFi.h> 
#include <PubSubClient.h> 
#include <Wire.h>
#include "Adafruit_Si7021.h"
#include <ArduinoJson.h>
#include <OneWire.h>
#include <EEPROM.h>

#include "phSensor.h"
#include "waterLevelSensor.h"
#include "ecMeter.h"
#include "config.h"

OneWire  ds(D4);
Adafruit_Si7021 sensor = Adafruit_Si7021();
PHSensor phSensor = PHSensor();
WaterLevelSensor waterLevelSensor = WaterLevelSensor();
ECMeter ecMeter = ECMeter();

const char* ssid = "Chalk-wifi";
int16_t deviceID = 0;
const int sleepTimeSec = 600; // 10 minutes

const char* mqtt_server = "water.local";
const char* mqtt_ambient_temperature_topic = "/state/ambient_temperature";
const char* mqtt_ambient_humidity_topic = "/state/ambient_humidity";
const char* mqtt_water_temperature_topic = "/state/water_temperature";
const char* mqtt_water_level_topic = "/state/water_level";
const char* mqtt_water_ec_topic = "/state/water_ec";
const char* mqtt_pH_topic = "/state/pH";
const char* mqtt_username = "";
const char* mqtt_password = "";

#ifdef AMBIENT_TEMP
const char* clientID = "Ambient Temp Sensor";
const int ipAddress = 99;
#elif EC_METER
const char* clientID = "EC Meter";
const int ipAddress = 98;
#elif WATER_TEMP
const char* clientID = "Water Temperature Sensor";
const int ipAddress = 97;
#endif

WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient);

float waterTemperature = 0.0;

void waterTemperatureCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  const int capacity = JSON_OBJECT_SIZE(3);
  StaticJsonDocument<capacity> waterTemperatureJSON;
  deserializeJson(waterTemperatureJSON, payload);
  waterTemperature = waterTemperatureJSON["temperature"].as<float>();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientID)) {
      Serial.println("connected");
      #if EC_METER
      client.subscribe(mqtt_water_temperature_topic);
      #endif
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setupAmbientTempAndHumidity() {
  Serial.println("Si7021 test!");
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true);
  }
  Serial.print("Found model ");
  switch(sensor.getModel()) {
    case SI_Engineering_Samples:
      Serial.print("SI engineering samples"); break;
    case SI_7013:
      Serial.print("Si7013"); break;
    case SI_7020:
      Serial.print("Si7020"); break;
    case SI_7021:
      Serial.print("Si7021"); break;
    case SI_UNKNOWN:
    default:
      Serial.print("Unknown");
  }
  Serial.print(" Rev(");
  Serial.print(sensor.getRevision());
  Serial.print(")");
  Serial.print(" Serial #"); Serial.print(sensor.sernum_a, HEX); Serial.println(sensor.sernum_b, HEX);
}

void publishState(float state, char* label, const char* topic) {
  const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> stateJson;
    stateJson[label] = state;
    char stateJsonCStr[128];
    serializeJson(stateJson, stateJsonCStr);
    if (client.publish(topic, stateJsonCStr)) {
      Serial.println("State measured and message sent");
    } else {
      Serial.println("Message failed to send via mqtt");
      reconnect();
      client.publish(topic, stateJsonCStr);
    }
}

void getDeviceId() {
  byte firstSection = EEPROM.read(0);
  byte secondSection = EEPROM.read(1);

  deviceID = (int16_t)secondSection << 8 | (int16_t)firstSection;
}

void addDeviceID() {
  long firstPartID = random(255);
  long secondPartID = random(255);
  EEPROM.write(0, (uint8_t)firstPartID);
  EEPROM.write(1, (uint8_t)firstPartID);
  Serial.print("Sensor ID: "); Serial.print(firstPartID); Serial.print(secondPartID);  
}


void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  randomSeed(analogRead(0));

  while (! Serial) {
    delay(1);
  }

  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, WIFI_PASSWORD);
  IPAddress ip(192,168,0,ipAddress);   
  IPAddress gateway(192,168,0,1);   
  IPAddress subnet(255,255,255,255);   
  WiFi.config(ip, gateway, subnet);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  getDeviceId();
  if (deviceID == 0) {
    addDeviceID();
  }

  Serial.print(deviceID);
  
  #ifdef AMBIENT_TEMP
    setupAmbientTempAndHumidity();
    float humidity = sensor.readHumidity();
    float temperature = sensor.readTemperature();

    Serial.print("Humidity:    ");
    Serial.print(humidity, 2);
    Serial.print("\tTemperature: ");
    Serial.println(temperature, 2);
    publishState(temperature, "temperature", mqtt_ambient_temperature_topic);
    publishState(humidity, "humidity", mqtt_ambient_humidity_topic);
  #endif

  #ifdef WATER_LEVEL
    waterLevelSensor.Setup();
    float waterLevel = waterLevelSensor.GetReading();
    Serial.print("water level:"); Serial.print(waterLevel, 2); Serial.println("mm");
    publishState(waterLevel, "waterLevel", mqtt_water_level_topic);
  #endif

  #ifdef PH_SENSOR
    phSensor.Setup();
    float ph = phSensor.GetReading();
    publishState(ph, "pH", mqtt_pH_topic);
  #endif

  #ifdef EC_METER
    ecMeter.Setup();
    client.setCallback(waterTemperatureCallback);
    float ecLevel;
    Serial.println(waterTemperature);
    while(waterTemperature == 0.0){
      delay(500);
      if (!client.connected()){
        reconnect();
      } else {
        client.loop();
      }
    }
    ecLevel = ecMeter.GetReading(waterTemperature);
    publishState(ecLevel, "ecLevel", mqtt_water_ec_topic);
  #endif

  #ifdef WATER_TEMP
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];

    if ( !ds.search(addr)) 
    {
      ds.reset_search();
      delay(250);
      return;
    }
  
  
    if (OneWire::crc8(addr, 7) != addr[7]) 
    {
        Serial.println("CRC is not valid!");
        return;
    }
    Serial.println();
  
    switch (addr[0]) 
    {
      case 0x10:
        type_s = 1;
        break;
      case 0x28:
        type_s = 0;
        break;
      case 0x22:
        type_s = 0;
        break;
      default:
        Serial.println("Device is not a DS18x20 family device.");
        return;
    } 
  
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        
    delay(1000);
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         
  
    for ( i = 0; i < 9; i++) 
    {           
      data[i] = ds.read();
    }
  
    int16_t rawTemperature = (data[1] << 8) | data[0];
    if (type_s) {
      rawTemperature = rawTemperature << 3; // 9 bit resolution default
      if (data[7] == 0x10) 
      {
        rawTemperature = (rawTemperature & 0xFFF0) + 12 - data[6];
      }
    } 
    else 
    {
      byte cfg = (data[4] & 0x60);
      if (cfg == 0x00) rawTemperature = rawTemperature & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) rawTemperature = rawTemperature & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) rawTemperature = rawTemperature & ~1; // 11 bit res, 375 ms
  
    }
    float waterTemperatureCelcius = (float)rawTemperature / 16.0;
    Serial.print("  Temperature = ");
    Serial.print(waterTemperatureCelcius);
    Serial.print(" Celsius, ");
    publishState(waterTemperatureCelcius, "temperature", mqtt_water_temperature_topic);
  #endif

  //Sleep
   Serial.println("ESP8266 in sleep mode");
   ESP.deepSleep(sleepTimeSec * 1000000); 
}

void loop() {
}