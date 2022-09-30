#include <esp_now.h>
#include <WiFi.h>
#include <MQTT.h>
#include <PubSubClient.h>
#include "EEPROM.h"
#include <OneWire.h>
#include <DFRobot_PH.h>

DFRobot_PH ph;
// DFRobot_ESP_PH ph;
#define ESPADC      4096.0    //the esp Analog Digital Convertion value
#define ESPVOLTAGE  3300      //the esp voltage supply value
#define PH_PIN      25        //34 //the esp gpio data pin number
float voltage, phValue, watertemp, sensorturbidity;
int turbidity = analogRead(34);

int DS18S20_Pin = 23;      //DS18S20 Signal pin on digital 2
//Temperature chip i/o
OneWire ds(DS18S20_Pin);   // on digital pin 2

const char ssid[] = "KingAPK";
const char pass[] = "AisyahAR";

WiFiClient net;
MQTTClient client;

unsigned long lastMillis = 0;

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("GATEWAY")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("/hello");
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(32);//needed to permit storage of calibration value in eeprom
  ph.begin();

  Serial.begin(115200);
  WiFi.begin(ssid, pass);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin("test.mosquitto.org", net);
  client.onMessage(messageReceived);

  connect();
 
}

void loop() 
{
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) //time interval: 1s
  {
    timepoint = millis();
    //voltage = rawPinValue / esp32ADC * esp32Vin
    voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
    Serial.print("Voltage:");
    Serial.println(voltage/1000, 2);
    
    // //temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
    // temperature = getTemp();
    // Serial.print("Temperature:");
    // Serial.print(temperature, 2);
    // Serial.println("^C");

    phValue = ph.readPH(voltage, watertemp); // convert voltage to pH with temperature compensation
    Serial.print("pH Value:");
    Serial.println(phValue, 2);
    

    sensorturbidity = turbidity * (3.3 / 1024.0);
    Serial.print("Turbidity:");
    Serial.println(sensorturbidity);
    
    watertemp = getTemp();
    Serial.print("Water Temperature:");
    Serial.println(watertemp);
    Serial.println();
  }

  ph.calibration(voltage, watertemp); // calibration process by Serail CMD

  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  // publish a message roughly every second.
  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
    String 
    jsonData = "{"; 
    jsonData += "\"Voltage\":" + (String)(voltage/1000, 2) + ",";
    jsonData += "\"pH\":" + (String)phValue + ",";
    jsonData += "\"Turbidity\":" + (String)sensorturbidity + ",";
    jsonData += "\"WaterTemp\":" + (String)watertemp;
    jsonData += "}";
    
    Serial.println("Sensor Node JSON Payload > " + jsonData + "\n");
    client.publish("KingAPK-FinalProject/DATA", jsonData);
  }
}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  return TemperatureSum;
}
