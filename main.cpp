//////////////////////////////
///////////ESP-1/////////////
////////////////////////////

#include <Arduino.h>
#include <string.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "DHT.h"

// Replace the next variables with your SSID/Password combination
const char* ssid = "Redmi Note 10 Pro";
const char* password = "00000000";

// Add your MQTT Broker IP address
const char* mqtt_server = "192.168.130.246";
const char* mqttUser = "mqttbroker";
const char* mqttPassword = "mqttbroker";


#define ONE_WIRE_BUS 26
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DHT dht(12, DHT11);

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

float temperature = 0;
float humidity = 0;
int buttonState = 0;
bool isOff = true;

// LED Pin
int ledPinGreen = 14;
int ledPinRed = 25;
int buttonPin = 35;

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client1", mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/light1/state");
      client.subscribe("esp32/light1/brightness");
      client.subscribe("esp32/light2/state");
      client.subscribe("esp32/light2/brightness");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void dhtRead(float h, float t){

  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
}


void ledLights(int ledPin, char* topic, String message, const char* brightnessStr, const char* stateStr){
  if (strcmp(topic, brightnessStr) == 0) {
    int brightness = message.toInt();
    analogWrite(ledPin, brightness);
    Serial.print("Set brightness to ");
    Serial.println(brightness);
  } 
  else if (strcmp(topic, stateStr) == 0) {    
    if (message == "ON" && isOff) {
      isOff = false;
      client.publish(brightnessStr, "255");
      analogWrite(ledPin, 255);
      Serial.println("Turned LED on");
    } 
    else if (message == "OFF") {
      isOff = true;
      analogWrite(ledPin, 0);
      Serial.println("Turned LED off");
    }    
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message received on topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);

  ledLights(ledPinGreen, topic, message, "esp32/light1/brightness", "esp32/light1/state");
  ledLights(ledPinRed, topic, message, "esp32/light2/brightness", "esp32/light2/state");
}


void setup() {
  Serial.begin(115200);
  sensors.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Wire.begin();
  dht.begin();

  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinRed, OUTPUT);
  pinMode(buttonPin, INPUT);
}

void loop() {
  if (!client.connected()) {
      reconnect();
    }
  client.loop();

  buttonState = digitalRead(buttonPin);
  //Serial.println(buttonState);
  if (buttonState == HIGH) {
    client.publish("esp32/button/state", "pressed");
    Serial.println("Button pressed");
    delay(100);
    client.publish("esp32/button/state", "released");
  }
  
  long now = millis();
  if (now - lastMsg > 20000) {
    lastMsg = now;
    sensors.requestTemperatures();

    float h = dht.readHumidity();
    float t = dht.readTemperature();    
    
    // Temperature in Celsius
    temperature = sensors.getTempCByIndex(0);
    
    // Convert the value to a char array
    char tempString[8];
    char humString[8];
    dtostrf(temperature, 1, 2, tempString);
    dtostrf(h, 1, 2, humString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    if (temperature != -127){
      client.publish("esp32/temperature/temp1", tempString);
    }
    client.publish("esp32/humidity/hum1", humString);
    dtostrf(t, 1, 2, tempString);
    client.publish("esp32/temperature/temp2", tempString);
    
    dhtRead(h, t);

  }
}