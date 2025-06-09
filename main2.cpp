//////////////////////////////
///////////ESP-2/////////////
////////////////////////////

#include <Arduino.h>
#include <string.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <i2cdetect.h>

// Replace the next variables with your SSID/Password combination
const char* ssid = "Redmi Note 10 Pro";
const char* password = "00000000";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.62.246";
const char* mqttUser = "mqttbroker";
const char* mqttPassword = "mqttbroker";

const int loopTime = 15000;  //time to loop in miliseconds
// LED Pin
const int ledPin = 4;
const int buttonPin = 19;

BH1750 lightMeter;
// Data wire is conntec to the Arduino digital pin 5
#define ONE_WIRE_BUS 5

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

bool isOff = true;
float temperature = 0;
float humidity = 0;
int buttonState = 0;




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
    if (client.connect("ESP32Client2", mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32-2/light/state");
      client.subscribe("esp32-2/light/brightness");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
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
  
  if (strcmp(topic, "esp32-2/light/brightness") == 0) {
    int brightness = message.toInt();
    analogWrite(ledPin, brightness);
    Serial.print("Set brightness to ");
    Serial.println(brightness);
  } 
  else if (strcmp(topic, "esp32-2/light/state") == 0) {    
    if (message == "ON" && isOff) {
      isOff = false;
      client.publish("esp32-2/light/brightness", "255");
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


void setup() {
  Serial.begin(9600);
  sensors.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  Wire.begin(21, 22);
  Serial.println("i2cdetect example\n");
  Serial.print("Scanning address range 0x03-0x77\n\n");
  Serial.println(F("BH1750 Test begin"));
  i2cdetect();  // default range from 0x03 to 0x77
  lightMeter.begin();
  delay(2000);

  
}

void loop() {
  buttonState = digitalRead(buttonPin);
  //Serial.println(buttonState);
  if (buttonState == HIGH) {
    client.publish("esp32-2/button/state", "pressed");
    Serial.println("Button pressed");
    delay(300);
    client.publish("esp32-2/button/state", "released");
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  sensors.requestTemperatures();

  
  long now = millis();
  if (now - lastMsg > loopTime) {
    lastMsg = now;

    i2cdetect();  // default range from 0x03 to 0x77
    float lux = lightMeter.readLightLevel();
    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lx");
    
    char luxString[8];
    dtostrf(lux, 1, 2, luxString);
    client.publish("esp32-2/lux", luxString);

    // Temperature in Celsius
    temperature = sensors.getTempCByIndex(0);
    
    // Convert the value to a char array
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32-2/temperature", tempString);
  }
}

