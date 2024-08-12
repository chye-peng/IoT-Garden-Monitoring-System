// Teo Chye Peng - TP065441
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

// WiFi
const char *ssid = "cp";     // Enter your WiFi name
const char *password = "chyepeng"; // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "broker.emqx.io";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

const char *humidity_topic = "diota_ast/humidity";
const char *temperature_topic = "diota_ast/temperature";
const char *rain_value_topic = "diota_ast/rain_value";
const char *light_intensity_topic = "diota_ast/light_intensity";
const char *soil_moisture_topic = "diota_ast/soil_moisture";
const char *ph_value_topic = "diota_ast/ph_value";
const char *nitrogen_topic = "diota_ast/nitrogen";
const char *phosphorous_topic = "diota_ast/phosphorous";
const char *potassium_topic = "diota_ast/potassium";


// DHT Sensor
#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

#define POWER_PIN 32  // ESP32's pin GPIO32 that provides the power to the rain sensor
#define AO_PIN 35     // ESP32's pin GPIO35 connected to AO pin of the rain sensor

#define ledPin 5 // GPIO 5 (D5) for LED
#define LIGHT_SENSOR_PIN 39 // GPIO 39 (ADC3) for LDR

const int soilMoisturePin = 36; // GPIO 36 for Soil Moisture Sensor

#define PH_OFFSET -1.00 // Offset if needed
#define SensorPin 34 // GPIO 34

#define RE 33  //npk sensor
#define DE 25  

// Modbus RTU requests for reading NPK values remain the same
const byte nitro[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[] = {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};

unsigned long int avgValue; // Store the average value of the sensor feedback
float b;
int buf[10], temp;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(9600);  // Debug serial port

  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  
  pinMode(2, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(POWER_PIN, OUTPUT); 
  
  Serial.println(F("Garden Monitoring System Initiated"));
  
  dht.begin();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  // Connect to MQTT Broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public EMQX MQTT broker connected");
    } else {
      Serial.print("Failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void loop() {
  // Read DHT sensor data
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {
    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.println(F("°C "));
    
    // Publish humidity and temperature to MQTT
    char humStr[6];
    dtostrf(h, 4, 2, humStr);
    char tempStr[6];
    dtostrf(t, 4, 2, tempStr);
    client.publish(humidity_topic, humStr);
    client.publish(temperature_topic, tempStr);  
  } else {
    Serial.println(F("Failed to read from DHT sensor!"));
  }

  // Read rain sensor data
  digitalWrite(POWER_PIN, HIGH);  
  delay(10);                      

  int rain_value = analogRead(AO_PIN);
  digitalWrite(POWER_PIN, LOW);  // turn the rain sensor's power OFF
  //the more water is detected, the low the value read from rain sensor
  Serial.print("rain value: ");
  Serial.println(rain_value); 

  // Publish rain value to MQTT
  char rainStr[6];
  dtostrf(rain_value, 4, 2, rainStr);
  client.publish(rain_value_topic , rainStr);

  // Read LDR sensor data
  int ldrAnalogValue = analogRead(LIGHT_SENSOR_PIN);
  Serial.print("Analog Value of LDR: ");
  Serial.print(ldrAnalogValue);
  Serial.println(" cd");  
  digitalWrite(ledPin, ldrAnalogValue < 800 ? HIGH : LOW);

  // Publish light intensity content to MQTT
  char ldrStr[6];
  dtostrf(ldrAnalogValue, 4, 2, ldrStr);
  client.publish(light_intensity_topic , ldrStr);

  // Read Soil Moisture sensor data
  int soilMoistureValue = analogRead(soilMoisturePin);
  float moisture_percentage = (100.00 - ((soilMoistureValue / 4095.00) * 100
    ));
  Serial.print("Soil Moisture: ");
  Serial.print(moisture_percentage);
  Serial.println("%");

  // Publish soil moisture content to MQTT
  char soilMoistureStr[6];
  dtostrf(moisture_percentage, 4, 2, soilMoistureStr);
  client.publish(soil_moisture_topic, soilMoistureStr);

  // Read pH sensor data
  for (int i = 0; i < 10; i++) {
    buf[i] = analogRead(SensorPin);
    delay(10);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;
  for (int i = 2; i < 8; i++)
    avgValue += buf[i];
  float phValue = (float)avgValue * 3.3 / 4095 / 6;
  phValue = 3.5 * phValue;
  phValue = phValue + PH_OFFSET;
  Serial.print("pH:");
  Serial.println(phValue, 2);
  digitalWrite(2, HIGH);
  delay(800);
  digitalWrite(2, LOW);

  // Publish ph value content to MQTT
  char phStr[6];
  dtostrf(phValue, 4, 2, phStr);
  client.publish(ph_value_topic, phStr);

  // Read NPK sensor data
  byte val1, val2, val3;
  val1 = nitrogen();
  delay(250);
  val2 = phosphorous();
  delay(250);
  val3 = potassium();

  Serial.print("Nitrogen: ");
  Serial.print(val1);
  Serial.println(" mg/kg");
  Serial.print("Phosphorous: ");
  Serial.print(val2);
  Serial.println(" mg/kg");
  Serial.print("Potassium: ");
  Serial.print(val3);
  Serial.println(" mg/kg");
  Serial.println("************************************");
  
  // Publish NPK sensor data to MQTT
  client.publish(nitrogen_topic, String(val1).c_str());
  client.publish(phosphorous_topic, String(val2).c_str());
  client.publish(potassium_topic , String(val3).c_str());

  delay(2000); // Delay before next loop iteration
}

byte nitrogen() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);
  if (Serial2.write(nitro, sizeof(nitro)) == 8) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    delay(10); // Allow time for response
    for (byte i = 0; i < 7 && Serial2.available(); i++) {
      Serial.print(Serial2.read(), HEX);
    }
    Serial.println();
  }
  return Serial2.read();
}

byte phosphorous() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);
  if (Serial2.write(phos, sizeof(phos)) == 8) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    delay(10); // Allow time for response
    for (byte i = 0; i < 7 && Serial2.available(); i++) {
      Serial.print(Serial2.read(), HEX);
    }
    Serial.println();
  }
  return Serial2.read();
}

byte potassium() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);
  if (Serial2.write(pota, sizeof(pota)) == 8) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    delay(10); // Allow time for response
    for (byte i = 0; i < 7 && Serial2.available(); i++) {
      Serial.print(Serial2.read(), HEX);
    }
    Serial.println();
  }
  return Serial2.read();
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}
