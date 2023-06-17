#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>

#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include "DHTesp.h"




// #include <Adafruit_BME280.h>
// #include <Adafruit_Sensor.h>

// Replace with your network credentials
const char* ssid = "Huawei";
const char* password = "qornainaji";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "172.20.12.166";

WiFiClient espClient;
PubSubClient client(espClient);


// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 500;

// Create a sensor object
// Adafruit_BME280 bme; // BME280 connect to ESP32 I2C (GPIO 21 = SDA, GPIO 22 = SCL)

// Init BME280
// void initBME(){
//   if (!bme.begin(0x76)) {
//     Serial.println("Could not find a valid BME280 sensor, check wiring!");
//     while (1);
//   }
// }

const int DHT_PIN = 15;
const int potpin = 34; 
const int butright = 26;
const int butleft = 27;

int value = 0;
int press = 90;

DHTesp dhtSensor;

// void initBME(){
//   if (!bme.begin(0x76)) {
//     Serial.println("Could not find a valid BME280 sensor, check wiring!");
//     while (1);
//   }
// }

// Get Sensor Readings and return JSON object
String getSensorReadings(){
  TempAndHumidity  data = dhtSensor.getTempAndHumidity();
  readings["temperature"] = String(data.temperature, 2);
  readings["humidity"] =  String(data.humidity, 1);
  readings["potentiometer"] = String(value);
  readings["tiltmeter"] = String(press);
  return readings;
}

String converttoJson(){
  readings = getSensorReadings()
  String jsonString = JSON.stringify(readings)
  return jsonString
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(butright, INPUT_PULLUP);
  pinMode(butleft, INPUT_PULLUP);
  initWiFi();
  initSPIFFS();

  // Init mqtt connection
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  // Request for the latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = converttoJson();
    request->send(200, "application/json", json);
    json = String();
  });

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  // Start server
  server.begin();
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if ((millis() - lastTime) > timerDelay) {
    // Send Events to the client with the Sensor Readings Every 1/2 seconds
    value = analogRead(potpin);
    // bool right = digitalRead(butright);
    // bool left = digitalRead(butleft);
    
    // if buttons are press
    // if (!right)
    // {
    //   press--;
    //   Serial.println("right");
    //   Serial.println(press+" Degree");
    // }

    // if (!left)
    // {
    //   press++;
    //   Serial.println("right");
    //   Serial.println(press+" Degree");
    // }

    // if (press<0)
    // {
    //    press = 0;
    // }

    // if (press>180)
    // {
    //   press = 180;
    // }

    if (digitalRead(butright) == LOW && digitalRead(butleft) == HIGH)
    {
      press++;
      Serial.println("right");
    }
    if (digitalRead(butright) == HIGH && digitalRead(butleft) == LOW)
    {
      press--;
      Serial.println("left");
    }

    if (press<0)
    {
       press = 0;
    }

    if (press>180)
    {
      press = 180;
    }
    Serial.println(press);
    events.send("ping",NULL,millis());

    readings = getSensorReadings()
    String jsonString = JSON.stringify(readings)

    events.send(jsonString.c_str(),"new_readings" ,millis());


    readings["temperature"] = String(data.temperature, 2);
    readings["humidity"] =  String(data.humidity, 1);
    readings["potentiometer"] = String(value);
    readings["tiltmeter"] = String(press); 
    

    Serial.print("Temperature: ");
    Serial.println(readings["temperature"]);
    client.publish("esp32/temperature", readings["temperature"]);

    Serial.print("Humidity: ");
    Serial.println(readings["humidity"]);
    client.publish("esp32/humidity", readings["humidity"]);

    Serial.print("Potentiometer: ");
    Serial.println(readings["potentiometer"]);
    client.publish("esp32/potentiometer", readings["potentiometer"]);

    Serial.print("Titlmeter: ");
    Serial.println(readings["titlmeter"]);
    client.publish("esp32/titlmeter", readings["titlmeter"]);


    lastTime = millis();
  }
}
