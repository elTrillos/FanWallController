#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int eepromSize = 64;  // Adjust size as needed

const char* ssid = "Trollis";
const char* password = "123456789a";

const char* mqttBroker = "broker.hivemq.com";
const char* mqttUsername = "your_mqtt_username";  // If required
const char* mqttPassword = "your_mqtt_password";  // If required

const char *selfTopic = "fanWall/wall/";
const char *topic1 = "fanWall/wall/control";
const char *topic2 = "fanWall/wall/status";
const char *topic3 = "fanWall/wall/id";
const int mqttPort = 1883;

float currentSpeed = 1000.0;
int pollingFrequency = 2000;
int currpower = 0;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
TaskHandle_t fanControlTask;
TaskHandle_t mqttTask;

char storedSSID[eepromSize];
char storedPassword[eepromSize];

void writeCredentials(const char* ssid, const char* password) {
    EEPROM.begin(eepromSize);
    EEPROM.writeString(0, ssid);
    EEPROM.writeString(strlen(ssid) + 1, password);
    EEPROM.commit();
    EEPROM.end();
}

void readCredentials() {
    EEPROM.begin(eepromSize);
    EEPROM.readString(0, storedSSID, eepromSize);
    EEPROM.readString(strlen(storedSSID) + 1, storedPassword, eepromSize);
    EEPROM.end();
}

void setupWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    selfTopic = strdup((String(selfTopic)+WiFi.macAddress()).c_str());
    Serial.printf(selfTopic);
}

void connectMQTT() {
    mqttClient.setServer(mqttBroker, mqttPort);
    mqttClient.setCallback(messageReceivedCallback);
    while (!mqttClient.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (mqttClient.connect(client_id.c_str(), mqttUsername, mqttPassword)) {
            Serial.println("Public EMQX MQTT broker connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(mqttClient.state());
            delay(2000);
        }
    }
    // Publish and subscribe
    mqttClient.publish(topic1, "Hi, I'm ESP32 ^^");
    
    mqttClient.subscribe(selfTopic);
    mqttClient.subscribe(topic1);
    mqttClient.subscribe(topic3);
}

void fanControlFunction(void *parameter) {
    while (true) {
        // Implement fan control logic here
        delay(1000);  // Adjust delay as needed
        Serial.println("Thread 1");
    }
}

void mqttLoopTask(void *parameter) {
    while (true) {
        if (mqttClient.connected()) {
            mqttClient.loop();
        } else {
          Serial.println("disconnected");
            connectMQTT();  // Reconnect if MQTT connection is lost
        }
        Serial.println(pollingFrequency);
        delay(pollingFrequency);
    }
}

void setSpeeds(int targetSpeed){
  if(targetSpeed!=100){
    for(int fan = 0 ; fan<16;fan++){
      pwm.setPWM(fan,0,targetSpeed*40);
    
    }
  }
  else{
    for(int fan = 0 ; fan<16;fan++){
      pwm.setPWM(fan,0,4096);
    }
  }
  
}


void setup() {
    Serial.begin(9600);
    pwm.begin();

    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(250);  // This is the maximum PWM frequency
    // Connect to WiFi and MQTT
    Wire.setClock(400000);
    pwm.setPWM(0,4096,0);
    setupWiFi();
    connectMQTT();
    mqttClient.setCallback(messageReceivedCallback);
    // Create task for fan control
    xTaskCreatePinnedToCore(
        fanControlFunction,
        "fanControlTask",
        10000,
        NULL,
        1,
        &fanControlTask,
        1  // Assign fan control task to core 1 (0-indexed)
    );

    // Create task for MQTT client loop
    xTaskCreatePinnedToCore(
        mqttLoopTask,
        "mqttTask",
        10000,
        NULL,
        1,
        &mqttTask,
        0  // Assign MQTT client loop task to core 0
    );
}

void loop() {
    delay(1000);
}
void messageReceivedCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message received on topic: ");
    Serial.println(topic);
    Serial.print("Payload: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    String message = "";
      for (int i = 0; i < length; i++) {
          message += (char)payload[i];
      }
      message.trim();
    // Check if the message is from topic2 and contains "start" or "cancel"
    if (strcmp(topic, topic1) == 0) {
        if (message.equals("start")) {
            Serial.println("Received start command from topic2");
            pollingFrequency=500;
        } else if (message.equals("cancel")) {
            Serial.println("Received cancel command from topic2");
            pollingFrequency=2000;
        }
        else{
          Serial.println("setting speed to");
          currpower+=125;
          Serial.println(currpower);
          pwm.setPWM(0, 0, currpower);
          pwm.setPWM(1, 0, currpower);
          pwm.setPWM(2, 0, currpower);
          pwm.setPWM(3, 0, currpower);
        }
    }
    else if (strcmp(topic, topic2) == 0) {
        // Convert payload to a String
        int delimiterIndex = message.indexOf('/');
        if (delimiterIndex != -1) {
            // Extract the first part of the message (before ':')
            String id = message.substring(0, delimiterIndex);
            String speedToSet = message.substring(delimiterIndex+1);

            // Compare first part with ESP32 MAC address
            if (id.equals(WiFi.macAddress())) {
                Serial.println("Message matches ESP32 MAC address:");
                Serial.println("setting speed to");
                Serial.println(speedToSet);
            }
        }
    }
    if (strcmp(topic, topic3) == 0) {
        if (message.equals("get")) {
            Serial.println("Received 'get' command from topic2");
            
            // Publish ESP32's MAC address to the same topic
            mqttClient.publish(topic3, WiFi.macAddress().c_str());
            Serial.println("Published ESP32 MAC address to topic2");
        }
    }
    if(strcmp(topic,selfTopic)==0){
      Serial.print("speed set to: ");
      Serial.println(message);
      setSpeeds(message.toInt());
    }
}
