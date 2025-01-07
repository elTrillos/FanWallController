#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFiManager.h>
#include <TaskScheduler.h>  // For task management

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int eepromSize = 128;  // Increase EEPROM size to store MQTT credentials
const int buttonPin = 33;    // WiFi credentials button pin
const int ledPin = LED_BUILTIN;        // Onboard LED pin (GPIO 2)
bool buttonPressed = false;

char mqttBroker[40];
char mqttUsername[40];
char mqttPassword[40];
int mqttPort = 1883;

const char *selfTopic = "fanWall/wall/";
const char *topic1 = "fanWall/wall/control";
const char *topic2 = "fanWall/wall/status";
const char *topic3 = "fanWall/wall/id";

float currentSpeed = 1000.0;
int pollingFrequency = 2000;
int currpower = 0;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
TaskHandle_t fanControlTask;
TaskHandle_t mqttTask;

char storedSSID[eepromSize];
char storedPassword[eepromSize];

// WiFiManager custom parameters
WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqttBroker, 40);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", String(mqttPort).c_str(), 6);
WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqttUsername, 40);
WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqttPassword, 40);

void saveConfigCallback() {
  Serial.println("Saving config...");
  strcpy(mqttBroker, custom_mqtt_server.getValue());
  mqttPort = atoi(custom_mqtt_port.getValue());
  strcpy(mqttUsername, custom_mqtt_user.getValue());
  strcpy(mqttPassword, custom_mqtt_pass.getValue());
  
  EEPROM.begin(eepromSize);
  EEPROM.writeString(64, mqttBroker);  // Store MQTT broker
  EEPROM.writeString(104, mqttUsername);  // Store MQTT username
  EEPROM.writeString(144, mqttPassword);  // Store MQTT password
  EEPROM.writeInt(184, mqttPort);  // Store MQTT port
  EEPROM.commit();
  EEPROM.end();
}

void readConfig() {
  EEPROM.begin(eepromSize);
  EEPROM.readString(64, mqttBroker, 40);
  EEPROM.readString(104, mqttUsername, 40);
  EEPROM.readString(144, mqttPassword, 40);
  EEPROM.end();
  Serial.printf("MQTT DATA");
  Serial.printf(mqttBroker);
}

void connectMQTT() {
    Serial.println(mqttBroker);
    Serial.println(mqttPort);
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
    String helloMessage = "Connected/" + WiFi.macAddress();
    mqttClient.publish(topic2, helloMessage.c_str());
    
    mqttClient.subscribe(selfTopic);
    mqttClient.subscribe(topic1);
    mqttClient.subscribe(topic3);
}

void setupWiFi() {
  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Set custom parameters
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);

  // Automatically connect using saved credentials,
  // or start configuration portal if none are found
  if (!wifiManager.autoConnect("ESP32_AP")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
    delay(5000);
  }

  // If you get here you have connected to the WiFi
  Serial.println("Connected to WiFi");
  digitalWrite(ledPin, HIGH); // Turn on LED when WiFi is connected
  selfTopic = strdup((String(selfTopic) + WiFi.macAddress()).c_str());
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
          digitalWrite(ledPin, LOW); // Turn off LED when WiFi is disconnected
          connectMQTT();  // Reconnect if MQTT connection is lost
          digitalWrite(ledPin, HIGH); // Turn on LED when reconnected
        }
        Serial.println(pollingFrequency);
        delay(pollingFrequency);
    }
}

void setSpeeds(int targetSpeed) {
    for (int fan = 0; fan < 16; fan++) {
      pwm.setPWM(fan, 0, targetSpeed * 40);
    }
}

void setup() {
    Serial.begin(9600);
    pinMode(ledPin, OUTPUT);  // Initialize LED pin as an output
    digitalWrite(ledPin, LOW);  // Ensure LED is off initially
    pwm.begin();

    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(250);  // This is the maximum PWM frequency
    Wire.setClock(400000);
    pwm.setPWM(0, 4096, 0);

    readConfig();  // Read stored WiFi and MQTT credentials from EEPROM
    setupWiFi();  // Set up WiFi using WiFiManager

    connectMQTT();  // Connect to the MQTT broker
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
            pollingFrequency = 500;
        } else if (message.equals("cancel")) {
            Serial.println("Received cancel command from topic2");
            pollingFrequency = 2000;
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
    if (strcmp(topic, selfTopic) == 0) {
      Serial.print("speed set to: ");
      Serial.println(message);
      setSpeeds(message.toInt());
    }
}
