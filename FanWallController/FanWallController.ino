
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int eepromSize = 64;
const int buttonPin = 33;//WiFi credentials button pin
bool buttonPressed = false;

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

void waitForSerialMessage() {
  Serial.println("Waiting for serial message...");
  while (!Serial.available()) {
    // Wait until a serial message is available
  }
  // Read the serial message
  String serialMessage = Serial.readStringUntil('/');
  Serial.println("Received Serial Message: " + serialMessage);
  
  // Split the serial message into ssid and password
  int separatorIndex = serialMessage.indexOf('/');
  if (separatorIndex != -1) { // Check if the separator was found
    String ssid = serialMessage.substring(0, separatorIndex);
    String password = serialMessage.substring(separatorIndex + 1);
    writeCredentials(ssid.c_str(), password.c_str()); // Call writeCredentials with ssid and password
    Serial.println("Credentials written to EEPROM.");
  } else {
    Serial.println("Invalid serial message format.");
  }
}


void checkButtonDuringSetup() {
  // Check button state for a couple of seconds
  unsigned long startTime = millis(); // Get the current time
  while (millis() - startTime < 2000) { // Check for 2 seconds
    if (digitalRead(buttonPin) == LOW) {
      buttonPressed = true;
      break; // Exit the loop if the button is pressed
    }
  }

  // Print the button state after the check
  Serial.print("Button state after setup: ");
  Serial.println(buttonPressed);

  if (buttonPressed) {
    waitForSerialMessage(); // Wait for a serial message if the button is pressed
  }
}


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

void setupWiFi(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  selfTopic = strdup((String(selfTopic) + WiFi.macAddress()).c_str());
  Serial.printf("Connected to WiFi. Topic: %s\n", selfTopic);
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
    String helloMessage = "Connected/" + WiFi.macAddress();
    mqttClient.publish(topic2, helloMessage.c_str());
    
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
    //checkButtonDuringSetup();
    readCredentials(); // Read stored credentials from EEPROM
  
    if (strlen(storedSSID) > 0 && strlen(storedPassword) > 0) {
      Serial.println("Stored credentials found. Connecting to WiFi...");
      setupWiFi(storedSSID, storedPassword); // Connect to WiFi using stored credentials
    } else {
      setupWiFi(ssid, password);
    }
    
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
