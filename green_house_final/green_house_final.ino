// Necessary Libraries
#include <AsyncMqttClient.h>
#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

// For Temperature and Humidity
#include <DHT.h>

// For Servo that is the window motor
#include <ESP32Servo.h>


// WiFi configuration
#define WIFI_SSID  "Router"
#define WIFI_PASSWORD  "aishwarya@007"



// MQTT configuration
// make sure it is an static IP
// If the IP is not static set it up as a static IP in the router configuration
// If you using a cloud MQTT broker type in the domain name
// #define MQTT_HOST "io.adafruit.com"
#define MQTT_HOST "192.168.31.169"
// This port is insecure(not encrypted) but if your running it locally it's okay if your wifi is not hacked
// If you are using a cloud broker make sure you use an SSL connection.
// https://www.digitalocean.com/community/tutorials/how-to-install-and-secure-the-mosquitto-mqtt-messaging-broker-on-ubuntu-16-04
#define MQTT_PORT 1883
// Setting up authentication for mosquitto broker
// https://medium.com/@eranda/setting-up-authentication-on-mosquitto-mqtt-broker-de5df2e29afc
#define MQTT_USERNAME "rohvis"
#define MQTT_PASS "incorrect007"


// MQTT Topics
// Sensor topics
#define MQTT_PUB_TEMP              "greenhouse/feeds/temperature"
#define MQTT_PUB_HUM               "greenhouse/feeds/humidity"
#define MQTT_PUB_LDR1              "greenhouse/feeds/ldr1"
#define MQTT_PUB_LDR2              "greenhouse/feeds/ldr2"
#define MQTT_PUB_SOIL_MOISTURE     "greenhouse/feeds/soil-moisture"
#define MQTT_PUB_PH_LEVEL          "greenhouse/feeds/pH-level"

// Threshold Settings topics
#define MQTT_SUB_TEMP_LOW_SET           "greenhouse/feeds/temperature-low-set"
#define MQTT_SUB_TEMP_HIGH_SET          "greenhouse/feeds/temperature-high-set"
#define MQTT_SUB_HUM_LOW_SET            "greenhouse/feeds/humidity-low-set"
#define MQTT_SUB_HUM_HIGH_SET           "greenhouse/feeds/humidity-high-set"
#define MQTT_SUB_LDR1_LOW_SET           "greenhouse/feeds/ldr1-low-set"
#define MQTT_SUB_LDR1_HIGH_SET          "greenhouse/feeds/ldr1-high-set"
#define MQTT_SUB_LDR2_LOW_SET           "greenhouse/feeds/ldr2-low-set"
#define MQTT_SUB_LDR2_HIGH_SET          "greenhouse/feeds/ldr2-high-set"
#define MQTT_SUB_PH_LEVEL_LOW_SET       "greenhouse/feeds/pH-level-low-set"
#define MQTT_SUB_PH_LEVEL_HIGH_SET      "greenhouse/feeds/pH-level-high-set"
#define MQTT_SUB_SOIL_MOISTURE_LOW_SET  "greenhouse/feeds/soil-moisture-low-set"
#define MQTT_SUB_SOIL_MOISTURE_HIGH_SET "greenhouse/feeds/soil-moisture-high-set"

// User Actions topics
#define MQTT_SUB_WATER_PUMP_SWITCH "greenhouse/feeds/water-pump-switch"
#define MQTT_SUB_WINDOW_SWITCH     "greenhouse/feeds/window-switch"
#define MQTT_SUB_MANUAL_OVERRIDE   "greenhouse/feeds/manual-override"


// Feedback topics
#define MQTT_PUB_STATUS            "greenhouse/feeds/status"
#define MQTT_PUB_STATUS1            "greenhouse/feeds/status1"

// Board : https://circuits4you.com/2018/12/31/esp32-devkit-esp32-wroom-gpio-pinout/
// Pin Configuration for Sensors
#define DHTPIN       26
#define SOILMOISTURE 35
#define PH           17
#define LDR1         32
#define LDR2         33


// Pin Configuration for actuators
#define WATERPUMP    12
#define WINDOW_MOTOR 14

#define DHTTYPE DHT11
// create a dht object
DHT dht(DHTPIN, DHTTYPE);


// Initializing a MQTT client Object
AsyncMqttClient mqttClient;

// Creates a new software timer instance and returns a handle by which the timer can be referenced.
// Doc:https://www.freertos.org/FreeRTOS-timers-xTimerCreate.html
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// Stores last time when data was published
unsigned long previousMillis = 0;
unsigned long previousSubMillis = 0;
// Interval at which to publish the sensor readings
const long interval = 5000;
const long subscribeInterval = 2000;


//  Variables to hold sensor readings
float temperature = 26;
float humidity = 62;
int soilMoisture = 55;
int ldrOne = 62, ldrTwo = 75;
float pH = 6;

// Variables to hold theresholds from the app
int tempLowThreshold = 25, tempHighThreshold = 40;
int humLowThreshold = 30, humHighThreshold = 70;
int soilMoistureLowThreshold = 50, soilMoistureHighThreshold = 65;
int ldrOneLowThreshold = 20, ldrOneHighThreshold = 95;
int ldrTwoLowThreshold = 20, ldrTwoHighThreshold = 95;
int pHLowThreshold = 5, pHHighThreshold = 12;

// setting ideal safe values
int tempSafe = (tempLowThreshold + tempHighThreshold)/2;
int humSafe = (humLowThreshold + humLowThreshold)/2;
int soilSafe = ( soilMoistureLowThreshold + soilMoistureHighThreshold)/2;
int ldrOneSafe =  (ldrOneLowThreshold + ldrOneHighThreshold)/2;
int ldrTwoSafe =  (ldrOneLowThreshold + ldrOneHighThreshold)/2;
int pHSafe = (pHLowThreshold+pHHighThreshold)/2;
boolean soilSafeState = false;

//User Actions
boolean manualOverride = false;

// first status
boolean initialStatus = true;

// Servo object for the window switch
Servo windowMotor;


void connectToWifi() {
  Serial.println("[ACTION] -- Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("[ACTION] -- Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("[STATUS] -- WiFi connected");
      Serial.println("[INFO] -- IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("[ALERT] -- WiFi connection lost!");
      // Ensure we don't connect to MQTT broker when there is no WiFi
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}


void onMqttConnect(bool sessionPresent) {
  Serial.println("[STATUS] -- Connected to MQTT.");
  Serial.print("[STATUS] -- Session Present:");
  Serial.println(sessionPresent);

  // When we connect to the MQTT we subscribe to topics

  uint16_t packetIdSub1 = mqttClient.subscribe(MQTT_SUB_TEMP_LOW_SET, 1);
  
  uint16_t packetIdSub2 = mqttClient.subscribe(MQTT_SUB_TEMP_HIGH_SET, 1);

  uint16_t packetIdSub3 = mqttClient.subscribe(MQTT_SUB_HUM_LOW_SET, 1);

  uint16_t packetIdSub4 = mqttClient.subscribe(MQTT_SUB_HUM_HIGH_SET, 1);

  uint16_t packetIdSub5 = mqttClient.subscribe(MQTT_SUB_SOIL_MOISTURE_LOW_SET, 1);

  uint16_t packetIdSub6 = mqttClient.subscribe(MQTT_SUB_SOIL_MOISTURE_HIGH_SET, 1);

  uint16_t packetIdSub7 = mqttClient.subscribe(MQTT_SUB_LDR1_LOW_SET, 1);

  uint16_t packetIdSub8 = mqttClient.subscribe(MQTT_SUB_LDR1_HIGH_SET, 1);

  uint16_t packetIdSub9 = mqttClient.subscribe(MQTT_SUB_LDR2_LOW_SET, 1);

  uint16_t packetIdSub10 = mqttClient.subscribe(MQTT_SUB_LDR2_HIGH_SET, 1);

  uint16_t packetIdSub11 = mqttClient.subscribe(MQTT_SUB_PH_LEVEL_LOW_SET, 1);

  uint16_t packetIdSub12 = mqttClient.subscribe(MQTT_SUB_PH_LEVEL_HIGH_SET, 1);

  uint16_t packetIdSub13 = mqttClient.subscribe(MQTT_SUB_MANUAL_OVERRIDE, 1);

  uint16_t packetIdSub14 = mqttClient.subscribe(MQTT_SUB_WATER_PUMP_SWITCH, 1);

  uint16_t packetIdSub15 = mqttClient.subscribe(MQTT_SUB_WINDOW_SWITCH, 1);
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("[ALERT] -- Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    // If the connection is lost to the MQTT.We check if the WiFi connection is present or not
    // Then we re-attempt to connect to the MQTT
    xTimerStart(mqttReconnectTimer, 0);
  }
}


void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("[STATUS] -- Subscribe acknowledged.");
  Serial.print("[INFO] -- packet ID:");
  Serial.print(packetId);
  Serial.print("[INFO] QOS: ");
  Serial.println(qos);
}


void onMqttPublish(uint16_t packetId) {
  Serial.printf("[INFO] -- Data Published with Packet ID:%d\n", packetId);
}


String valueExtract(char* payload, size_t len) {
  String temp = "";
  for (int i = 0; i <= len; i++) {
    temp += String((char)payload[i]);
  }
  Serial.println(temp);
  return temp;
}

void sendStatusFeedback(String msg, int channel) {
  if (channel == 1)
    mqttClient.publish(MQTT_PUB_STATUS, 1, true, String(msg).c_str());
  if (channel == 2)
    mqttClient.publish(MQTT_PUB_STATUS1, 1, true, String(msg).c_str());
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("[STATUS] -- Publish received.");
  Serial.print("[INFO] -- topic: ");
  Serial.println(topic);
  String value = valueExtract(payload, len);
  Serial.print(String(topic) + ":" + String(value));

  if (String(topic) == MQTT_SUB_TEMP_LOW_SET)
    tempLowThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_TEMP_HIGH_SET)
    tempHighThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_HUM_LOW_SET)
    humLowThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_HUM_HIGH_SET)
    humHighThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_SOIL_MOISTURE_LOW_SET)
    soilMoistureLowThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_SOIL_MOISTURE_HIGH_SET)
    soilMoistureHighThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_LDR1_LOW_SET)
    ldrOneLowThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_LDR1_HIGH_SET)
    ldrOneHighThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_LDR2_LOW_SET)
    ldrTwoLowThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_LDR2_HIGH_SET)
    ldrTwoHighThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_PH_LEVEL_LOW_SET)
    pHLowThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_PH_LEVEL_HIGH_SET)
    pHHighThreshold = value.toInt();

  if (String(topic) == MQTT_SUB_MANUAL_OVERRIDE) {
    if (value[0] == 't') {
      manualOverride = true;
      Serial.printf("[INFO] -- Manual Ovveride ON");
      sendStatusFeedback("Manual Override ON!", 1);
    }
    else {
      manualOverride = false;
      Serial.printf("[INFO] -- Manual Ovveride OFF");
      sendStatusFeedback("Manual Override OFF!", 1);
    }
  }

  if (String(topic) == MQTT_SUB_WATER_PUMP_SWITCH) {
    if (value[0] == 't' && manualOverride) {
      Serial.printf("[ACTION] -- MOTOR ON");
      sendStatusFeedback("MOTOR ON", 2);
      waterPump(true);
    }
    else if (value[0] == 'f' && manualOverride) {
      Serial.printf("[ACTION] -- MOTOR ON");
      sendStatusFeedback("MOTOR OFF", 2);
      waterPump(false);
    }
    else {
      Serial.printf("[ALERT] -- Cannot execute action! Manual Ovveride is OFF");
      sendStatusFeedback("Cannot execute action! Manual Override OFF!", 1);
    }
  }

  if (String(topic) == MQTT_SUB_WINDOW_SWITCH) {
    if (value[0] == 't' && manualOverride) {
      sendStatusFeedback("Opening windows", 2);
      Serial.printf("[ACTION] -- OPENING WINDOWS");
      windows(true);
    }
    else if (value[0] == 'f' && manualOverride) {
      sendStatusFeedback("Closing windows", 2);
       Serial.printf("[ACTION] -- CLOSING WINDOWS");
      windows(false);
    }
    else {
      Serial.printf("[ALERT] -- Cannot execute action! Manual Ovveride is OFF");
      sendStatusFeedback("Cannot execute action! Manual Override OFF!", 1);
    }
  }
}
void setSafeValues(){
  //ideal mid values
tempSafe = (tempLowThreshold + tempHighThreshold)/2;
humSafe = (humLowThreshold + humLowThreshold)/2;
soilSafe = ( soilMoistureLowThreshold + soilMoistureHighThreshold)/2;
ldrOneSafe =  (ldrOneLowThreshold + ldrOneHighThreshold)/2;
ldrTwoSafe =  (ldrOneLowThreshold + ldrOneHighThreshold)/2;
pHSafe = (pHLowThreshold+pHHighThreshold)/2;
}

void getSensorData(boolean randomMode) {

  if (randomMode) {
    temperature = random(25, 31);
    humidity = random(60, 96);
    pH = random(5, 15);
    ldrOne = random(65, 100);
    ldrTwo = random(65, 100);
    soilMoisture = random(60, 96);
  }
  else {
    delay(500);
    temperature = dht.readTemperature();
    delay(500);
    humidity = dht.readHumidity();
    delay(500);
    if (isnan(temperature) || isnan(humidity))
      Serial.println(F("[ALERT] -- Failed to read from DHT sensor!"));
    int ldrOneRead = analogRead(LDR1);
    delay(500);
    ldrOne = map(ldrOneRead, 0, 4095, 1, 100);
    delay(500);
    int ldrTwoRead = analogRead(LDR2);
    delay(500);
    ldrTwo = map(ldrTwoRead, 0, 4095, 1, 100);
    delay(500);
    int soilRead = analogRead(SOILMOISTURE);
    soilMoisture = 100 - map(soilRead, 0, 4095, 1, 100);
    delay(500);
    pH = random(4, 15);
  }
}


void thresholdChecks() {
  if (initialStatus) {
    sendStatusFeedback("ALL OK!", 2);
    initialStatus = false;
  }
  if (temperature < tempLowThreshold) {
    Serial.println("[ALERT] -- Temperature below Lower Threshold!");
    sendStatusFeedback("[ALERT] -- Temperature below Lower Threshold", 1);
    if (!manualOverride) {
      thermostat(true, tempLowThreshold + 2);
    }
  }
  if (temperature > tempHighThreshold) {
    Serial.println("[ALERT] -- Temperature above Higher Threshold!");
    sendStatusFeedback("Temperature above Higher Threshold", 1);
    if (!manualOverride) {
      waterPump(true);
    }
  }
  if(temperature > tempSafe -5 && temperature < tempSafe+5){
    Serial.println("[INFO] -- Temperature in safe levels!");
    sendStatusFeedback("Temperature in safe levels!", 1);
  }
  delay(500);
  if (humidity < humLowThreshold ) {
    Serial.println("[ALERT] -- Humidity below Lower threshold!");
    sendStatusFeedback("Humidity below Lower threshold!", 1);
    if (!manualOverride) {
      waterPump(true);
    }
  }
  if (humidity > humHighThreshold ) {
    Serial.println("[ALERT] -- Humidity above Higher threshold!");
    sendStatusFeedback("Humidity above Higher threshold!", 1);
    if (!manualOverride) {
      thermostat(true, tempLowThreshold + 4);
    }
  }
  if(humidity > humSafe -5 && humidity < humSafe+5){
    Serial.println("[INFO] -- humidity in safe levels!");
    sendStatusFeedback("Humidity in safe levels!", 1);
  }
  delay(500);
  if (ldrOne < ldrOneLowThreshold) {
    Serial.println("[ALERT] -- LDR ONE below Lower threshold!");
    sendStatusFeedback("LDR ONE below Lower threshold!", 1);
    if (!manualOverride) {
      windows(true);
    }
  }
  if (ldrOne > ldrOneHighThreshold) {
    Serial.println("[ALERT] -- LDR ONE above higher threshold!");
    sendStatusFeedback("LDR ONE above higher threshold!", 1);
    if (!manualOverride) {
      windows(false);
    }
  }
  delay(500);
  if (ldrTwo < ldrTwoLowThreshold) {
    Serial.println("[ALERT] -- LDR TWO below Lower threshold!");
    sendStatusFeedback("LDR TWO below Lower threshold!", 1);
    if (!manualOverride) {
      windows(true);
    }
  }
  if (ldrTwo > ldrTwoHighThreshold) {
    Serial.println("[ALERT] -- LDR TWO above higher threshold!");
    sendStatusFeedback("LDR TWO above higher threshold!", 1);
    if (!manualOverride) {
      windows(false);
    }
  }
  delay(500);
  if (soilMoisture < soilMoistureLowThreshold ) {
    soilSafeState = false;
    Serial.println("[ALERT] -- Soil Moisture  below lower threshold!");
    sendStatusFeedback("Soil Moisture below lower threshold!", 1);
    if (!manualOverride) {
      waterPump(true);
    }
  }
  if (soilMoisture > soilMoistureHighThreshold ) {
    soilSafeState = false;
    Serial.println("[ALERT] -- Soil Moisture  above higher threshold!");
    sendStatusFeedback("Soil Moisture above higher threshold!", 1);
    if (!manualOverride) {
      waterPump(false);
    }
  }
  delay(500);
   if(soilMoisture > soilSafe -5 && soilMoisture < soilSafe+5){
    soilSafeState = true;
    Serial.println("[INFO] -- Soil Moisture  in safe levels!");
    sendStatusFeedback("soil Moisture in safe levels!", 1);
    delay(500);
    if(soilSafeState && (!manualOverride)) waterPump(false);
  }
  if (pH < pHLowThreshold ) {
    Serial.println("[ALERT] -- pH below Lower threshold!");
    sendStatusFeedback("pH below Lower threshold!", 1);
    sendStatusFeedback(" ", 2);
  }
  if (pH > pHHighThreshold ) {
    Serial.println("[ALERT] -- pH above Higher threshold!");
    sendStatusFeedback("pH above Higher threshold!", 1);
    sendStatusFeedback(" ", 2);
  }
  if(pH > pHSafe -5 && pH < pHSafe+5){
    Serial.println("[INFO] -- PHin safe levels!");
    sendStatusFeedback("pH in safe levels!", 1);
    sendStatusFeedback(" ", 2);
  }
  delay(1000);
}



void setup()
{
  pinMode(WATERPUMP, OUTPUT);
  pinMode(LDR1, INPUT);
  pinMode(WINDOW_MOTOR, OUTPUT);
  // Initialize the Serial Monitor with a baud rate
  Serial.begin(115200);

  // Attach the pin to the servo object
  windowMotor.attach(WINDOW_MOTOR);

  // Initialize the DHT Sensor
  dht.begin();
  // if analog input pin 11 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  // https://www.arduino.cc/reference/en/language/functions/random-numbers/random/
  randomSeed(analogRead(11));
  // The below timers are for reconnecting to the WiFi router and MQTT broker
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  // https://techtutorialsx.com/2019/08/11/esp32-arduino-getting-started-with-wifi-events/
  WiFi.onEvent(WiFiEvent);

  // http://marvinroger.viewdocs.io/async-mqtt-client/2.-API-reference/
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.onMessage(onMqttMessage);
  // If your broker requires authentication.
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASS);
  connectToWifi();
  sendStatusFeedback("ALL OK!", 2);
}

void loop() {

  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds)
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= 2000) {
    //Save the last time a new message was sent.
    previousMillis = currentMillis;
    getSensorData(false);
    Serial.printf("[ACTION] -- Publishing temperature\n");
    Serial.printf("[INFO] -- Temperature:%d\n", temperature);
    mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temperature).c_str());
    delay(500);

    Serial.printf("[ACTION] -- Publishing humidity\n");
    Serial.printf("[INFO] -- Humidity:%d\n", humidity);
    mqttClient.publish(MQTT_PUB_HUM, 1, true, String(humidity).c_str());
    delay(500);

    Serial.printf("[ACTION] -- Publishing soil moisture\n");
    Serial.printf("[INFO] -- Soil Moisture:%d\n", soilMoisture);
    mqttClient.publish(MQTT_PUB_SOIL_MOISTURE, 1, true, String(soilMoisture).c_str());
    delay(500);

    Serial.printf("[ACTION] -- Publishing ldr one\n");
    Serial.printf("[INFO] -- LDR ONE:%d\n", ldrOne);
    mqttClient.publish(MQTT_PUB_LDR1, 1, true, String(ldrOne).c_str());
    delay(500);

    Serial.printf("[ACTION] -- Publishing ldr two\n");
    Serial.printf("[INFO] -- LDR TWO:%d\n", ldrTwo);
    mqttClient.publish(MQTT_PUB_LDR2, 1, true, String(ldrTwo).c_str());
    delay(500);

    Serial.printf("[ACTION] -- Publishing pH\n");
    Serial.printf("[INFO] -- pH:%d\n", pH);
    mqttClient.publish(MQTT_PUB_PH_LEVEL, 1, true, String(pH).c_str());
    delay(500);
  }
  delay(3000);
  setSafeValues();
  thresholdChecks();
}


void waterPump(int state) {
  if (state) {
    Serial.println("[ACTION] -- MOTOR ON");
    digitalWrite(WATERPUMP, HIGH);
    sendStatusFeedback("MOTOR ON", 2);
    delay(2000);
  }
  else {
    Serial.println("[ACTION] -- MOTOR OFF");
    digitalWrite(WATERPUMP, LOW);
    sendStatusFeedback("MOTOR OFF", 2);
    delay(2000);
  }
}


void windows(int state) {
  if (state) {
    Serial.println("[ACTION] -- OPENING WINDOWS");
    delay(2000);
    windowMotor.write(180);
    sendStatusFeedback("OPENING WINDOWS", 2);
    delay(2000);
  }
  else {
    Serial.println("[ACTION] -- CLOSING WINDOWS");
    delay(2000);
    windowMotor.write(0);
    sendStatusFeedback("CLOSING WINDOWS", 2);
    delay(2000);
  }
}


void thermostat(int state, int temp) {
  if (state) {
    Serial.println("[ACTION] -- TURNING ON THERMOSTAT");
    Serial.printf("Setting temperature to:%d", temp);
    sendStatusFeedback("Setting thermostat to " + String(temp) + "°C", 2);
  }
  else {
    Serial.println("[ACTION] -- TURNING OF THERMOSTAT");
    sendStatusFeedback("Thermostat off", 2);
  }
}
