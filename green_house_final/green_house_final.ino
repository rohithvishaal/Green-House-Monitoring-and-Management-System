// Necessary Libraries
#include <AsyncMqttClient.h>
#include <WiFi.h>
extern "C"{
    #include "freertos/FreeRTOS.h"
    #include "freertos/timers.h"
}

// For Temperature and Humidity
#include <DHT.h>



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
#define MQTT_PASS "incorrect@007"


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
#define DHTPIN       13
#define SOILMOISTURE 12
#define PH           17
#define LDR1         26
#define LDR2         25


// Pin Configuration for actuators
#define WATERPUMP    34
#define WINDOW_MOTOR 27

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
const long interval = 10000;
const long subscribeInterval = 2000;


//  Variables to hold sensor readings
float temperature=26;
float humidity=65;
int soilMoisture=92;
int ldrOne=70,ldrTwo=70;
float pH=5;

// Variables to hold theresholds from the app
int tempLowThreshold=25,tempHighThreshold=30;
int humLowThreshold=62, humHighThreshold=95;
int soilMoistureLowThreshold=61, soilMoistureHighThreshold=95;
int ldrOneLowThreshold=65, ldrOneHighThreshold=99;
int ldrTwoLowThreshold=65, ldrTwoHighThreshold=99;
int pHLowThreshold=4, pHHighThreshold=12;

//User Actions
boolean manualOverride = false;

// first status
boolean initialStatus = true;





void connectToWifi(){
    Serial.println("--> Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt(){
    Serial.println("--> Connecting to MQTT...");
    mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event){
    switch(event){
        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.println("--> WiFi connected");
            Serial.println("--> IP address: ");
            Serial.println(WiFi.localIP());
            connectToMqtt();
            break;
        
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("--> WiFi connection lost!");
            // Ensure we don't connect to MQTT broker when there is no WiFi
            xTimerStop(mqttReconnectTimer, 0);
            xTimerStart(wifiReconnectTimer, 0);
            break;
    }
}


void onMqttConnect(bool sessionPresent){
    Serial.println("--> Connected to MQTT.");
    Serial.print("--> Session Present:");
    Serial.println(sessionPresent);

    // When we connect to the MQTT we subscribe to topics
    
    uint16_t packetIdSub1 = mqttClient.subscribe(MQTT_SUB_TEMP_LOW_SET,1);

    uint16_t packetIdSub2 = mqttClient.subscribe(MQTT_SUB_TEMP_HIGH_SET,1);

    uint16_t packetIdSub3 = mqttClient.subscribe(MQTT_SUB_HUM_LOW_SET,1);

    uint16_t packetIdSub4 = mqttClient.subscribe(MQTT_SUB_HUM_HIGH_SET,1);

    uint16_t packetIdSub5 = mqttClient.subscribe(MQTT_SUB_SOIL_MOISTURE_LOW_SET,1);
    
    uint16_t packetIdSub6 = mqttClient.subscribe(MQTT_SUB_SOIL_MOISTURE_HIGH_SET,1);

    uint16_t packetIdSub7 = mqttClient.subscribe(MQTT_SUB_LDR1_LOW_SET,1);
  
    uint16_t packetIdSub8 = mqttClient.subscribe(MQTT_SUB_LDR1_HIGH_SET,1);
  
    uint16_t packetIdSub9 = mqttClient.subscribe(MQTT_SUB_LDR2_LOW_SET,1);

    uint16_t packetIdSub10 = mqttClient.subscribe(MQTT_SUB_LDR2_HIGH_SET,1);

    uint16_t packetIdSub11 = mqttClient.subscribe(MQTT_SUB_PH_LEVEL_LOW_SET,1);

    uint16_t packetIdSub12 = mqttClient.subscribe(MQTT_SUB_PH_LEVEL_HIGH_SET,1);
    
    uint16_t packetIdSub13 = mqttClient.subscribe(MQTT_SUB_MANUAL_OVERRIDE,1);

    uint16_t packetIdSub14 = mqttClient.subscribe(MQTT_SUB_WATER_PUMP_SWITCH,1);

    uint16_t packetIdSub15 = mqttClient.subscribe(MQTT_SUB_WINDOW_SWITCH,1);
    


}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason){
    Serial.println("--> Disconnected from MQTT.");
    if(WiFi.isConnected()){
        // If the connection is lost to the MQTT.We check if the WiFi connection is present or not
        // Then we re-attempt to connect to the MQTT
        xTimerStart(mqttReconnectTimer, 0);
    }
}


void onMqttSubscribe(uint16_t packetId, uint8_t qos){
    Serial.println("--> Subscribe acknowledged.");
    Serial.print("--> packet ID:");
    Serial.print(packetId);
    Serial.print("--> qos: ");
    Serial.println(qos);

}


void onMqttPublish(uint16_t packetId){
    Serial.print("--> data published:");
    Serial.println(packetId);
}


String valueExtract(char* payload, size_t len){
    String temp = "";
    for(int i=0;i<=len;i++){
        temp += String((char)payload[i]);
    }
    Serial.println(temp);
    return temp;    
}

void sendStatusFeedback(String msg, int channel){
  if(channel==1)
  mqttClient.publish(MQTT_PUB_STATUS, 1, true, String(msg).c_str());
  if(channel==2)
  mqttClient.publish(MQTT_PUB_STATUS1, 1, true, String(msg).c_str());
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    Serial.println("--> Publish received.");
    Serial.print("--> topic: ");
    Serial.println(topic);
    String value = valueExtract(payload,len);
    Serial.print(String(topic)+":"+String(value));

      if(String(topic)==MQTT_SUB_TEMP_LOW_SET)
           tempLowThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_TEMP_HIGH_SET)
           tempHighThreshold = value.toInt();

      if(String(topic)==MQTT_SUB_HUM_LOW_SET)
           humLowThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_HUM_HIGH_SET)
           humHighThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_SOIL_MOISTURE_LOW_SET)
           soilMoistureLowThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_SOIL_MOISTURE_HIGH_SET)
           soilMoistureHighThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_LDR1_LOW_SET)
           ldrOneLowThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_LDR1_HIGH_SET)
           ldrOneHighThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_LDR2_LOW_SET)
           ldrTwoLowThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_LDR2_HIGH_SET)
           ldrTwoHighThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_PH_LEVEL_LOW_SET)
           pHLowThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_PH_LEVEL_HIGH_SET)
           pHHighThreshold = value.toInt();

       if(String(topic)==MQTT_SUB_MANUAL_OVERRIDE){
          if(value[0]=='t') {manualOverride = true;
          sendStatusFeedback("Manual Override ON!",1);
          }
          else{ manualOverride = false;
          sendStatusFeedback("Manual Override OFF!",1);}
        }

       if(String(topic)==MQTT_SUB_WATER_PUMP_SWITCH){
          if(value[0]=='t' && manualOverride){
            sendStatusFeedback("Turning on water pump",2);
          }
          else if(value[0]=='f' && manualOverride){
            sendStatusFeedback("Turning off water pump",2);
          }
          else{
            sendStatusFeedback("Cannot execute action! Manual Override OFF!",1);
          }
        }

         if(String(topic)==MQTT_SUB_WINDOW_SWITCH){
          if(value[0]=='t' && manualOverride){
            sendStatusFeedback("Opening windows",2);
          }
          else if(value[0]=='f' && manualOverride){
            sendStatusFeedback("Closing windows",2);
          }
          else{
            sendStatusFeedback("Cannot execute action! Manual Override OFF!",1);
          }
        }
           
        

    
}

int ldrReading(int pin){
  delay(500);
  int val = analogRead(pin);
  int res = 100 - map(analogRead(val), 0, 4095, 1, 100);
  return res;
  
}

void getSensorData(boolean randomMode){
 
  if(randomMode){
    temperature = random(25,31);
    humidity = random(60,96);
    pH = random(4,15);
    ldrOne = random(65,100);
    ldrTwo = random(65,100);
    soilMoisture = random(60,96);
    
  }
  else{
    temperature = dht.readTemperature();
    delay(500);
    humidity = dht.readHumidity();
    delay(500);
    if (isnan(temperature) || isnan(humidity)) 
        Serial.println(F("--> Failed to read from DHT sensor!"));
    pH = random(4,15);
    ldrOne = 100 - map(analogRead(LDR1), 0, 4095, 1, 100);
    delay(1000);
    ldrTwo = 100 - map(analogRead(LDR2), 0, 4095, 1, 100);
    delay(1000);
    soilMoisture = 100 - map(analogRead(SOILMOISTURE), 0, 4095, 1, 100);
    
  }
}

void thresholdChecks(){
   if(initialStatus) {sendStatusFeedback("ALL OK!",2); initialStatus = false;}
  if(temperature<tempLowThreshold){
  Serial.println("--> Temperature below Lower Threshold!");
  sendStatusFeedback("Temperature below Lower Threshold",1);
  if(!manualOverride){
    thermostat(true,tempLowThreshold+2);
  }
}
  if(temperature>tempHighThreshold){
  Serial.println("--> Temperature above Higher Threshold!");
  sendStatusFeedback("Temperature above Higher Threshold",1);
  if(!manualOverride){
    waterPump(true);
  }
}
delay(1000);
if(humidity<humLowThreshold ){
  Serial.println("--> Humidity below Lower threshold!");
  sendStatusFeedback("Humidity below Lower threshold!",1);
   if(!manualOverride){
    waterPump(true);
  }
}
if(humidity>humHighThreshold ){
  Serial.println("--> Humidity above Higher threshold!");
  sendStatusFeedback("Humidity above Higher threshold!",1);
   if(!manualOverride){
    thermostat(true,tempLowThreshold+4);
  }
}
delay(1000);
if(ldrOne<ldrOneLowThreshold){
   Serial.println("--> LDR ONE below Lower threshold!");
  sendStatusFeedback("LDR ONE below Lower threshold!",1);
   if(!manualOverride){
    windows(true);
  }
}
if(ldrOne>ldrOneHighThreshold){
   Serial.println("--> LDR ONE above higher threshold!");
  sendStatusFeedback("LDR ONE above higher threshold!",1);
   if(!manualOverride){
    windows(false);
  }
}
delay(1000);
if(ldrTwo<ldrTwoLowThreshold){
   Serial.println("--> LDR TWO below Lower threshold!");
  sendStatusFeedback("LDR TWO below Lower threshold!",1);
   if(!manualOverride){
    windows(true);
  }
}
if(ldrTwo>ldrTwoHighThreshold){
   Serial.println("--> LDR TWO above higher threshold!");
  sendStatusFeedback("LDR TWO above higher threshold!",1);
   if(!manualOverride){
    windows(false);
  }
}
delay(1000);
if(soilMoisture<soilMoistureLowThreshold ){
   Serial.println("--> Soil Moisture  below lower threshold!");
  sendStatusFeedback("Soil Moisture below lower threshold!",1);
   if(!manualOverride){
    waterPump(true);
  }
}
if(soilMoisture>soilMoistureHighThreshold ){
   Serial.println("--> Soil Moisture  above higher threshold!");
  sendStatusFeedback("Soil Moisture above higher threshold!",1);
   if(!manualOverride){
    waterPump(false);
  }
}
delay(1000);
if(pH<pHLowThreshold ){
   Serial.println("--> pH below Lower threshold!");
  sendStatusFeedback("pH below Lower threshold!",1);
}
if(pH>pHHighThreshold ){
   Serial.println("--> pH above Higher threshold!");
  sendStatusFeedback("pH above Higher threshold!",1);
}
delay(5000);
}



void setup()
{
   
    // Initialize the Serial Monitor with a baud rate
    Serial.begin(115200);
    // Initialize the DHT Sensor
    dht.begin();
    // if analog input pin 11 is unconnected, random analog
    // noise will cause the call to randomSeed() to generate
    // different seed numbers each time the sketch runs.
    // randomSeed() will then shuffle the random function.
    // https://www.arduino.cc/reference/en/language/functions/random-numbers/random/
    randomSeed(analogRead(11));
    // The below timers are for reconnecting to the WiFi router and MQTT broker
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE,(void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE,(void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
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
    mqttClient.setCredentials(MQTT_USERNAME,MQTT_PASS);
    connectToWifi();
    sendStatusFeedback("ALL OK!",2);

}

void loop() {

  unsigned long currentMillis = millis();
    // Every X number of seconds (interval = 10 seconds) 
    // it publishes a new MQTT message
    if(currentMillis - previousMillis >= 10000){
        //Save the last time a new message was sent.
        previousMillis = currentMillis;
        getSensorData(true);
        mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temperature).c_str()); 
        delay(1000);
        Serial.printf(" Temperature: %.2f \n", temperature);
        mqttClient.publish(MQTT_PUB_HUM, 1, true, String(humidity).c_str()); 
        delay(1000);
        Serial.printf(" Humidity: %.2f \n", humidity);
        mqttClient.publish(MQTT_PUB_SOIL_MOISTURE, 1, true, String(soilMoisture).c_str());
        delay(1000);
        Serial.printf(" Soil Moisture: %.2f \n", soilMoisture);
        mqttClient.publish(MQTT_PUB_LDR1, 1, true, String(ldrOne).c_str()); 
        delay(1000);
        Serial.printf(" LDR ONE: %d \n", ldrOne);
        mqttClient.publish(MQTT_PUB_LDR2, 1, true, String(ldrTwo).c_str());
        delay(1000);                           
        Serial.printf(" LDR TWO: %d \n", ldrTwo);
        mqttClient.publish(MQTT_PUB_PH_LEVEL, 1, true, String(pH).c_str());
        delay(1000);                            
        Serial.printf(" pH: %.2f \n", pH); 
}
delay(5000);
thresholdChecks();
}


void waterPump(int state){
  if(state){
    Serial.println("--> MOTOR ON");
    sendStatusFeedback("MOTOR ON",2);
    mqttClient.publish(MQTT_SUB_WATER_PUMP_SWITCH, 1, true, String("true").c_str());
  }
  else{
    Serial.println("--> MOTOR OFF");
    sendStatusFeedback("MOTOR OFF",2);
    mqttClient.publish(MQTT_SUB_WATER_PUMP_SWITCH, 1, true, String("false").c_str());
  }
}


void windows(int state){
  if(state){
    Serial.println("--> OPENING WINDOWS");
    sendStatusFeedback("OPENING WINDOWS",2);
    mqttClient.publish(MQTT_SUB_WINDOW_SWITCH, 1, true, String("true").c_str());
  }
  else{
    Serial.println("--> CLOSING WINDOWS");
    sendStatusFeedback("CLOSING WINDOWS",2);
    mqttClient.publish(MQTT_SUB_WINDOW_SWITCH, 1, true, String("false").c_str());
  }
  
}


void thermostat(int state, int temp){
  if(state){
    Serial.println("--> TURNING ON THERMOSTAT");
    Serial.printf("Setting temperature to:%d",temp);
    sendStatusFeedback("Setting thermostat to "+String(temp)+"°C",2);
  }
  else{
    Serial.println("--> TURNING OF THERMOSTAT");
    sendStatusFeedback("Thermostat off",2);
  }
}
