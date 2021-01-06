// For Temperature and Humidity
#include <DHT.h>


#define DHTPIN       13
#define SOILMOISTURE 15
#define LDR1         26
#define LDR2         25


#define DHTTYPE DHT11
// create a dht object
DHT dht(DHTPIN, DHTTYPE);

int temperature, humidity,ldrOne,ldrTwo,soilMoisture;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin(); 

}

void loop() {
  // put your main code here, to run repeatedly:
    temperature = dht.readTemperature();
    delay(500);
    Serial.printf("Temp:%d\n",temperature);
    
    humidity = dht.readHumidity();
    delay(500);
    Serial.printf("Humidity:%d\n",humidity);
    
    if (isnan(temperature) || isnan(humidity)) 
        Serial.println(F("--> Failed to read from DHT sensor!"));
    int ldrOneRead = analogRead(LDR1);
    ldrOne = 100 - map(ldrOneRead, 0, 4095, 1, 100);
    delay(500);
    Serial.printf("LDR ONE:%d\n",ldrOne);
    Serial.printf("LDR RAW ONE:%d\n",ldrOneRead);
    
    int ldrTwoRead = analogRead(LDR2);
    ldrTwo = 100 - map(ldrTwoRead, 0, 4095, 1, 100);
    delay(500);
    Serial.printf("LDR TWO:%d\n",ldrTwo);
    Serial.printf("LDR RAW TWO:%d\n",ldrTwoRead);
    int soilRead = analogRead(SOILMOISTURE);
    soilMoisture = 100 - map(soilRead, 0, 4095, 1, 100);
    delay(500);
    Serial.printf("Soil Moisture:%d\n",soilMoisture);
    Serial.printf("Soil Moisture Raw:%d\n",soilRead);
    
}
