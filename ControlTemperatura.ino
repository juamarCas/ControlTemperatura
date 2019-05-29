
#include "DHT.h"
#include <Adafruit_Sensor.h>

#define DHTPIN 2
#define DHTTYPE DHT22
#define controlPin 4
DHT dht(DHTPIN, DHTTYPE);
float t; 
float tMin = 29; // temperatura mínima
float tMax = 32; // temperatura máxima

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(controlPin, OUTPUT);
  dht.begin();

}

void loop() {

 t = dht.readTemperature();
 
 if(t <= tMin){
    digitalWrite(controlPin, HIGH);
 }else if (t > tMax){
    digitalWrite(controlPin, LOW);
 }
 Serial.println(t);

}
