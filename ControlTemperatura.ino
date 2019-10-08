#include <LiquidCrystal.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <avr/io.h>
#define tempControl PD0
#define humControl PD1
#define batteryControl PB4

//variables de la temperatura y humedad
#define DHTPIN 7
#define DHTTYPE DHT22
int t, h; 

int maxTemp = 32, minTemp = 29; 
int maxHum, minHum;
int counter = 0;
int interval = 2000; 
int lastTime; 
int batteryLevel, minBatteryLevel; 
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(5, 6, 8, 9, 10, 11); 

void setup() {
  DDRD = (1 << tempControl) | (1 << humControl);
  DDRB = (1 << batteryControl);
  PORTB &= ~(1 << batteryControl);
  PORTD = (1 << PD0); //activo la resistencia pullUp en el pin 0
  dht.begin();
  Serial.begin(9600);
  lcd.begin(16, 2);

  EIMSK = (1 << INT0) | (1 << INT1); //activo la interrupción del PD2
  EICRA = (1 << ISC01) | (1 << ISC11); // flanco de bajada 
  sei();
}
/*
 set cursor (col, row) 
*/
void loop() {
  
  //Serial.println(t);

  if(t >= maxTemp){
     PORTD &= ~(1 << tempControl);
  }else if (t < minTemp){
    PORTD |= (1 << tempControl);
  }

  if(h >= maxHum){
     PORTD &= ~(1 << humControl);
  }else if (t < minHum){
    PORTD |= (1 << humControl);
  }

  if(batteryLevel < minBatteryLevel){
    PORTB |= (1 << batteryControl);
  }else{
    PORTB &= ~(1 << batteryControl);
  }
 

  unsigned long now = millis();
  if(now - lastTime >= interval){
    lastTime = now;
    t = dht.readTemperature();
    h = dht.readHumidity();
    UpdateText();
  }
   
}

void UpdateText(){
  lcd.setCursor(0,0);
  lcd.print("temp=");
  lcd.print(t);
  lcd.setCursor(8,0);
  lcd.print("batt=");
  lcd.print(batteryLevel);
  lcd.setCursor(0,1);
  lcd.print("hum=");
  lcd.print(h);
  
}

ISR(INT0_vect) {
  Serial.println("hon");
}

ISR(INT1_vect) {
  Serial.println("hon1");
}


