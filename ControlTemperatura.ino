#include <LiquidCrystal.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <avr/io.h>
#define tempControl 4
#define humControl PB1
#define batteryControl PD4
#define DHTPIN 7
#define DHTTYPE DHT22

int t, h;
unsigned int menuState = 0;
unsigned int maxTemp = 30, minTemp = 28;
unsigned int maxHum = 47;
unsigned int counter = 0;
const int interval = 2000;
int lastTime = 0;
int batteryLevel;
int minBatteryLevel;
bool isInConfiguration = false;
DHT dht(DHTPIN, DHTTYPE);
double dutyCycle = 20; 
LiquidCrystal lcd(4, 5, 11, 6, 13, 8);
/*
  batteryControl lo que encenderá o apagará el led indicando baja batería
*/
void setup() {
  DDRB = (1 << tempControl);
  DDRD = (1 << batteryControl);
  //PORTD |= (1 << PD0); //resistencia pull up
  pinMode(9, OUTPUT); 
  dht.begin();
  Serial.begin(9600);
  lcd.begin(16, 2);

  EIMSK = (1 << INT0) | (1 << INT1);//activo la interrupción del PD2 y PD3
  EICRA = (1 << ISC01) | (1 << ISC11);//flanco de bajada
/*
  startPWM
  conft1b
  sMask*/
  //OCR1A = (dutyCycle/100)*255;
  sei();
}
/*
  set cursor (col, row)
*/
void loop() {
   
  // control de los estados de menú
  if (menuState == 0) {
    isInConfiguration = false;
  }/*
  if ((PIND &= (1 << PD0)) == 0) {
    
    lcd.clear();
    menuState += 1;
    isInConfiguration = true;  
    if (menuState > 4) {
      menuState = 0;
      isInConfiguration = false;
    }
     _delay_ms(300);
  }*/
  // fin de control de menú

  //Código que maneja el control de variables en el horno
  if (t >= maxTemp) {
    PORTB &= ~(1 << tempControl);
  } else if (t < minTemp) {
    PORTB |= (1 << tempControl);
  }

  if (h >= maxHum) { 
    analogWrite(9, 100); 
  } else {
    analogWrite(9, 0); 
  }

  if (batteryLevel < minBatteryLevel) {
    PORTD |= (1 << batteryControl);
  } else {
    PORTD &= ~(1 << batteryControl);
  }
  //Fin de código de control de variables en el horno

  unsigned long now = millis();
  if (now - lastTime >= interval) {
    lastTime = now;
    t = dht.readTemperature();
    h = dht.readHumidity();
    if (!isInConfiguration) {
      PrincipalMenuText();
    }
  }

  if (isInConfiguration) {
    ConfigurationMenuText();
  }


}

void PrincipalMenuText() {
 // lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("temp=");
  lcd.print(t);
  lcd.setCursor(8, 0);
  lcd.print("batt=");
  lcd.print(batteryLevel);
  lcd.setCursor(0, 1);
  lcd.print("hum=");
  lcd.print(h);

}

void ConfigurationMenuText() {
  
  if (menuState == 1) {
    lcd.setCursor(0, 0);
    lcd.print("MaxTemp= ");
    lcd.print(maxTemp);
  } else if (menuState == 2) {
    lcd.setCursor(0, 0);
    lcd.print("MinTemp= ");
    lcd.print(minTemp);
  } else if (menuState == 3) {
    lcd.setCursor(0, 0);
    lcd.print("MaxHum= ");
    lcd.print(maxHum);
  } else if (menuState == 4) {
    lcd.setCursor(0, 0);
    lcd.print("MinHum= ");
    lcd.print(minHum);
  }
}

//Incremento de las variables
ISR(INT0_vect) {
  if (isInConfiguration) {
    if (menuState == 1) {
      maxTemp++;
    } else if (menuState == 2) {
      minTemp++;
      if (minTemp >= maxTemp - 1) {
        minTemp = maxTemp - 2;
      }
    } else if (menuState == 3) {
      maxHum++;
    } else if (menuState == 4) {
      minHum++;
      if (minHum >= maxHum - 1) {
        minHum = maxHum - 2;
      }
    }
  }
  _delay_ms(10);
}

//Decremento de las variables
ISR(INT1_vect) {
  if (isInConfiguration) {
    if (menuState == 1) {
      maxTemp--;
      if (maxTemp <= minTemp + 1) {
        maxTemp = minTemp + 2;
      }
    } else if (menuState == 2) {
      minTemp--;
    } else if (menuState == 3) {
      maxHum--;
      if (maxHum <= minHum + 1) {
        maxHum = minHum + 2;
      }
    } else if (menuState == 4) {
      minHum--;

    }
  }
  _delay_ms(10);
}
