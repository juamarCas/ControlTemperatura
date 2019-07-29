#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>

//variables de la temperatura y humedad
#define DHTPIN 5
#define DHTTYPE DHT22
#define tempControl PD4
#define humControl PD6

//LCD
#define RS 1
#define EN 2
#define CONTROL 0
#define DATA 1


#define clk 11
#define dt 12
#define button PD3


//encoder
#define outputA 10
#define outputB 11


int aState;
int aLastState;
//#define led 4
DHT dht(DHTPIN, DHTTYPE);

float t;
float h; //humedad
float tMin = 29; // temperatura mínima
float tMax = 32; // temperatura máxima
float hMin = 50; // humedad mínima
float hMax = 60; // humedad máxima
int counter = 0; // contador para el timer


//control de estados
int state = 0; //estados de configuración
bool isInConfiguration = false; // está en configuración?
bool updateText = false;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  DDRD = (1 << tempControl) | (1 << humControl);
  DDRC = 0X0F; // LCD
  DDRB = (1 << PB1) | (1 << PB2); // en el puerto b están los pins del encoder

  EIMSK = (1 << INT0); //activo la interrupción del PD2
  EICRA = (1 << ISC00); // flanco de subida

  TCCR0A = (1 << WGM01);
  OCR0A = 195; // 0.01248 Secs
  TCCR0B = (1 << CS02) | (1 << CS00);
  TIMSK0 = (1 << OCIE0A); // habilito interrupción

  lcdStart();
  UpdateText();

  aLastState = digitalRead(outputA);

  dht.begin();
  sei();

}

void loop() {
  aState = digitalRead(outputA);

  if (t <= tMin) {
    PORTD |= (1 << tempControl);
  } else if (t > tMax) {
    PORTD &= ~(1 << tempControl);
  }

  if (h <= hMin) {
    PORTD |= (1 << humControl);
  } else if (t > hMax) {
    PORTD &= ~(1 << humControl);
  }

  if (state > 0) {
    isInConfiguration = true;
  } else if (state <= 0) {
    isInConfiguration = false;
  }
  
  if(isInConfiguration){
    Encoder();
    UpdateText();
  } 
 

}

void Encoder(){
  if (aState != aLastState){         
     if (digitalRead(outputB) != aState) { 
       if(state == 1){
          tMax++;
       }else if(state == 2){
          tMin++;
       }else if(state == 3){
          hMax++;
       }else if(state == 4){
          hMin++;
       }
     } else {
       if(state == 1){
          tMax--;
       }else if(state == 2){
          tMin--;
       }else if(state == 3){
          hMax--;
       }else if(state == 4){
          hMin--;
       }
     }   
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state
}



void UpdateText() {
  clearLcd();
  if (isInConfiguration == false) {

    setLcdCursor(0, 0); // temperatura real
    lcdString("t=");
    lcdNumber(t);
    writeCommand(DATA, 0xDF);

    setLcdCursor(1, 0); // temperatura real
    lcdString("h=");
    lcdNumber(h);
    lcdString("%");
  } else {

    if (state == 1) {
        TempControlConfText();
    } else if (state == 2) {
        TempControlConfText();
    } else if (state == 3) {
        HumControlConfText();
    } else if (state == 4) {
        HumControlConfText();
    }
  }
}

void TempControlConfText() {
  // Lo que mostrará el display cuando se configure temperatura
  setLcdCursor(0, 0); 
  lcdString("tMax=");
  lcdNumber(tMax);
  writeCommand(DATA, 0xDF);

  setLcdCursor(1, 0); 
  lcdString("tMin=");
  lcdNumber(tMin);
  writeCommand(DATA, 0xDF);

}

void HumControlConfText() {
  // Lo que mostrará el display cuando se configure humedad
  lcdString("hMax=");
  lcdNumber(hMax);
  lcdString("%");

  setLcdCursor(1, 0); 
  lcdString("hMin=");
  lcdNumber(hMin);
  lcdString("%");

}

/*
  state -> 1 Configura temp max
  state -> 2 Configura temp min
  state -> 3 Configura hum max
  state -> 4 Configura hum min
*/
ISR(INT0_vect) {
  //cambio de estados
  state += 1;
  if (state >= 5) {
    state = 0;
  }
}

ISR(TIMER0_COMPA_vect){
  counter++;
  if(counter >= 120){
    t = dht.readTemperature();
    h = dht.readHumidity();
    if(isInConfiguration){
       UpdateText();
    }
    counter = 0;
  }
}



void lcdStart()
{
  _delay_ms(16);  //wait for LCD startup: 15ms

  writeCommand( CONTROL, 0x20 );  // function set: 4-bit bus
  _delay_us(45);    //execution time is 40us
  // this instruction will be handled as 8-bit bus. However the function assumes 4-bit bus
  // this will be ok, because the lower bits will be ignored by the lcd when it is busy

  writeCommand( CONTROL, 0x20 );  // function set: 4-bit bus, 1 lines, 5x8 chars
  _delay_us(45);    //execution time is 40us



  writeCommand( CONTROL, 0x0E );  // display on/off control: display on, cursor on, no blink
  _delay_us(45);    //execution time is 40us

  writeCommand( CONTROL, 0x07 );  // entry mode set: increment, no display shift
  _delay_us(45);    //execution time is 40us

  writeCommand( CONTROL, 0x18 );  //display shift: SC = 1 RL = 0
  _delay_us(45);


  writeCommand( CONTROL, 0x01 );  // clear display
  _delay_ms(2);   //execution time is 1.64ms

  return;
}

//-------------------------------------------------------------------------

// write an lcd data/command, 4-bit version
// PB1: RS, PB2: Enable. PC3-PC0: D7-D4
void writeCommand( char RS_type, char CMD )
{
  // set RS line
  if ( RS_type ) PORTB |= 1 << RS;
  else PORTB &= ~(1 << RS);
  _delay_us(1); // min delay is 140ns (t_AS)

  // set enable
  PORTB |= 1 << EN;

  // place high 4-bit data/command on bus
  PORTC = (PORTC & 0xF0) | (CMD >> 4);
  _delay_us(1); // min delay is 195ns (t_DSW)

  // note: min Enable pulse is 450ns (t_PWH). At this point this delay is already met

  // clear enable
  PORTB &= ~(1 << EN);
  _delay_us(1); // delay for next data is 300ns

  // set enable
  PORTB |= 1 << EN;

  // place low 4-bit data/command on bus
  PORTC = (PORTC & 0xF0) | (CMD & 0x0F);
  _delay_us(1); // min delay is 195ns (t_DSW)

  // note: min Enable pulse is 450ns (t_PWH). At this point this delay is already met

  // clear enable
  PORTB &= ~(1 << EN);
  _delay_us(1); // delay is 10ns (t_H)

  return;
}

//-------------------------------------------------------------------------

void lcdString(char text[])
{
  int i, length;

  length = strlen(text);

  for (i = 0; i < length; i++)
  {
    writeCommand( DATA, text[i] );
    _delay_us(45);    //execution time is 40us
  }

  return;
}

//-------------------------------------------------------------------------

void lcdChar(char c)
{
  writeCommand( DATA, c );
  _delay_us(45);    //execution time is 40us

  return;
}

//-------------------------------------------------------------------------

void lcdNumber(int num)
{
  char n[8];

  itoa( num, n, 10);
  lcdString( n );

  return;
}

//-------------------------------------------------------------------------

void lcdFloat(float num)
{
  int i, d;
  float n;

  i = (int) num;  //get integer part
  if ( num < 0 ) n = i - num;   //get fraction
  else n = num - i;
  d = (int)( n * 100 ); //convert fraction to integer: 2 digits

  lcdNumber( i );
  lcdChar( '.' );
  lcdNumber( d );

  return;
}

//-------------------------------------------------------------------------

void setLcdCursor(char line, char col)
{
  char position = 0x00;

  // make sure position is inside limits
  if ( line < 0 ) line = 0; // 2-line lcd
  if ( line > 1 ) line = 1;
  if ( col < 0 ) col = 0;   // 16-char lcd
  if ( col > 15 ) col = 15;

  if ( line == 0 ) position = 0x80 + col;
  if ( line == 1 ) position = 0xC0 + col;

  writeCommand( CONTROL, position );    // set DDRAM address to position
  _delay_us(45);    //execution time is 40us

  return;
}

//-------------------------------------------------------------------------

void clearLcd()
{
  writeCommand( CONTROL, 0x01 );  // clear display
  _delay_ms(2);   //execution time is 1.64ms

  return;
}
