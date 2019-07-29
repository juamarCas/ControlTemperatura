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
int counter = 0;

int aState;
int aLastState;
//#define led 4
DHT dht(DHTPIN, DHTTYPE);

float t;
float h; //humedad
float tMin = 29; // temperatura mínima
float tMax = 32; // temperatura máxima



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
  
  lcdStart();
  UpdateText();

  aLastState = digitalRead(outputA);

  dht.begin();
  sei();

}

void loop() {

  t = dht.readTemperature();
  h = dht.readHumidity();

  if (t <= tMin) {
    digitalWrite(controlPin, HIGH);
  } else if (t > tMax) {
    digitalWrite(controlPin, LOW);
  }

}




void UpdateText() {
  if (isInConfiguration == false) {
    setLcdCursor(0, 0); // temperatura real
    lcdString("t=");
    lcdNumber(t);
    writeCommand(DATA, 0xDF);

    setLcdCursor(0, 0); // temperatura real
    lcdString("h=");
    lcdNumber(t);
    lcdString("%");
  }else{
    
  }
}


ISR(INT0_vect){
  
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
