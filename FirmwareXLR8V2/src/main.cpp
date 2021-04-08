/*
XLR8 V2

TODO:
----------------

Pin Mapping:
Physical Pin   Arduino Pin    Port Pin     Function
======================================
19                A0             PC0          SEN1
20                A1             PC1          SEN2
21                A2             PC2          SEN3
22                A3             PC3          SEN4
23                A4             PC4          SEN5
24                A5             PC5          SEN6
25                A6             PC0          SEN7
26                A7             PC1          SEN8
16                D13            AAA          LED_ON_SENSOR
13                D10            PB1          PWM_TURBINA
05                D2             PB2          M_START
10                D7             PB5          BTN_CALIBRA
11                D8             PB3          LED_VERDE
12                D9             PB4          LED_ROJO
06                D3                          PWMA
14                D11                         PWMB
07                D4                          AIN2
08                D5                          AIN1
09                D6                          BIN1
15                D12                         BIN2
02                D0                          BT_TX
01                D1                          BT_RX
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ADC.h>

#define blanco 1
#define negro 0

uint16_t sensores[8];
uint16_t umbral = 50;
uint8_t linea = negro;

void setup()
{
  cli();
  Serial.begin(9600);
  ADCInit();
  DDRB |= (1 << DDB5); // pin 13 salida
  DDRB |= (1 << DDB0); // pin 8 salida
  DDRB |= (1 << DDB1); // pin 9 salida

  PORTB |= (1 << PORTB5); // LED ON SENSOR
  //PORTB |= (1 << PORTB0); // LED GREEN ON
  //PORTB |= (1 << PORTB1); // LED RED ON

  sei();
}

void loop()
{

  static unsigned long previousMillis = 0;

  if ((millis() - previousMillis) > 200) {

    for (uint8_t i = 0; i < 8; i++)
    {
      sensores[i] = ADCGetData(i);
      //ADCGetData(i) <= umbral ? sensores[i] = 0 : sensores[i] = 1;
      Serial.print(sensores[i]);
      Serial.print("\t");
    }
    Serial.println("");

    previousMillis += 200;
  }

}