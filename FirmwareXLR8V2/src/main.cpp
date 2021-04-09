/*
XLR8 V2

TODO:
* Agregar control PID
* Agregar control motores
* Agregar posicion
----------------

Pin Mapping:
Physical Pin      Arduino Pin    Port Pin     Function
========================================================
19                A0             PC0          SEN1
20                A1             PC1          SEN2
21                A2             PC2          SEN3
22                A3             PC3          SEN4
23                A4             PC4          SEN5
24                A5             PC5          SEN6
25                A6             PC0          SEN7
26                A7             PC1          SEN8
16                D13            PB5          LED_ON_SENSOR
13                D10            PB2          PWM_TURBINA
05                D2             PD2          M_START         PCINT18
10                D7             PD7          BTN_CALIBRA     PCINT23
11                D8             PB0          LED_VERDE
12                D9             PB1          LED_ROJO
06                D3             PD3          PWMA
14                D11            PB3          PWMB
07                D4             PD4          AIN2
08                D5             PD5          AIN1
09                D6             PD6          BIN1
15                D12            PB4          BIN2
02                D0             PD0          BT_TX
01                D1             PD1          BT_RX
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <ADC.h>

// Pines direccion puente h
#define MOT_DER_ADELANTE  PB4
#define MOT_DER_ATRAS     PD6
#define MOT_IZQ_ADELANTE  PD5
#define MOT_IZQ_ATRAS     PD4

// Estados
#define INICIALIZADO          0
#define CALIBRANDO_SENSORES   1
#define PARADO                2
#define RASTREANDO            3

volatile uint8_t estado = INICIALIZADO;

// Tiempo de estrober
#define stro1interval1  50     // interval at which to blink on(milliseconds)
#define stro1interval2  100    // interval at which to blink off(milliseconds)
#define stro1interval3  150    // interval at which to blink on(milliseconds)
#define stro1interval4  1800   // interval at which to blink off(milliseconds)

// Blink leds
unsigned long t = 0;
unsigned long t_blink = 0;


volatile uint16_t sensores[8];
uint16_t lecturaFondoMax[100];
long int sumap;
long int suma;
long int pos;

uint16_t umbral;
boolean linea = true; // true = Linea Negra // false = Linea Blanca


void calibrar();
void stroberOn();
void blinkOn();
void leerSensores();


void setup() {
  cli();
  Serial.begin(9600);

  //CONFIGURAR ADC (SENSORES IR)
  ADCInit();
  PORTB |= (1 << PORTB5); // LED ON SENSOR

  //CONFIGURACION PINES DE SALIDA
  DDRB = 0xFF; // Todos los pines del puerto B
  PORTB = 0x00; // OFF ALL
  DDRD |= (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6); //Pin 3 del puerto D ,//Pin 4 del puerto D, Pin 5 del puerto D, Pin 6 del puerto D

  //CONFIGURACION PINES DE ENTRADA
  DDRD &= ~((1 << DDD7) | (1 << DDD2)); // Pin 7 del puerto D como entrada btn_calibra, Pin 2 del puerto D como entrada Modulo_arranque

  //CONFIGURAR INTERRUPCION
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT23); //M_START BTN_START

  sei();
}

void loop() {
  switch (estado) {
  case INICIALIZADO:
    stroberOn();
    PORTD &= ~((1 << MOT_IZQ_ATRAS) | (1 << MOT_IZQ_ADELANTE) | (1 << MOT_DER_ATRAS)); // MOTOR OFF
    PORTB &= ~(1 << MOT_DER_ADELANTE); // MOTOR OFF
    break;
  case CALIBRANDO_SENSORES:
    PORTB &= ~(1 << PORTB0); // LED GREEN OFF
    PORTB |= (1 << PORTB1); // LED RED ON
    calibrar();
    PORTB |= (1 << PORTB0); // LED GREEN ON
    PORTB &= ~(1 << PORTB1); // LED RED OFF
    break;
  case PARADO:
    blinkOn();
    PORTD &= ~((1 << MOT_IZQ_ATRAS) | (1 << MOT_IZQ_ADELANTE) | (1 << MOT_DER_ATRAS)); // MOTOR OFF
    PORTB &= ~(1 << MOT_DER_ADELANTE); // MOTOR OFF
    break;
  case RASTREANDO:
    break;
  }


}

ISR(PCINT2_vect) {
  static uint8_t previousValuePD2 = (PIND & (1 << PIND2)); // M START
  static uint8_t previousValuePD7 = (PIND & (1 << PIND7)); // BTN CALIBRA
  _delay_ms(20); //debounce
  uint8_t actualValuePD2 = (PIND & (1 << PIND2));
  uint8_t actualValuePD7 = (PIND & (1 << PIND7));

  // M START
  if ((previousValuePD2 != actualValuePD2)) { // from 5V to 0V Falling PARADO : 0V to 5V Rising RASTREANDO
    actualValuePD2 == 0 ? estado = PARADO : estado = RASTREANDO;
  }

  // BTN CALIBRA
  if ((previousValuePD7 != actualValuePD7) && (actualValuePD7 == 0)) { // from 5v to 0v Falling estado ++
    estado < 4 ? estado++ : estado = 0;
  }
}

// calibrar sensores en fondo
void calibrar() {
  int total = 0;
  static unsigned long previousMillis = 0;

  if ((millis() - previousMillis) > 15) {
    for (uint8_t i = 0; i < 100; i++)
    {
      for (uint8_t j = 0; j < 8; j++)
      {
        if (ADCGetData(j) > lecturaFondoMax[i]) { // Calcular maximos
          lecturaFondoMax[i] = ADCGetData(j);
        }
        //Serial.print(lecturaFondo[i][j]);
        //Serial.print("\t");
      }
      Serial.println(lecturaFondoMax[i]);
      //Serial.println("");
      total = total + lecturaFondoMax[i];
    }
    umbral = uint16_t(total / 20) + 20; // lectura erronea 20 puntos por arriba
    Serial.println(umbral);
    previousMillis += 15;
  }
}

// return sensor 1(linea) o 0(fondo)
void leerSensores() {
  for (uint8_t i = 0; i < 8; i++)
  {
    //sensores[i] = ADCGetData(i); // Analog values
    ADCGetData(i) <= umbral ? linea ? sensores[i] = 0 : sensores[i] = 1 : linea ? sensores[i] = 1 : sensores[i] = 0;
    //Serial.print(sensores[i]);
    //Serial.print("\t");
  }
  //Serial.println("");
}

// calcular posicion
void calcPosicion() {

}

// strober led green and red
void stroberOn() {
  uint8_t stro1num = 1;             // ledState used to set the LED
  unsigned long prestro1Millis = 0; // will store last time LED was updated

  // check to see what strobe count you are on to see what interval is needed
  unsigned long curstro1Millis = millis();  // set time to now

  if (stro1num == 1 && curstro1Millis - prestro1Millis > stro1interval1) {// 1st flash on
    PORTB |= (1 << PORTB1) | (1 << PORTB0); // LED RED ON | LED GREEN ON
    stro1num = 2;
  }
  if (stro1num == 2 && curstro1Millis - prestro1Millis > stro1interval2) {// 1st flash off
    PORTB &= ~((1 << PORTB1) | (1 << PORTB0)); // LED RED OFF | LED GREEN OFF
    stro1num = 3;
  }
  if (stro1num == 3 && curstro1Millis - prestro1Millis > stro1interval3) {//2nd flash on
    PORTB |= (1 << PORTB1) | (1 << PORTB0); // LED RED ON | LED GREEN ON
    stro1num = 4;
  }
  if (stro1num == 4 && curstro1Millis - prestro1Millis > stro1interval4) { //2nd flash off
    PORTB &= ~((1 << PORTB1) | (1 << PORTB0)); // LED RED OFF | LED GREEN OFF
    stro1num = 1;
    prestro1Millis = curstro1Millis;  // set the time to start loop over
  }
}

// blink led green
void blinkOn() {
  t = millis();
  if (t - t_blink < 250)
    PORTB &= ~(1 << PORTB0); // LED GREEN OFF
  else if (t - t_blink < 500)
    PORTB |= (1 << PORTB0); // LED GREEN ON
  else
    t_blink = t;
}

//