/*
 *	PWM.c
 *	PWM for timer 2
 *	Created: 09/04/2021 14:15:27
 *	Author: Nicolas
 */

#include "PWM.h"
int _freq = 0;

void PWM_init(int freq)
{
	//Operation Mode = Fast PWM
	TCCR2A |= (1 << WGM21);	//1
	TCCR2A |= (1 << WGM20);	//1
	TCCR2B |= (1 << WGM22);	//1

	//Compare value
	_freq = freq;
	OCR0A = (F_CPU / 1024 / _freq) - 1; //top reg
}

void PWM_on()
{
	TCNT2 = 0x0000;
	//N = 1024 Timer 2
	TCCR2B |= (1 << CS20);	//1
	TCCR2B |= (1 << CS21);	//1
	TCCR2B |= (1 << CS22);	//1
}

void PWM_off()
{
	//Clock setting T1clock = 0 Hz
	TCCR2B &= ~(1 << CS20); //0
	TCCR2B &= ~(1 << CS21); //0
	TCCR2B &= ~(1 << CS22); //0
}

void setDutyPWMB(int duty)
{
	//Pin configuration
	//DDRB |= (1<<DDB1);

	//Output active
	TCCR2A &= ~(1 << COM2A0);	//0
	TCCR2A |= (1 << COM2A1);	//1

	OCR2A = (((F_CPU / 1024 / _freq) - 1) * duty) / 100;
}

void setDutyPWMA(int duty)
{
	//Pin configuration
	//DDRB |= (1<<DDB2);

	//Output active
	TCCR2A &= ~(1 << COM2B0); //0
	TCCR2A |= (1 << COM2B1);	//1

	OCR2B = (((F_CPU / 1024 / _freq) - 1) * duty) / 100;
}
