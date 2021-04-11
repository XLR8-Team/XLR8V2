/*
 *	PWM.c
 *	PWM for timer 2 7812 Hz
 *	Created: 09/04/2021 14:15:27
 *	Author: Nicolas
 */

#include "PWM.h"

void PWM_init()
{
	//Operation Mode = Fast PWM
	TCCR2A |= (1 << WGM21);	//1
	TCCR2A |= (1 << WGM20);	//1

}

void PWM_on()
{
	TCNT2 = 0x0000;
	//N = 8  Prescaler Timer 2 7812.5FHz
	TCCR2B &= ~(1 << CS20); //0
	TCCR2B |= (1 << CS21);	//1
	TCCR2B &= ~(1 << CS22); //0
}

void PWM_off()
{
	//Clock setting T1 clock = 0 Hz
	TCCR2B &= ~(1 << CS20); //0
	TCCR2B &= ~(1 << CS21); //0
	TCCR2B &= ~(1 << CS22); //0
}

void setDutyPWMB(int duty)
{
	TCCR2A &= ~(1 << COM2A0);	//0
	TCCR2A |= (1 << COM2A1);	//1

	OCR2A = duty;
}

void setDutyPWMA(int duty)
{
	TCCR2A &= ~(1 << COM2B0); //0
	TCCR2A |= (1 << COM2B1);	//1

	OCR2B = duty;
}
