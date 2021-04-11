/*
 *	PWM.h
 *	PWM for timer 2 7812 Hz
 *	Created: 09/04/2021 14:15:27
 *	Author: Nicolas
 */

#include <avr/io.h>

#ifndef PWM_H_
#define PWM_H_

#ifdef __cplusplus
extern "C" {
#endif
	void PWM_init();
	void PWM_on();
	void PWM_off();

	void setDutyPWMA(int duty);
	void setDutyPWMB(int duty);
#ifdef __cplusplus
}
#endif

#endif /* PWM_H_ */