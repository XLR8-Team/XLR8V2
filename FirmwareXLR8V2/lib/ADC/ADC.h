/*
 * ADC.h
 *
 * Created: 08/04/2021 06:43:42 p.m.
 * Author: Nicolas
 */

#include <avr/io.h>
#include <util/delay.h>

#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

  void ADCInit();
  uint16_t ADCGetData(uint8_t canal);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H_ */