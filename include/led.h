#ifndef LED_H
#define LED_H

#include <Arduino.h>

// Constants (Defined in main)
extern const int LED_PIN;
extern const int DAC_RANGE_LOW;
extern const int DAC_RANGE_HIGH;

void cmd_led_power();

#endif // LED_H