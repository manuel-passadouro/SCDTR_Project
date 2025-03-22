#ifndef NODE_TIMERS_H
#define NODE_TIMERS_H

#include <Arduino.h>
#include "pico/stdlib.h"

// Defined in main.cpp
extern const int CONTROL_PERIOD;
extern const int CAN_PERIOD;

extern volatile bool control_timer_flag;
extern volatile bool can_timer_flag;

// Declare the timer setup function
void control_timer_setup();
void can_timer_setup();

// Declare the timer callback function
bool control_timer_callback(repeating_timer_t *rt);
bool can_timer_callback(repeating_timer_t *rt);

#endif
