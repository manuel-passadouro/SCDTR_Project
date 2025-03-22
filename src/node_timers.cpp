#include "node_timers.h"

repeating_timer_t control_timer;
repeating_timer_t can_timer;

volatile bool control_timer_flag = false;
volatile bool can_timer_flag = false;

bool control_timer_callback(repeating_timer_t *rt) {
    if (!control_timer_flag) {
        control_timer_flag = true;
    }
    return true;  // Must return ture to keep repeating
}

bool can_timer_callback(repeating_timer_t *rt) {
    if (!can_timer_flag) {
        can_timer_flag = true;
    }
    return true;  // Must return ture to keep repeating
}

void control_timer_setup() {
    
    add_repeating_timer_ms(CONTROL_PERIOD, control_timer_callback, NULL, &control_timer);
}

void can_timer_setup() {
    
    add_repeating_timer_ms(CAN_PERIOD, can_timer_callback, NULL, &can_timer);
}
