#include "led.h"

extern int dutyCycle; // Default duty cycle (Defined in main)

/*
 * Function: cmd_led_power
 * ------------------------
 * Reads an integer from the serial input and sets the LED duty cycle.
 * The duty cycle must be between 0 and the maximum value (DAC_RANGE_HIGH).
 * If the input is valid, it updates the duty cycle and prints a confirmation message.
 * If the input is invalid, it prompts the user to enter a value between 0 and 4096.
 *
 * Returns: None
 */
void cmd_led_power(){
    int newDutyCycle = Serial.parseInt(); // Read integer from Serial
        if (newDutyCycle >= 0 && newDutyCycle <= DAC_RANGE_HIGH)
        {
            dutyCycle = newDutyCycle;
            Serial.print("New duty cycle set: ");
            Serial.println(dutyCycle);
        }
        else
        {
            Serial.println("Invalid value. Enter a number between 0 and 4096.");
        }
}

