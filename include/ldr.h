#ifndef LDR_H
#define LDR_H

#include <Arduino.h> 
#include "data.h"

// Constants (Defined in main)
extern const int LED_PIN;
extern const float VCC;
extern const int R;
extern const int DAC_RANGE_HIGH;
extern const int numSamples;
extern const int numSteps;


// Prototypes
int getFilteredADC(int newSample);
void calibrate_gain();
float get_ldr_data();

#endif // LDR_H
