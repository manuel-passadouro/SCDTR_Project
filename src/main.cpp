#include <Arduino.h>
#include "ldr.h"
#include "led.h"

// Circuit Parameters
const float VCC = 3.3; // Rpi Pico Supply voltage
const int R = 10000; // Voltage Divider Resistor (10kΩ)

// Luminarie ID
int easy_id = 0;

// LED
const int LED_PIN = 15;
const int DAC_RANGE_LOW = 0;
const int DAC_RANGE_HIGH = 4096;
int dutyCycle = 0; // Default duty cycle

//ADC Filter
const int numSamples = 100;  // Number of samples for averaging
int adcSamples[numSamples]; // Array to store ADC readings
int sampleIndex = 0;        // Index for storing new samples
long sum = 0;               // Sum of samples for averaging

//Calibration
float m = -0.8;  // LDR Datasheet Parameters (Nominal)
float b = 6.15; 
const int numSteps = 10; // Number of duty cycle steps for calibration
float G;

//LDR
float ldr_lux = 0;


void setup() {
 Serial.begin(115200);
 Serial.println("setup");
 analogReadResolution(12); //default is 10
 analogWriteFreq(10000); //Does not allow less than 100Hz
 analogWriteRange(DAC_RANGE_HIGH); //100% duty cycle

 // Initialize sample array with zeros
 for (int i = 0; i < numSamples; i++)
 {
     adcSamples[i] = 0;
 }

 delay(3000); //Wait for monitor to open

 get_ldr_param();
 G = calibrate_gain();

}

void loop()
{ // the loop function runs cyclically
    
    int read_adc;
    if (Serial.available() > 0)
    {
        cmd_led_power();
    }

    analogWrite(LED_PIN, dutyCycle); // Actuate LED (PWM)
    delay(1);                        // Small delay to prevent excessive CPU usage

    ldr_lux = get_ldr_data();

}