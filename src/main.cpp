#include <Arduino.h>

const int LED_PIN = 15;
const int DAC_RANGE_LOW = 0;
const int DAC_RANGE_HIGH = 4096;

const float VCC = 3.3; //Rpi Pico Supply voltage
const int R = 10000; // Voltage Divider Resistor (10kΩ)

const float m = -0.8;  // Computed from LDR Datasheet Parameters
const float b = 6.19794; 

int dutyCycle = 0; // Default duty cycle

void setup() {// the setup function runs once
 Serial.begin(115200);
 Serial.println("setup");
 analogReadResolution(12); //default is 10
 analogWriteFreq(10000); //Does not allow less than 100Hz
 analogWriteRange(DAC_RANGE_HIGH); //100% duty cycle

 //Serial.print("DAC_RANGE_LOW DAC_RANGE_HIGH read_adc counter");
}

void loop()
{ // the loop function runs cyclically
    int read_adc;
    if (Serial.available() > 0)
    {
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

    analogWrite(LED_PIN, dutyCycle); // Actuate LED (PWM)
    delay(1);                        // Small delay to prevent excessive CPU usage
    read_adc = analogRead(A0);       // read analog voltage

    
    float V = (read_adc / 4095.0) * VCC; // Calculate the ADC input voltage

    
    float R_LDR = R * ((VCC - V) / V); // Calculate the LDR resistance 
                                       
    
    float LUX = pow(10, (log10(R_LDR) - b) / m); // Calculate LUX using the logarithmic model

    // Print to terminal (formated for teleplot)
    Serial.print(">ADC_READ:");
    Serial.print(read_adc);
    Serial.print("\n");
    Serial.print(">V:");
    Serial.print(V);
    Serial.print("\n");
    Serial.print(">R_LDR:");
    Serial.print(R_LDR);
    Serial.print("\n");
    Serial.print(">LUX:");
    Serial.print(LUX);
    Serial.print("\n");
}