#include <Arduino.h>

const int LED_PIN = 15;
const int DAC_RANGE_LOW = 0;
const int DAC_RANGE_HIGH = 4096;
int counter = 0;

void setup() {// the setup function runs once
 Serial.begin(115200);
 analogReadResolution(12); //default is 10
 analogWriteFreq(60000); //60KHz, about max
 analogWriteRange(DAC_RANGE_HIGH); //100% duty cycle

 //Serial.print("DAC_RANGE_LOW DAC_RANGE_HIGH read_adc counter");
}

void loop() {// the loop function runs cyclically
 int read_adc;
 analogWrite(LED_PIN, counter); // set led PWM
 delay(1); //delay 1ms
 read_adc = analogRead(A0); // read analog voltage
 counter = counter + 1;
 if (counter > DAC_RANGE_HIGH) // if counter saturates
 counter = 0; // reset counter
 //format that Serial Plotter likes
 Serial.print("DAC_LOW:");
 Serial.print(DAC_RANGE_LOW); Serial.print(" ");
 Serial.print("DAC_HIGH:");
 Serial.print(DAC_RANGE_HIGH); Serial.print(" ");
 Serial.print("LDR_READ:");
 Serial.print(read_adc); Serial.print(" ");
 Serial.print("Counter:");
 Serial.print(counter); Serial.println(); 
}