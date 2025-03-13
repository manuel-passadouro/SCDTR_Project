#include "ldr.h"

// External variables (defined in main)
extern int easy_id;
extern float m;
extern float b;
extern int dutyCycle;
extern int adcSamples[];
extern int sampleIndex;
extern long sum;

int getFilteredADC(int newSample) {
    // Remove the oldest sample from sum
    sum -= adcSamples[sampleIndex];

    // Store new sample and add it to sum
    adcSamples[sampleIndex] = newSample;
    sum += newSample;

    // Update sample index (circular buffer)
    sampleIndex = (sampleIndex + 1) % numSamples;

    // Compute moving average
    return sum / numSamples;
}

void get_samples(float* V_LED, float* LUX) {
    const int numReadings = 500; // Number of readings per step
    int filtered_adc = 0;

    for (int step = 0; step <= 10; step++) {
        int dutyCycle = (step * DAC_RANGE_HIGH) / 10; // Increase in steps
        analogWrite(LED_PIN, dutyCycle); // Set PWM duty cycle
                       
        float V_LED_samp = (dutyCycle / 4095.0) * VCC; // Voltage applied to the LED (approximate) 

        for (int i = 0; i < numReadings; i++) {
            int read_adc = analogRead(A0);
            filtered_adc = getFilteredADC(read_adc);
            delay(10); // 1 read every 10ms
        }

        float V = (filtered_adc / 4095.0) * VCC; // Calculate the ADC input voltage
        float R_LDR = R * ((VCC - V) / V); // Calculate the LDR resistance 
        float LUX_samp = pow(10, (log10(R_LDR) - b) / m); // Calculate LUX using the logarithmic model
        
        Serial.print("Duty Cycle: ");
        Serial.print(dutyCycle);
        Serial.print(" - Duty Cycle Voltage: ");
        Serial.print(V_LED_samp, 2);
        Serial.print("V - Filtered LUX: ");
        Serial.print(LUX_samp);
        Serial.print(" - R_LDR: ");
        Serial.print(R_LDR, 2);
        Serial.println(" Ohm");

        V_LED[step] = V_LED_samp;
        LUX[step] = LUX_samp;
    }
}

float calibrate_gain() {
    float* V_LED = new float[numSteps + 1]; // Dynamically allocate memory
    float* LUX = new float[numSteps + 1];  // Dynamically allocate memory

    // Get the samples of V_LED and LUX
    get_samples(V_LED, LUX);

    // Linear Regression to compute the slope and intercept
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i <= numSteps; i++) {
        sumX += V_LED[i];
        sumY += LUX[i];
        sumXY += V_LED[i] * LUX[i];
        sumX2 += V_LED[i] * V_LED[i];
    }

    // Compute the slope and intercept using the least squares method
    float slope = (numSteps + 1) * sumXY - sumX * sumY;
    slope /= (numSteps + 1) * sumX2 - sumX * sumX;
    float intercept = (sumY - slope * sumX) / (numSteps + 1);

    // Print the linear regression results
    Serial.print("Calibration Complete\n");
    Serial.print("Slope (G): ");
    Serial.println(slope, 5);
    Serial.print("Intercept: ");
    Serial.println(intercept, 5);

    // Free allocated memory
    delete[] V_LED;
    delete[] LUX;

    return slope;
}

void get_ldr_param(){
    
    // Get RP2040 Unique Chip ID
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);

    // Convert the chip ID to a string
    char chipID[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];       // Hex string buffer
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++)
    {
        sprintf(&chipID[i * 2], "%02X", id.id[i]);
    }

    // Switch case using chip ID string
    if (strcmp(chipID, "E66118604B1F4021") == 0)        // Luminarie 1
    { 
        easy_id = 1;
        m = -0.95;
        b = 6.2;
    }
    else if (strcmp(chipID, "E660C0D1C71DA034") == 0)   // Luminarie 2
    { 
        easy_id = 2;
        m = -0.93;
        b = 6.15;
    }
    else
    {
        // Default to nominal values (defined in main)
    }

    Serial.print("Luminary: "); 
    Serial.print(easy_id);
    Serial.print(" (RP2040 Chip ID: ");
    Serial.print(chipID);
    Serial.println(")");
    Serial.print("Using m = "); 
    Serial.println(m, 5);
    Serial.print("Using b = "); 
    Serial.println(b, 5);
}

float get_ldr_data() {
    int read_adc = analogRead(A0);                  // Read analog voltage
    int filtered_adc = getFilteredADC(read_adc);    // Apply filtering
    
    float V = (filtered_adc / 4095.0) * VCC;        // Calculate ADC input voltage
    float R_LDR = R * ((VCC - V) / V);              // Calculate LDR resistance
    float LUX = pow(10, (log10(R_LDR) - b) / m);    // Calculate LUX using logarithmic model

    // Print to terminal (formatted for teleplot)
    Serial.print(">ADC_READ:");
    Serial.print(filtered_adc);
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

    return LUX;
}

