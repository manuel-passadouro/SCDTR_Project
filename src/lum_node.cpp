#include "lum_node.h"

int getFilteredADC(int newSample) {
    // Remove the oldest sample from sum
    sum -= adc_samples[sample_index];

    // Store new sample and add it to sum
    adc_samples[sample_index] = newSample;
    sum += newSample;

    // Update sample index (circular buffer)
    sample_index = (sample_index + 1) % NUM_SAMPLES;

    // Compute moving average
    return sum / NUM_SAMPLES;
}

void Node::get_samples(float* V_LED, float* LUX) {

    int filtered_adc = 0;
    float m = node_data.ldr_m.value;
    float b = node_data.ldr_b.value;

    for (int step = 0; step <= 10; step++) {
        int dutyCycle = (step * DAC_RANGE_HIGH) / 10; // Increase in steps
        analogWrite(LED_PIN, dutyCycle); // Set PWM duty cycle
                       
        float V_LED_samp = (dutyCycle / 4095.0) * VCC; // Voltage applied to the LED (approximate) 

        for (int i = 0; i < CAL_SAMPLES; i++) {
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

void Node::calibrate_gain(int numSteps) {
    float* V_LED = new float[numSteps + 1]; 
    float* LUX = new float[numSteps + 1];

    // Convert the chip ID to a string
    char chipID[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];       // Hex string buffer
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        sprintf(&chipID[i * 2], "%02X", node_data.board_id.id[i]);
    }

    // Configure node based on chip ID
    if (strcmp(chipID, "E66118604B1F4021") == 0) {  
        node_data.node_id = 1;
        node_data.ldr_m.value = -0.95f;
        node_data.ldr_b.value = 6.2f;
        node_data.G.value = 8.0f; // Default G if not calibration is run
    } 
    else if (strcmp(chipID, "E660C0D1C71DA034") == 0) {  
        node_data.node_id = 2;
        node_data.ldr_m.value = -0.93f;
        node_data.ldr_b.value = 6.15f;
        node_data.G.value = 6.5f; // Default G if not calibration is run
    } 
    else {
        Serial.println("Unknown board ID. Using default calibration values.");
    }

    // Debug Toggle for calibration
    if(0){
        
        get_samples(V_LED, LUX);

        // Perform Linear Regression to compute slope and intercept
        float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (int i = 0; i <= numSteps; i++) {
            sumX += V_LED[i];
            sumY += LUX[i];
            sumXY += V_LED[i] * LUX[i];
            sumX2 += V_LED[i] * V_LED[i];
        }

        // Compute slope and intercept using the least squares method
        float slope = ((numSteps + 1) * sumXY - sumX * sumY) /
                    ((numSteps + 1) * sumX2 - sumX * sumX);
        float intercept = (sumY - slope * sumX) / (numSteps + 1);

        node_data.G.value = slope;

        // Free allocated memory
        delete[] V_LED;
        delete[] LUX;

        Serial.println("Calibration complete.");
        Serial.println();
    }
}

void Node::print_setup_data() const {
    Serial.print("Hello, this is node ");
    Serial.println(node_data.node_id);

    // Print the Chip ID in hex format
    Serial.print("Chip ID: ");
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        Serial.print(node_data.board_id.id[i], HEX);
        if (i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 1) {
            Serial.print(":");
        }
    }
    Serial.println();

    // Print LDR data and calibration values
    Serial.print("LDR Slope (m): ");
    Serial.println(node_data.ldr_m.value, 5);
    Serial.print("LDR Intercept (b): ");
    Serial.println(node_data.ldr_b.value, 5);
    Serial.print("Slope (G): ");
    Serial.println(node_data.G.value, 5);

    // Print controller parameters
    /*
    Serial.println("Controller Parameters:");
    Serial.print("K: ");
    Serial.println(controller.getK(), 5);
    Serial.print("b: ");
    Serial.println(controller.getB(), 5);
    Serial.print("Ti: ");
    Serial.println(controller.getTi(), 5);
    Serial.print("Td: ");
    Serial.println(controller.getTd(), 5);
    Serial.print("h: ");
    Serial.println(controller.getH(), 5);
    */
    // Print occupancy state
    Serial.print("Desk Occupancy: ");
    Serial.println(node_data.desk_state ? "Occupied" : "Unoccupied");
}

void Node::update_ldr_data() {
    // Read from ADC
    node_data.raw_adc = analogRead(A0);
    node_data.filtered_adc = getFilteredADC(node_data.raw_adc);
    
    // Compute the voltage at the capacitor
    float V = (node_data.filtered_adc / 4095.0) * VCC;
    node_data.c_voltage.value = V;
    
    // Compute the resistance of the LDR
    float R_LDR = R * ((VCC - V) / V);
    node_data.ldr_resistance.value = R_LDR;
    
    // Compute the total and external illuminance
    node_data.ldr_lux.value = pow(10, (log10(R_LDR) - node_data.ldr_b.value) / node_data.ldr_m.value);

    node_data.ldr_lux_extern.value = node_data.ldr_lux.value - (node_data.G.value * (float)(node_data.duty_cycle/ 4096.0));

    // Optional: Print for debugging
    /*
    Serial.print(">ADC_READ:");
    Serial.print(node_data.raw_adc);
    Serial.print("\n>V:");
    Serial.print(node_data.input_voltage.value);
    Serial.print("\n>R_LDR:");
    Serial.print(node_data.ldr_resistance.value);
    Serial.print("\n>LUX:");
    Serial.print(node_data.ldr_lux.value);
    Serial.print("\n");
    */
}

void Node::set_controller_params() {
    switch (node_data.node_id) {
        case 1: // For node 1
            controller.set_K(1.0f);
            controller.set_h(0.01f);
            controller.set_b(1.0f);
            controller.set_Ti(0.05f);
            break;

        case 2: // For node 2
            controller.set_K(1.0f);
            controller.set_h(0.01f);
            controller.set_b(1.0f);
            controller.set_Ti(0.03f);
            break;

        default:
            //Keep default values
            break;
    }
}


