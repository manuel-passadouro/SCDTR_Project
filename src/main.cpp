#include <Arduino.h>
#include "mcp2515.h"
#include "communication.h"
#include "lum_node.h"

// Circuit Parameters
const float VCC = 3.3;              // Rpi Pico Supply voltage
const int R = 10000;                // Voltage Divider Resistor (10kΩ)

// LED
const int LED_PIN = 15;
const int DAC_RANGE_LOW = 0;
const int DAC_RANGE_HIGH = 4096;

//ADC Filter
const int NUM_SAMPLES = 100;         // Number of samples for averaging
int adc_samples[NUM_SAMPLES] = {0};   // Array to store ADC readings
int sample_index = 0;                // Index for storing new samples
long sum = 0;                       // Sum of samples for averaging

// Calibration
const int CAL_SAMPLES = 500;            // Number of duty cycle steps for calibration
const int CAL_STEPS = 10;            // Number of duty cycle steps for calibration

// CAN
uint8_t node_address;                   // short id
struct can_frame canMsgTx, canMsgRx;
unsigned long counter{0};
MCP2515::ERROR err;
int unsigned long time_to_write;
unsigned long write_delay{1000};
const int BUFSZ = 100;
char printbuf[BUFSZ];
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};

//Luminary Node Vector
std::vector<Node> nodes;

void setup() {
    // Serial setup
    Serial.begin(115200);
    Serial.println("setup");

    // DAC/ADC setup
    analogReadResolution(12);         // default is 10
    analogWriteFreq(10000);           // Does not allow less than 100Hz
    analogWriteRange(DAC_RANGE_HIGH); // 100% duty cycle

    nodes.emplace_back();             // Create new node
    nodes[0].get_pico_id();
    nodes[0].calibrate_gain(CAL_STEPS);
    nodes[0].set_controller_params();
    nodes[0].print_setup_data();

    can_setup(nodes[0].get_board_id());
    delay(1000);                      // Wait for monitor to open before completing setup
   
}

void loop(){ 
    
    float r = 1000; // Reference for the controller

    // Read the LDR data
    nodes[0].update_ldr_data();
    //Timer for control
    nodes[0].update_control(r,nodes[0].get_ldr_lux());
    nodes[0].update_led();

    // Actuate LED Manually

    //Control will go here (use timer)

    // Send CAN message periodically (non-blocking)
    unsigned long current_time = millis();                
    
    if( current_time >= time_to_write ) {
        CAN_send(current_time, nodes[0].get_node_id(), nodes[0].get_ldr_lux());   
    }

    // Do this with ISR and flag 
    while ( (err = can0.readMessage( &canMsgRx )) == MCP2515::ERROR_OK ) {
        CAN_receive(nodes[0].get_node_id());
    }

    if (err == MCP2515::ERROR_FAIL) {
        Serial.println("CAN Error");
    }

}