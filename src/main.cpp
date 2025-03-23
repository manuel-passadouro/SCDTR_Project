#include <Arduino.h>
#include "mcp2515.h"
#include "communication.h"
#include "lum_node.h"
#include "node_timers.h"
#include "interface.h"

// Circuit Parameters
const float VCC = 3.1;              // Rpi Pico Supply voltage
const int R = 10000;                // Voltage Divider Resistor (10kΩ)

// LED
const int LED_PIN = 15;
const int DAC_RANGE_LOW = 0;
const int DAC_RANGE_HIGH = 4096;

//ADC Filter
const int NUM_SAMPLES = 50;          // Number of samples for ADC Filter
int adc_samples[NUM_SAMPLES] = {0};  // Array to store ADC readings
int sample_index = 0;                // Index for storing new samples
long sum = 0;                        // Sum of samples for averaging

// Calibration
const int CAL_SAMPLES = 500;         // Number of duty cycle steps for calibration
const int CAL_STEPS = 10;            // Number of duty cycle steps for calibration

// Control
const int CONTROL_PERIOD = 10;       // Control period in ms (100Hz)

// CAN
uint8_t node_address;                // short id
struct can_frame canMsgTx, canMsgRx;
unsigned long counter{0};
MCP2515::ERROR err;
int unsigned long time_to_write;
//unsigned long write_delay{1000};
const int CAN_PERIOD = 100;         // CAN TX period in ms (10Hz)
const int BUFSZ = 100;
char printbuf[BUFSZ];
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};

//Luminary Node Vector
std::vector<Node> nodes;

// Last miunute data buffer
DataBuffer data_buffer(3000);           //At 100Hz, 6000 samples = 1 minute

void setup() {
    // Serial setup
    Serial.begin(115200);
    delay(1000);                        // Wait for monitor to open before continuing setup
    Serial.println("setup");

    // DAC/ADC setup
    analogReadResolution(12);           // default is 10
    analogWriteFreq(10000);             // Does not allow less than 100Hz
    analogWriteRange(DAC_RANGE_HIGH);   // 100% duty cycle

    nodes.emplace_back();               // Create new node
    nodes[0].get_pico_id();
    nodes[0].calibrate_gain(CAL_STEPS);
    nodes[0].set_controller_params();
    nodes[0].print_setup_data();

    can_setup(nodes[0].get_board_id()); // Setup the CAN bus

    control_timer_setup();              // Setup the control timer
    can_timer_setup();                  // Setup the CAN timer
         
    Serial.print("CAN SETUP COMPLETE");
    
}

void loop(){ 
    
    //Serial.print("MAIN");
        
    // Check for serial input
    if(Serial.available()){    
        handle_serial_commands();
        //Serial.println("HELLO");
    }

    // Stream Duty cycle and Illuminance if enabled
    if(nodes[0].get_stream_u()){
        Serial.print("Node ");
        Serial.print(nodes[0].get_node_id());
        Serial.print(" Duty Cycle: ");
        Serial.println(nodes[0].get_led_duty_cycle());
    }

    if(nodes[0].get_stream_y()){
        Serial.print("Node ");
        Serial.print(nodes[0].get_node_id());
        Serial.print(" LDR Lux: ");
        Serial.println(nodes[0].get_ldr_lux());
    }

    // Read the LDR data
    //nodes[0].update_ldr_data();
    
    //Control
    if (control_timer_flag) {
        control_timer_flag = false;
        // Enable/Disable control
        nodes[0].set_timestamp(millis());
        nodes[0].update_ldr_data();
        nodes[0].update_control(nodes[0].get_reference(),nodes[0].get_ldr_lux());
        nodes[0].update_led();

        // Update last minute buffer
        data_buffer.add_data(nodes[0].get_node_data());
    }
        
    // CAN                
    if(can_timer_flag) {
        can_timer_flag = false;
        //CAN_send(nodes[0].get_node_id(), nodes[0].get_ldr_lux());
        //Serial.print("Node ");
        //Serial.print(nodes[0].get_node_id());

        Serial.print(">Time:");
        Serial.print(millis());
        Serial.print("\n");
        
        Serial.print(">Duty_Cycle:");
        Serial.print(nodes[0].get_led_duty_cycle());
        Serial.print("\n");

        Serial.print(">LDR_Lux:");
        Serial.print(nodes[0].get_ldr_lux());
        Serial.print("\n");
                
        Serial.print(">Ref:");
        Serial.print(nodes[0].get_reference());
        Serial.print("\n");
    }

    // Do this with ISR and flag 
    while ( (err = can0.readMessage( &canMsgRx )) == MCP2515::ERROR_OK ) {
        CAN_receive(nodes[0].get_node_id());
    }

    if (err == MCP2515::ERROR_FAIL) {
        Serial.println("CAN Error");
    }

}