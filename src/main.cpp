#include <Arduino.h>
#include "mcp2515.h"
#include "communication.h"
#include "lum_node.h"
#include "node_timers.h"
#include "interface.h"

// Circuit Parameters
const float VCC = 3.1;              // Rpi Pico Supply voltage (Approx)
const int R = 10000;                // Voltage Divider Resistor (10kΩ)

// LED
const int LED_PIN = 15;
const int DAC_RANGE_LOW = 0;
const int DAC_RANGE_HIGH = 4096;
const float P_MAX = 0.01 * VCC;      // Maximum power (10mA)

//ADC Filter
const int NUM_SAMPLES = 50;          // Number of samples for ADC Filter
int adc_samples[NUM_SAMPLES] = {0};  // Array to store ADC readings
int sample_index = 0;                // Index for storing new samples
long sum = 0;                        // Sum of samples for averaging

// Calibration
const int CAL_SAMPLES = 500;         // Number of duty cycle steps for calibration
const int CAL_STEPS = 10;            // Number of duty cycle steps for calibration

// Control
const int CONTROL_PERIOD = 1000;       // Control period in ms (100Hz)

// CAN
uint8_t node_address;                // short id
struct can_frame canMsgTx, canMsgRx;
unsigned long counter{0};
MCP2515::ERROR err;
int unsigned long time_to_write;
//unsigned long write_delay{1000};
const int CAN_PERIOD = 2000;         // CAN TX period in ms (10Hz)
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

void loop() {
    unsigned long start_time, elapsed_time;

    // Check for serial input
    if (Serial.available()) {
        start_time = micros();  // Start time for handling serial commands
        handle_serial_commands();
        elapsed_time = micros() - start_time;  // Elapsed time
        Serial.print("Serial command processing time: ");
        Serial.println(elapsed_time);
    }

    // Stream Duty cycle and Illuminance if enabled
    if (nodes[0].get_stream_u()) {
        start_time = micros();  // Start time for streaming Duty cycle
        Serial.print("Node ");
        Serial.print(nodes[0].get_node_id());
        Serial.print(" Duty Cycle: ");
        Serial.println(nodes[0].get_led_duty_cycle());
        elapsed_time = micros() - start_time;  // Elapsed time
        Serial.print("Duty cycle streaming time: ");
        Serial.println(elapsed_time);
    }

    if (nodes[0].get_stream_y()) {
        start_time = micros();  // Start time for streaming LDR Lux
        Serial.print("Node ");
        Serial.print(nodes[0].get_node_id());
        Serial.print(" LDR Lux: ");
        Serial.println(nodes[0].get_ldr_lux());
        elapsed_time = micros() - start_time;  // Elapsed time
        Serial.print("LDR Lux streaming time: ");
        Serial.println(elapsed_time);
    }

    // Control
    if (control_timer_flag) {
        control_timer_flag = false;
        
        // Update LDR data independently
        start_time = micros();  // Start time for updating LDR data
        nodes[0].set_timestamp(micros());
        nodes[0].update_ldr_data();
        elapsed_time = micros() - start_time;  // Elapsed time
        Serial.print("LDR data update time: ");
        Serial.println(elapsed_time);

        // Now update control based on the latest LDR data
        start_time = micros();  // Start time for updating control
        nodes[0].update_control(nodes[0].get_reference(), nodes[0].get_ldr_lux());
        nodes[0].update_led();
        elapsed_time = micros() - start_time;  // Elapsed time
        Serial.print("Control update time: ");
        Serial.println(elapsed_time);

        // Update last minute buffer
        data_buffer.add_data(nodes[0].get_node_data());
    }

    // CAN
    if (can_timer_flag) {
        can_timer_flag = false;
        start_time = micros();  // Start time for sending CAN message
        CAN_send(nodes[0].get_node_id(), nodes[0].get_ldr_lux());
        elapsed_time = micros() - start_time;  // Elapsed time
        Serial.print("CAN send time: ");
        Serial.println(elapsed_time);
    }

    // CAN receive
    
    while ((err = can0.readMessage(&canMsgRx)) == MCP2515::ERROR_OK) {
        start_time = micros();  // Start time for receiving CAN messages
        CAN_receive(nodes[0].get_node_id());
        elapsed_time = micros() - start_time;  // Elapsed time
        Serial.print("CAN receive time: ");
        Serial.println(elapsed_time);
    }
    if (err == MCP2515::ERROR_FAIL) {
        Serial.println("CAN Error");
    }
    
}