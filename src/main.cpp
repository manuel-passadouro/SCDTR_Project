#include <Arduino.h>
#include "mcp2515.h"
#include "communication.h"
#include "ldr.h"
#include "led.h"
#include "data.h"

// Circuit Parameters
const float VCC = 3.3;              // Rpi Pico Supply voltage
const int R = 10000;                // Voltage Divider Resistor (10kΩ)

// LED
const int LED_PIN = 15;
const int DAC_RANGE_LOW = 0;
const int DAC_RANGE_HIGH = 4096;
int dutyCycle = 0;                  // Default duty cycle

//ADC Filter
const int numSamples = 100;         // Number of samples for averaging
int adcSamples[numSamples] = {0};   // Array to store ADC readings
int sampleIndex = 0;                // Index for storing new samples
long sum = 0;                       // Sum of samples for averaging

//Node data structure
std::vector<NodeData> nodes;

// Calibration
const int numSteps = 10;                // Number of duty cycle steps for calibration

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

void node_setup(){
    
    NodeData local_node = {
        .node_id = 0,                      
        .board_id = {0},                   // board_id, initialize to 0
        .m = { .value = -0.8f },           // m union, initializing the value
        .b = { .value = 6.15f },           // b union, initializing the value
        .ldr_lux = { .value = 0.0f },      // ldr_lux union, initializing the value
        .G = { .value = 0.0f },            // G union, initializing the value
        .desk_state = false                // desk_state
    };

    nodes.push_back(local_node);           // Add the first node to the vector
    pico_get_unique_board_id(&nodes[0].board_id);
    calibrate_gain();

    // Print the linear regression results
    Serial.print("Hello, this is node ");
    Serial.println(nodes[0].node_id);
    Serial.print("Chip ID: ");
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        Serial.print(nodes[0].board_id.id[i], HEX);
        if (i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 1) {
            Serial.print(":");
        }
    }
    Serial.print("Calibration Complete\n");
    Serial.print("Slope (G): ");
    Serial.println(nodes[0].G.value, 5);
    
}

void setup() {
    // Serial setup
    Serial.begin(115200);
    Serial.println("setup");

    // DAC/ADC setup
    analogReadResolution(12);         // default is 10
    analogWriteFreq(10000);           // Does not allow less than 100Hz
    analogWriteRange(DAC_RANGE_HIGH); // 100% duty cycle

    delay(1000);                      // Wait for monitor to open before completing setup

    node_setup();

    can_setup();
   
}

void loop(){ 
    
    int read_adc;  

    // Actuate LED Manually

    if (Serial.available() > 0)
    {
        cmd_led_power();
    }

    analogWrite(LED_PIN, dutyCycle);
    
    // Read the LDR data
    nodes[0].ldr_lux.value = get_ldr_data();

    //Control will go here (use timer)

    // Send CAN message periodically (non-blocking)
    unsigned long current_time = millis();                
    
    if( current_time >= time_to_write ) {
        CAN_send(current_time);   
    }

    // Do this with ISR and flag 
    while ( (err = can0.readMessage( &canMsgRx )) == MCP2515::ERROR_OK ) {
        CAN_receive();
    }

    if (err == MCP2515::ERROR_FAIL) {
        Serial.println("CAN Error");
    }

}