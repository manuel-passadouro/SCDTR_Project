#include <Arduino.h>
#include "mcp2515.h"
#include "ldr.h"
#include "led.h"

// Circuit Parameters
const float VCC = 3.3; // Rpi Pico Supply voltage
const int R = 10000; // Voltage Divider Resistor (10kΩ)

// Luminarie ID
int chip_id = 0;
int easy_id = 0;
pico_unique_board_id_t pico_board_id; // full id


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

// Calibration
float m = -0.8;  // LDR Datasheet Parameters (Nominal)
float b = 6.15; 
const int numSteps = 10; // Number of duty cycle steps for calibration
float G;

// LDR
union {
    float value = 0;
    uint8_t bytes[4]; // Float is 4 bytes
} ldr_lux;

// CAN
uint8_t node_address; // short id
struct can_frame canMsgTx, canMsgRx;
unsigned long counter{0};
MCP2515::ERROR err;
int unsigned long time_to_write;
unsigned long write_delay
{1000};
const int BUFSZ = 100;
char printbuf[BUFSZ];
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};



void setup() {
 // Serial setup
 Serial.begin(115200);
 Serial.println("setup");

 // DAC/ADC setup
 analogReadResolution(12); //default is 10
 analogWriteFreq(10000); //Does not allow less than 100Hz
 analogWriteRange(DAC_RANGE_HIGH); //100% duty cycle

 // CAN setup
 pico_get_unique_board_id(&pico_board_id);
 node_address = pico_board_id.id[7]; // check
 can0.reset();
 can0.setBitrate(CAN_1000KBPS);
 can0.setNormalMode(); // setLoopbackMode()debug
 unsigned long current_time = millis();
 time_to_write = current_time + write_delay;

 // Initialize sample array with zeros
 for (int i = 0; i < numSamples; i++)
 {
     adcSamples[i] = 0;
 }

 delay(1000); //Wait for monitor to open before completing setup

 // Start-up calibration
 get_ldr_param(); //This is redundant
 //G = calibrate_gain(); //TODO: get m and b here

}

void loop(){ 
    
    int read_adc;                      
    if (Serial.available() > 0)
    {
        cmd_led_power();
    }

    analogWrite(LED_PIN, dutyCycle); // Actuate LED (PWM)
    delay(1);                        // Small delay to prevent excessive CPU usage

    ldr_lux.value = get_ldr_data();

    //CAN TX and RX
    union {
        float value;        // To interpret the 4 bytes as a float
        byte data[4];   // To store 4 bytes of data
    } ldr_lux_rx;

    unsigned long current_time = millis();
    if( current_time >= time_to_write ) {
        canMsgTx.can_id = node_address;
        canMsgTx.can_dlc = sizeof(ldr_lux); //How many data bytes to send?
        unsigned long div = counter;
        for(int i = 0; i < canMsgTx.can_dlc; i++ ) {
            canMsgTx.data[i] = ldr_lux.bytes[i];    
        }
        err = can0.sendMessage(&canMsgTx);
        snprintf(printbuf, BUFSZ,"#%d TXmessage: %ld lux: %f", easy_id,
            counter++, ldr_lux.value);
        Serial.println(printbuf);
        time_to_write = current_time+write_delay;

    }
    while ( (err = can0.readMessage( &canMsgRx )) == MCP2515::ERROR_OK ) {
        unsigned long rx_msg = 0;
        unsigned long mult = 1;
        // Assuming that the first 4 bytes of the message are the float
        for (int i = 0; i < 4; i++) {
            ldr_lux_rx.data[i] = canMsgRx.data[i];  // Copy the bytes into the union
        }
        /*
        for(int i=0 ; i < canMsgRx.can_dlc ; i++) {
            rx_msg += mult*(canMsgRx.data[i]-'0');mult = mult*10;
        }
        */
        snprintf(printbuf, BUFSZ,"#%d RXmessage %ld lux: %f\n", easy_id,
            rx_msg, ldr_lux_rx.value);
        Serial.println(printbuf);
    }
    if( err == MCP2515::ERROR_FAIL){
        Serial.println("Error");
    }

}