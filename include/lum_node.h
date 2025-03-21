#ifndef LUM_NODE_H
#define LUM_NODE_H
#include <Arduino.h>
#include "mcp2515.h"
#include "control.h"

// Constants (Defined in main)
extern const int LED_PIN;
extern const float VCC;
extern const int R;
extern const int DAC_RANGE_HIGH;
extern const int NUM_SAMPLES;
extern const int CAL_SAMPLES;
extern const int CAL_STEPS;

// External variables (defined in main)
extern int adc_samples[];
extern int sample_index;
extern long sum;


typedef union {
    float value;
    uint8_t bytes[sizeof(float)];         // Useful for CAN transmission
} FloatUnion;

typedef struct {                          // Structure for node data
    int node_id;                          // Friendly ID of the node
    pico_unique_board_id_t board_id;      // Unique board ID
    FloatUnion ldr_m;                     // Slope of the R_LDR/LUX characteristic
    FloatUnion ldr_b;                     // Intercept of the R_LDR/LUX characteristic
    FloatUnion ldr_lux;                   // Latest measured LUX value
    FloatUnion G;                         // Static Gain
    bool desk_state;                      // Desk cupied (1)/ Not ocupied (0)
    int raw_adc;                          // Raw ADC value
    int filtered_adc;                     // Filtered ADC value
    int duty_cycle;                       // Duty cycle for LED
    FloatUnion c_voltage;                 // Voltage at the Capacitior
    FloatUnion ldr_resistance;            // Calculated LDR resistance
} NodeData;

class Node {
    private:
        NodeData node_data;  // Private node data
        pid controller;      // PID controller
    
    public:
        // Constructor: Initialize NodeData and PID controller with default values.
        Node(float h = 0.01f, float K = 1.0f, float b = 0.0f, float Ti = 0.05f)
            : controller(h, K, b, Ti) {  
            node_data.node_id = 0;
            node_data.board_id = {0}; 
            node_data.ldr_m.value = -0.8f;
            node_data.ldr_b.value = 6.15f;
            node_data.ldr_lux.value = 0.0f;
            node_data.G.value = 0.0f;
            node_data.desk_state = false;
            node_data.raw_adc = 0;
            node_data.filtered_adc = 0;
            node_data.duty_cycle = 0;
            node_data.c_voltage.value = 0.0f;
            node_data.ldr_resistance.value = 0.0f;
        }
    
        //Menber Functions
        void get_pico_id() {
            pico_get_unique_board_id(&node_data.board_id);
        }

        void set_controller_params();
    
        void calibrate_gain(int numSteps);

        void get_samples(float* V_LED, float* LUX);
    
        void print_setup_data() const;
        
        void update_ldr_data();

        // Provide access to controller
        void update_control(float r, float y) {
            float u;    
            node_data.duty_cycle = (int)controller.compute_control(r, y);
            controller.housekeep(r, y);            
        }

        void update_led() {
            if (node_data.duty_cycle >= 0 && node_data.duty_cycle <= DAC_RANGE_HIGH){
                analogWrite(LED_PIN, node_data.duty_cycle);
                Serial.print("New duty cycle set: ");
                Serial.println(node_data.duty_cycle);
            }
            else{
                Serial.println("Invalid value. Duty Cycle must be between 0 and 4095.");
            }    
        }

        //Getter Functions
        float get_ldr_lux() const {
            return node_data.ldr_lux.value;
        }
        
        int get_led_duty_cycle() const {
            return node_data.duty_cycle;
        }

        int get_node_id() const {
            return node_data.node_id;
        }

        pico_unique_board_id_t get_board_id() const {
            return node_data.board_id;
        }
        
        //Setter Functions
        void set_ldr_lux(float lux) {
            node_data.ldr_lux.value = lux;
        }

    };
    
int getFilteredADC(int newSample);

void get_samples(float* V_LED, float* LUX);

#endif //LUM_NODE_H