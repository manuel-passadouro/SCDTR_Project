#ifndef LUM_NODE_H
#define LUM_NODE_H
#include <Arduino.h>
#include "mcp2515.h"
#include "control.h"


// Constants (Defined in main)
extern const int LED_PIN;
extern const float P_MAX;
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
    FloatUnion ldr_lux;                   // Measured illiminance (LUX)
    FloatUnion ldr_lux_extern;            // External illuminance (LUX)
    FloatUnion G;                         // Static Gain
    bool desk_state;                      // Desk cupied (1)/ Not ocupied (0)
    int raw_adc;                          // Raw ADC value
    int filtered_adc;                     // Filtered ADC value
    int duty_cycle;                       // Duty cycle for LED (0 to 4095)
    FloatUnion c_voltage;                 // Voltage at the Capacitior
    FloatUnion ldr_resistance;            // Calculated LDR resistance
    FloatUnion reference;                 // Reference value for control
    unsigned long timestamp;              // Time of last update
    bool stream_u;                        // Stream duty cycle flag
    bool stream_y;                        // Stream lux flag
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
            node_data.G.value = 8.0f;
            node_data.desk_state = false;
            node_data.raw_adc = 0;
            node_data.filtered_adc = 0;
            node_data.duty_cycle = 0;
            node_data.c_voltage.value = 0.0f;
            node_data.ldr_resistance.value = 0.0f;
            node_data.reference.value = 0.0f;
            node_data.timestamp = 0;
            node_data.stream_u = false;
            node_data.stream_y = false;

        }
    
        //Member Functions
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
            float G = node_data.G.value;
            float u = (int)controller.compute_control(r, y, G);    
            node_data.duty_cycle = u;
            controller.housekeep(r, y, u);            
        }

        void update_led() {
            if (node_data.duty_cycle >= 0 && node_data.duty_cycle <= DAC_RANGE_HIGH){
                analogWrite(LED_PIN, node_data.duty_cycle);
            }
            else{
                Serial.println("Invalid value. Duty Cycle must be between 0 and 4095.");
            }    
        }

        //Getter Functions
        NodeData get_node_data() const {
            return node_data;
        }

        float get_ldr_lux() const {
            return node_data.ldr_lux.value;
        }

        float get_ldr_lux_extern() const {
            return node_data.ldr_lux_extern.value;
        }

        float get_c_voltage() const {
            return node_data.c_voltage.value;
        }

        float get_G() const {
            return node_data.G.value;
        }
        
        int get_led_duty_cycle() const {
            return node_data.duty_cycle;
        }

        float get_reference() const {
            return node_data.reference.value;
        }

        int get_node_id() const {
            return node_data.node_id;
        }

        bool get_occupancy() const {
            return node_data.desk_state;
        }

        bool get_stream_u() const {
            return node_data.stream_u;
        }

        bool get_stream_y() const {
            return node_data.stream_y;
        }

        unsigned long get_timestamp() const {
            return node_data.timestamp;
        }

        pico_unique_board_id_t get_board_id() const {
            return node_data.board_id;
        }

        
        
        //Setter Functions
        void set_duty_cycle(int duty_cycle) {
            node_data.duty_cycle = duty_cycle;
       }

        void set_ldr_lux(float lux) {
            node_data.ldr_lux.value = lux;
        }

        void set_reference(float r) {
            node_data.reference.value = r;
        }

        void set_occupancy(bool o) {
            node_data.desk_state = o;
        }

        void set_stream_u(bool state) {
            node_data.stream_u = state;
        }

        void set_stream_y(bool state) {
            node_data.stream_y = state;
        }

        void set_timestamp(unsigned long time) {
            node_data.timestamp = time;
        }

    };
    

class DataBuffer {
    public:
        NodeData *data_buffer;      // Pointer to dynamically allocated array
        int buffer_size;            // Current size of the buffer
        int buffer_head;            // Index for the next insertion
        int buffer_tail;            // Index for the oldest data
            
        // Constructor: Initialize the buffer and size
        DataBuffer(int size) {
            buffer_size = size;
            data_buffer = new NodeData[buffer_size];  // Dynamically allocate memory for buffer
            buffer_head = 0;
            buffer_tail = 0;
        }
    
        // Destructor
        ~DataBuffer() {
            delete[] data_buffer;
        }
    
        // Add new data to the buffer         
        void add_data(NodeData new_data) {
                                    
            // Add the new data point to the buffer
            data_buffer[buffer_head] = new_data;
            buffer_head = (buffer_head + 1) % buffer_size;      // Circular buffer

            // Track buffer tail, for printing in order
            if (buffer_head == buffer_tail) {
                buffer_tail = (buffer_tail + 1) % buffer_size;  // Overwrite the oldest data
            }  
        }       
          
        // Print Variables to monitor

        void print_lux() {
            for (int index = buffer_tail; index != buffer_head; index = (index + 1) % buffer_size) {
                Serial.print("Timestamp: ");
                Serial.print(data_buffer[index].timestamp);
                Serial.print(" | Lux: ");
                Serial.println(data_buffer[index].ldr_lux.value);
            }
        }
        
        void print_filtered_adc() {
            for (int index = buffer_tail; index != buffer_head; index = (index + 1) % buffer_size) {
                Serial.print("Timestamp: ");
                Serial.print(data_buffer[index].timestamp);
                Serial.print(" | Filtered ADC: ");
                Serial.println(data_buffer[index].filtered_adc);
            }
        }

        // Function to print duty cycle values from oldest to newest
        void print_duty_cycle() {
            for (int index = buffer_tail; index != buffer_head; index = (index + 1) % buffer_size) {
                Serial.print("Timestamp: ");
                Serial.print(data_buffer[index].timestamp);
                Serial.print(" | Duty Cycle: ");
                Serial.println(data_buffer[index].duty_cycle);
            }
        }

        // Function to print illuminance values from oldest to newest
        void print_ldr_lux() {
            for (int index = buffer_tail; index != buffer_head; index = (index + 1) % buffer_size) {
                Serial.print("Timestamp: ");
                Serial.print(data_buffer[index].timestamp);
                Serial.print(" | Duty Cycle: ");
                Serial.println(data_buffer[index].ldr_lux.value);
            }
        }

        // Function to print voltage values from oldest to newest
        void print_c_voltage() {
            for (int index = buffer_tail; index != buffer_head; index = (index + 1) % buffer_size) {
                Serial.print("Timestamp: ");
                Serial.print(data_buffer[index].timestamp);
                Serial.print(" | Voltage: ");
                Serial.println(data_buffer[index].c_voltage.value);
            }
        }

        // Function to print ldr resistance values from oldest to newest
        void print_ldr_resistance() {
            for (int index = buffer_tail; index != buffer_head; index = (index + 1) % buffer_size) {
                Serial.print("Timestamp: ");
                Serial.print(data_buffer[index].timestamp);
                Serial.print(" | LDR Resistance: ");
                Serial.println(data_buffer[index].ldr_resistance.value);
            }
        }

        void print_all() {
            for (int index = buffer_tail; index != buffer_head; index = (index + 1) % buffer_size){
                // Print in csv friendly format
                Serial.print(data_buffer[index].timestamp);
                Serial.print(",");
                Serial.print(data_buffer[index].duty_cycle);
                Serial.print(",");
                Serial.print(data_buffer[index].ldr_lux.value, 4);
                Serial.print(",");
                Serial.print(data_buffer[index].reference.value, 4);
                Serial.println();
            }
        }

        void compute_energy() {
            float energy_sum = 0.0;
            if ((buffer_head - buffer_tail + buffer_size) % buffer_size < 2) {
                Serial.println("Not enough data to compute energy.");
                return;
            }
        
            for (int i = (buffer_tail + 1) % buffer_size; i != buffer_head; i = (i + 1) % buffer_size) {
                int prev_index = (i - 1 + buffer_size) % buffer_size;
                float delta_time = (data_buffer[i].timestamp - data_buffer[prev_index].timestamp) / 1000.0; // Convert ms to s
                float scaled_duty_cycle = data_buffer[prev_index].duty_cycle / 4096.0; // Scale from 0-1
                energy_sum += scaled_duty_cycle * delta_time;
            }
        
            float total_energy = P_MAX * energy_sum;
        
            Serial.print("Total Energy (Joules): ");
            Serial.println(total_energy);
        }

        void compute_visibility_error() {
            float visibility_sum = 0.0;
            int sample_count = 0;
        
            for (int i = buffer_tail; i != buffer_head; i = (i + 1) % buffer_size) {
                float error = std::max(0.0f, data_buffer[i].reference.value - data_buffer[i].ldr_lux.value);
                visibility_sum += error;
                sample_count++;
            }
        
            if (sample_count == 0) {
                Serial.println("No data available for visibility error calculation.");
                return;
            }
        
            float visibility_error = visibility_sum / sample_count;
            Serial.print("Average Visibility Error (LUX): ");
            Serial.println(visibility_error);
        }

        void compute_flicker() {
            float flicker_sum = 0.0;
            int sample_count = 0;
        
            for (int i = (buffer_tail + 2) % buffer_size; i != buffer_head; i = (i + 1) % buffer_size) {
                int prev1 = (i - 1 + buffer_size) % buffer_size;
                int prev2 = (i - 2 + buffer_size) % buffer_size;
        
                float d_k = data_buffer[i].duty_cycle;
                float d_k_1 = data_buffer[prev1].duty_cycle;
                float d_k_2 = data_buffer[prev2].duty_cycle;
        
                if ((d_k - d_k_1) * (d_k_1 - d_k_2) < 0) {
                    float flicker_value = std::abs(d_k - d_k_1) + std::abs(d_k_1 - d_k_2);
                    flicker_sum += flicker_value;
                    sample_count++;
                }
            }
        
            if (sample_count == 0) {
                Serial.println("No flicker detected.");
                return;
            }
        
            float average_flicker = flicker_sum / sample_count;
            Serial.print("Average Flicker (1/s): ");
            Serial.println(average_flicker);
        }
};

int getFilteredADC(int newSample);

#endif //LUM_NODE_H