#ifndef DATA_H
#define DATA_H
#include <Arduino.h>
#include "mcp2515.h"

typedef union {
    float value;
    uint8_t bytes[sizeof(float)];         // Useful for CAN transmission
} FloatUnion;

typedef struct {                          // Structure for node data
    uint8_t node_id;                      // Short ID of the node
    pico_unique_board_id_t board_id;      // Unique board ID
    FloatUnion m;                         // Slope of the R_LDR/LUX characteristic
    FloatUnion b;                         // Intercept of the R_LDR/LUX characteristic
    FloatUnion ldr_lux;              // Latest measured LUX value
    FloatUnion G;                         // Static Gain
    bool desk_state;                      // Desk cupied (1)/ Not ocupied (0)
    //TODO: add more node data
} NodeData;

#endif //DATA_H