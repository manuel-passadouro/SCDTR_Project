#include "communication.h"

// External Varibles (defined in main)
extern uint8_t node_address; // Short ID
extern struct can_frame canMsgTx, canMsgRx;
extern unsigned long counter;
extern MCP2515::ERROR err;
extern unsigned long time_to_write;
extern unsigned long write_delay;
extern const int BUFSZ;
extern char printbuf[];
extern MCP2515 can0;

void can_setup(pico_unique_board_id_t* pico_board_id) {
    // Set node address from the unique Pico ID
    node_address = pico_board_id->id[7]; // Assign last byte of ID

    // Initialize CAN
    can0.reset();
    can0.setBitrate(CAN_1000KBPS);
    can0.setNormalMode(); // Use setLoopbackMode() for debugging if needed

    // Initialize timing for message transmission
    unsigned long current_time = millis();
    time_to_write = current_time + write_delay;
}