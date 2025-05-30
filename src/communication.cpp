#include "communication.h"

void can_setup(pico_unique_board_id_t board_id) {
    // Set node address from the unique Pico ID
    node_address = board_id.id[7];
    // Initialize CAN
    can0.reset();
    can0.setBitrate(CAN_1000KBPS);
    can0.setNormalMode(); // Use setLoopbackMode() for debugging if needed
    
}

void CAN_send(int node_id, float data_to_send) {
    canMsgTx.can_id = node_id; // Use correct node ID
    canMsgTx.can_dlc = sizeof(data_to_send); // Number of data bytes to send

    union {
        float data;
        uint8_t bytes[sizeof(float)];
    } dataUnion;

    dataUnion.data = data_to_send;

    for (int i = 0; i < canMsgTx.can_dlc; i++) {
        canMsgTx.data[i] = dataUnion.bytes[i];
    }

    err = can0.sendMessage(&canMsgTx);

    //Debug
    //snprintf(printbuf, BUFSZ, "#%d TXmessage: %ld lux: %f", node_id, counter++, data_to_send);
    //Serial.println(printbuf);
    
}

void CAN_receive(int node_id) {
    union {
        float value;        // To interpret the 4 bytes as a float
        byte data[4];       // To store 4 bytes of data
    } ldr_lux_rx;

    unsigned long rx_msg = 0;
    unsigned long mult = 1;
    // Assuming that the first 4 bytes of the message are the float
    for (int i = 0; i < 4; i++) {
        ldr_lux_rx.data[i] = canMsgRx.data[i];  // Copy the bytes into the union
    }

    // Debug
    //snprintf(printbuf, BUFSZ, "#%d RXmessage lux: %f\n", node_id, rx_msg, ldr_lux_rx.value);
    //Serial.println(printbuf);
    
}