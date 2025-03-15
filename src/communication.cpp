#include "communication.h"

void can_setup() {
    // Set node address from the unique Pico ID
    //node_address = pico_board_id->id[7]; // Assign last byte of ID
    node_address = nodes[0].board_id.id[7];

    // Initialize CAN
    can0.reset();
    can0.setBitrate(CAN_1000KBPS);
    can0.setNormalMode(); // Use setLoopbackMode() for debugging if needed

    // Initialize timing for message transmission
    unsigned long current_time = millis();
    time_to_write = current_time + write_delay;
}

void CAN_send(unsigned long current_time) {
    canMsgTx.can_id = node_address;
    canMsgTx.can_dlc = sizeof(nodes[0].ldr_lux.value); // Number of data bytes to send
    for (int i = 0; i < canMsgTx.can_dlc; i++) {
        canMsgTx.data[i] = nodes[0].ldr_lux.bytes[i];
    }
    err = can0.sendMessage(&canMsgTx);
    snprintf(printbuf, BUFSZ, "#%d TXmessage: %ld lux: %f", nodes[0].node_id, counter++, nodes[0].ldr_lux.value);
    Serial.println(printbuf);
    time_to_write = current_time + write_delay; // Update the next write time
}

void CAN_receive() {
    union {
        float value;        // To interpret the 4 bytes as a float
        byte data[4];       // To store 4 bytes of data
    } ldr_lux_rx;

    while ((err = can0.readMessage(&canMsgRx)) == MCP2515::ERROR_OK) {
        unsigned long rx_msg = 0;
        unsigned long mult = 1;
        // Assuming that the first 4 bytes of the message are the float
        for (int i = 0; i < 4; i++) {
            ldr_lux_rx.data[i] = canMsgRx.data[i];  // Copy the bytes into the union
        }

        snprintf(printbuf, BUFSZ, "#%d RXmessage %ld lux: %f\n", nodes[0].node_id, rx_msg, ldr_lux_rx.value);
        Serial.println(printbuf);
    }
    if (err == MCP2515::ERROR_FAIL) {
        Serial.println("CAN Error");
    }
}