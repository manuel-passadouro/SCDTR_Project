#ifndef COMMUNICATION_H
#define COMMUNICATIONL_H

#include <Arduino.h>
#include "mcp2515.h"
#include "lum_node.h"

// External Varibles (defined in main)
extern uint8_t node_address; // Short ID
extern struct can_frame canMsgTx, canMsgRx;
extern unsigned long counter;
extern MCP2515::ERROR err;
extern const int BUFSZ;
extern char printbuf[];
extern MCP2515 can0;

extern std::vector<Node> nodes;

void can_setup(pico_unique_board_id_t board_id);
void CAN_send(int node_id, float data_to_send);
void CAN_receive(int node_id);

#endif // COMMUNICATION_H