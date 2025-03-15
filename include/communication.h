#ifndef COMMUNICATION_H
#define COMMUNICATIONL_H

#include <Arduino.h>
#include "mcp2515.h"
#include "data.h"

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

extern std::vector<NodeData> nodes;


void can_setup();
void CAN_send(unsigned long current_time);
void CAN_receive();

#endif // COMMUNICATION_H