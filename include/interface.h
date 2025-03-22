#ifndef INTERFACE_H
#define INTERFACE_H

#include <Arduino.h>
#include "lum_node.h" // Include your Node class definition

extern std::vector<Node> nodes; 
extern DataBuffer data_buffer;

void handle_serial_commands();

#endif
