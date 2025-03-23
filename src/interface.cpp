#include "interface.h"

// Handle serial commands to set the duty cycle
void handle_serial_commands() {

    String command = Serial.readStringUntil('\n');

    // Set Duty Cycle
    if (command.startsWith("u ")){  
        int first_space = command.indexOf(' ');
        int second_space = command.indexOf(' ', first_space + 1);

        if (second_space == -1){
            Serial.println("Invalid command. Use: u <node_id> <duty_cycle>");
            return;
        }

        int node_id = command.substring(first_space + 1, second_space).toInt();
        int duty_cycle = command.substring(second_space + 1).toInt();

        // Validate node_id using node_data.node_id
        for (auto &node : nodes){
            if (node.get_node_id() == node_id)
            {
                // Validate duty cycle
                if (duty_cycle < 0 || duty_cycle > 4095)
                {
                    Serial.println("Error: Duty cycle must be between 0 and 4095.");
                    return;
                }

                // Set the duty cycle
                node.set_duty_cycle(duty_cycle);
                node.update_led();

                Serial.print("Node ");
                Serial.print(node_id);
                Serial.print(" duty cycle set to ");
                Serial.print(duty_cycle);
                Serial.println(".");
                return;
            }
        }

        Serial.println("Error: Node ID not found.");
    }
    
    // Get Duty Cycle
    else if (command.startsWith("g u ")) {
        int node_id = command.substring(4).toInt(); 
        
        // Validate and find the node
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.print(" duty cycle: ");
                Serial.println(node.get_led_duty_cycle());
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    //Set reference
    else if (command.startsWith("r ")){
        int first_space = command.indexOf(' ');
        int second_space = command.indexOf(' ', first_space + 1);

        if (second_space == -1){
            Serial.println("Invalid command. Use: r <node_id> <reference>");
            return;
        }

        int node_id = command.substring(first_space + 1, second_space).toInt();
        float reference = command.substring(second_space + 1).toFloat();

        // Validate node_id using node_data.node_id
        for (auto &node : nodes){
            if (node.get_node_id() == node_id)
            {
                // Set the reference
                node.set_reference(reference);

                Serial.print("Node ");
                Serial.print(node_id);
                Serial.print(" reference set to ");
                Serial.print(reference);
                Serial.println(".");
                return;
            }
        }

        Serial.println("Error: Node ID not found.");
    }

    // Get Reference
    else if (command.startsWith("g r ")) {
        int node_id = command.substring(4).toInt(); 
        
        // Validate and find the node
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.print(" reference: ");
                Serial.println(node.get_reference());
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    //Get illuminance
    else if (command.startsWith("g y ")) {
        int node_id = command.substring(4).toInt(); 
        
        // Validate and find the node
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.print(" illuminance: ");
                Serial.println(node.get_ldr_lux());
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    // Get extarnal illuminance
    else if (command.startsWith("g d ")) {
        int node_id = command.substring(4).toInt(); 
        
        // Validate and find the node
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.print(" external illuminance: ");
                Serial.println(node.get_ldr_lux_extern());
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    // Get capacitor voltage
    else if (command.startsWith("g v ")) {
        int node_id = command.substring(4).toInt(); 
        
        // Validate and find the node
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.print(" capacitor voltage: ");
                Serial.println(node.get_c_voltage());
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    // Set Occupancy
    else if (command.startsWith("o ")){
        int first_space = command.indexOf(' ');
        int second_space = command.indexOf(' ', first_space + 1);

        if (second_space == -1){
            Serial.println("Invalid command. Use: o <node_id> <occupancy_state>");
            return;
        }

        int node_id = command.substring(first_space + 1, second_space).toInt();
        bool occupancy_state = command.substring(second_space + 1).toInt();

        // Validate node_id using node_data.node_id
        for (auto &node : nodes){
            if (node.get_node_id() == node_id)
            {
                // Set the occupancy state
                node.set_occupancy(occupancy_state);

                Serial.print("Node ");
                Serial.print(node_id);
                Serial.print(" occupancy state set to ");
                Serial.print(occupancy_state ? "Occupied" : "Unoccupied");
                Serial.println(".");
                return;
            }
        }

        Serial.println("Error: Node ID not found.");
    }

    // Get Occupancy
    else if (command.startsWith("g o ")) {
        int node_id = command.substring(4).toInt(); 
        
        // Validate and find the node
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.print(" occupancy state: ");
                Serial.println(node.get_occupancy() ? "Occupied" : "Unoccupied");
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    // Enable Duty Cycle Stream
    else if (command.startsWith("s u ")) {
        int node_id = command.substring(4).toInt();

        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                node.set_stream_u(true);  // Set the flag to true
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.println(" stream_u flag enabled.");
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    // Disable Duty Cycle Stream
    else if (command.startsWith("S u ")) {
        int node_id = command.substring(4).toInt();
                 
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                node.set_stream_u(false);  // Set the flag to true
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.println(" stream_u flag disabled.");
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    // Enable Illuminance Stream
    else if (command.startsWith("s y ")) {
        int node_id = command.substring(4).toInt();
                 
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                node.set_stream_y(true);  // Set the flag to true
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.println(" stream_y flag enabled.");
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    // Enable Illuminance Stream
    else if (command.startsWith("S y ")) {
        int node_id = command.substring(4).toInt();
                 
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                node.set_stream_y(false);  // Set the flag to true
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.println(" stream_y flag disabled.");
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    // Get last minute buffer for duty cycle
    else if (command.startsWith("g b u ")) {
        int node_id = command.substring(6).toInt();
        
        // Search for the node in the buffer
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.println(" duty cycles last minute buffer:");

                // Access the buffer and print all duty cycles
                data_buffer.print_duty_cycle();
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    // Get last minute buffer for illuminance
    else if (command.startsWith("g b y ")) {
        int node_id = command.substring(6).toInt();
        
        // Search for the node in the buffer
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.println(" duty cycles last minute buffer:");

                // Access the buffer and print all duty cycles
                data_buffer.print_ldr_lux();
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }
    
    else if (command.startsWith("g b ")) {
        int node_id = command.substring(4).toInt();
        
        // Search for the node in the buffer
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.println("last minute buffer:");

                // Access the buffer and print all duty cycles
                data_buffer.print_all();
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }

    // Get elapsed time
    else if (command.startsWith("g t ")) {
        int node_id = command.substring(4).toInt();
        
        // Search for the node in the buffer
        for (auto &node : nodes) {
            if (node.get_node_id() == node_id) {
                Serial.print("Node ");
                Serial.print(node_id);
                Serial.print(" elapsed time: ");
                Serial.print(node.get_timestamp());
                Serial.println(" ms.");
                return;
            }
        }
        Serial.println("Error: Node ID not found.");
    }
    
    else{
        Serial.println("Unknown or invalid command.");
    }
}
