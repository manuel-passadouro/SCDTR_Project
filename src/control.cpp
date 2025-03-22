#include "control.h"

pid::pid(float h_, float K_, float b_, float Ti_, float Td_, float N_)
    // member variable initialization list
    : h{h_}, K{K_}, b{b_}, Ti{Ti_}, Td{Td_},
      N{N_}, I{0.0}, D{0.0}, y_old{0.0} {} // should check arguments validity

float pid::compute_control(float r, float y){
    float P = K * (b * r - y);                  // Proportional term w/ setpoint weighting (b)
    //float ad = Td / (Td + N * h);
    //float bd = Td * K * N / (Td + N * h);       
    //D = ad * D - bd * (y - y_old);            // Differential term (not used)
    float u = P + I;

    if (u < 0)                                  // Output Saturation
        u = 0;
    if (u > 4095)
        u = 4095;
    return u;
}