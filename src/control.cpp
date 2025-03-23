#include "control.h"

float fb_flag = true; // Feedback flag

pid::pid(float h_, float K_, float b_, float Ti_, float uff_, float Td_, float N_)
    // member variable initialization list
    : h{h_}, K{K_}, b{b_}, Ti{Ti_}, uff{uff_}, Td{Td_},
      N{N_}, I{0.0}, D{0.0}, y_old{0.0}, K_old{0.0}, b_old{0.0} {} // should check arguments validity

float pid::compute_control(float r, float y, float G){
    float P = K * (b * r - y);                      // Proportional term w/ setpoint weighting (b)
    //float ad = Td / (Td + N * h);
    //float bd = Td * K * N / (Td + N * h);       
    //D = ad * D - bd * (y - y_old);                // Differential term (not used)
    float feedforward = uff * (r/((G/4095) * VCC)); // Get Duty Cycle directly from r 
                                                    //(set uff to 0 to disable)
    
    //float feedforward = uff  * r;
    float u = (fb_flag) ? (P + I + feedforward) : (feedforward); // FB + FF or just FF

    if (u < 0)                                      // Output Saturation
        u = 0;
    if (u > 4095)
        u = 4095;
    return u;
}