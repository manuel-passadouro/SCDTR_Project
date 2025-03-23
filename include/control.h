#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

extern const float VCC;

class pid{
    float I, D, K, Ti, uff, Td, b, h, y_old, K_old, b_old, N;

    public:
        explicit pid(float h_, float K_ = 1, float b_ = 1,
                    float Ti_ = 1,float uff_ = 0, float Td_ = 0, float N_ = 10);
        ~pid() {};
        float compute_control(float r, float y, float G);
        void housekeep(float r, float y, float u);
        
        // Setters for PID parameters
        void set_h(float _h) { h = _h; }
        void set_K(float _K) { K = _K; }
        void set_b(float _b) { b = _b; }
        void set_Ti(float _Ti) { Ti = _Ti; }
        void set_uff(float _uff) { uff = _uff; }
                
};

inline void pid::housekeep(float r, float y, float u){
    float e = r - y;                                // Compute Error
    float bi = K * h / Ti;                          // Integral gain
    float ao = h / Ti;                              // Anti-windup back calculation gain

    I += K_old * (b_old * r - y) - K * (b * r - y); // Bumpless transfer correction
    
    I += bi * e + ao * (u - (I + K * (b * r - y))); // Update integral with anti-windup

    //I += K * h / Ti * e;                          // Integral term (h is the sampling time)
    //y_old = y;                                    // Update y_old Por Differential
                                                    // term computation (not used)

    K_old = K;                                      // Store previous parameters for next cycle
    b_old = b;
}

#endif // CONTROL_H