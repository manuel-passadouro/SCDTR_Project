#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

class pid{
    float I, D, K, Ti, Td, b, h, y_old, N;

    public:
        explicit pid(float h_, float K_ = 1, float b_ = 1,
                    float Ti_ = 1, float Td_ = 0, float N_ = 10);
        ~pid() {};
        float compute_control(float r, float y);
        void housekeep(float r, float y);
        
        // Setters for PID parameters
        void set_h(float _h) { h = _h; }
        void set_K(float _K) { K = _K; }
        void set_b(float _b) { b = _b; }
        void set_Ti(float _Ti) { Ti = _Ti; }
                
};

inline void pid::housekeep(float r, float y){
    float e = r - y;
    I += K * h / Ti * e;
    y_old = y;
}

#endif // CONTROL_H