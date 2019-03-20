#include "HBridge.h"
#include "mbed.h"

HBridge::HBridge(PinName pwm, PinName dir, PinName slp, PinName cs) : _pwm(pwm), _dir(dir), _slp(slp), _cs(cs)
{
    _pwm.period(PWM_PERIOD);

    _dir = 1; //Default direction is forwards
    _slp = 1; //Disabled by default

}

void HBridge::set_hbridge_duty(float mag)
{

    //Control mag and dir pins
    if (mag >= 0) _dir = 0;                  // Sets forward direction
    if (mag < 0) _dir = 1;                  // Sets reverse direction
    
    float output = abs(mag);
    //generate duty cycle
    if (output >= 1.0f) output = 0.99f;
    //Write new duty cycle to h bridge
    _pwm.write(output);

}

float HBridge::get_current()
{

    float vin = _cs * 3.3f; //find the input voltage on the current sense pin
    _current = (vin - (3.3/2)) / 0.4;
    return _current;
    
}

void HBridge::enabled(bool enabled){
    
    _slp = enabled;
        
}