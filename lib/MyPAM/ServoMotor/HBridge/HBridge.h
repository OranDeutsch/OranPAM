#ifndef HBRIDGE_H
#define HBRIDGE_H

/*
 * Includes
 */
#include "mbed.h"

/**
 * Defines
 */
#define PWM_PERIOD 0.00005


/**
 * DC Motor with polulu H-Bridge interface 
 */
class HBridge
{

public:

    /**
     * Constructor.
     *
     * @param pwm       mbed pin for pwm
     * @param dir       mbed pin for direction (0 = backward 1 = forward)
     * @param slp       mbed pin for motor enable
     */
    HBridge(PinName pwm, PinName dir, PinName slp, PinName cs);
    
    /** 
    * set the motor input duty and direction+
    *
    * @param mag        Motor magnitude from -100 to 100% velocity
    */
    void set_hbridge_duty(float mag);
    
    /**
    * find the current motor current draw
    * 
    * @return motor current in amps
    */
    float get_current();
    
    void enabled(bool enabled);

private:

    PwmOut _pwm;

    DigitalOut _dir;
    DigitalOut _slp;
    
    
    AnalogIn   _cs;
    
    float _current;
    
};

#endif