#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

/*
 * Includes
 */
#include "mbed.h"
#include "SerialEncoder.h"

/**
 * Defines
 */

struct ServoMotorProperties
{
    float p, i, d;              //PID values

    float offset;               //Angular offset

    int pulsesPerRevolution;    //Motor encoder pulses per revolution 
    float gearRatio;            //Gear ratio of motor
};

class ServoMotor
{
  public:
    /**
       * Constructor.
       * Constructor for servo motor class
       *  @param JointID: The position of the joint on the MyPAM
       *                  use 0 for base and 1 for first
       *  @param SerialEncoder: pointer to a serial encoder object
       */
    ServoMotor(int jointID, SerialEncoder *serialEncoder);

    /**
       * Destructor.
       */
    ~ServoMotor();

    /**
       * Sets the properties of the servo motor
       * @param newProperties : new properties to replace old ones
       */
    void setProperties(ServoMotorProperties newProperties);

    /**
       * Gets the current angle of the servo motor
       * @param newProperties : new properties to replace old ones
       */
    float get_angle();

    /**
       * Gets the current angular velocity of the servo motor
       * @return angular velocity in rad/s
       */
    float get_angleV();

    /**
       * Updates the servo motor
       */
    void update();

  private:
    /**
       * ID of the joint (0 or 1)
       */
    int _jointID;

    /**
       * Angle of motor shaft in rads
       */
    float _angle;

    /**
       * Anglur velocity of motor shaft in rad/s
       */
    float _angleV;

    /**
       * Properties of individual servo motor
       */
    ServoMotorProperties _properties;

    /**
       * Pointer to existing serial encoder object
       */
    SerialEncoder *_serialEncoder;
};

#endif
