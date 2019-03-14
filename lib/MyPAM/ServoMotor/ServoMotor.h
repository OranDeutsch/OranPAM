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
    float p, i, d;

    float offset;

    int CountsPerRevolution;
    float gearRatio;
};

class ServoMotor
{
  public:
    ServoMotor(int jointID, SerialEncoder *serialEncoder);

    ~ServoMotor();

    void setProperties(ServoMotorProperties newProperties);

    float get_angle();

    float get_angleV();

    void update();

  private:
    int _jointID;

    float _angle;

    float _angleV;

    ServoMotorProperties _properties;

    SerialEncoder *_serialEncoder;
};

#endif
