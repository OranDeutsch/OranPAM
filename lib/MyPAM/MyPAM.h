#ifndef MYPAM_H
#define MYPAM_H

/*
 * Includes
 */
#include "mbed.h"
#include "ServoMotor.h"
#include "Matrix.h"
/**
 * Defines
 */

struct MyPAMProperties
{
    int l1;
    int l0;
};

class MyPAM
{
  public:
    MyPAM();

    ~MyPAM();

    void update();

    SerialEncoder _serialEncoder;
    ServoMotor _servo0;
    ServoMotor _servo1;

    Matrix getPositionVector();
    Matrix getVelocityVector();

  private:
    MyPAMProperties _properties;
};

#endif
