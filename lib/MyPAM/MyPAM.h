#ifndef MYPAM_H
#define MYPAM_H

/*
 * Includes
 */
#include "mbed.h"
#include "ServoMotor.h"

/**
 * Defines
 */

class MyPAM
{
  public:
    MyPAM();

    ~MyPAM();

    void update();

    ServoMotor servo0;
    ServoMotor servo1;
    SerialEncoder serialEncoder;

  private:
};

#endif
