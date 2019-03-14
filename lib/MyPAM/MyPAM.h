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

    ServoMotorProperties servo0Properties;
    ServoMotorProperties servo1Properties;
};

class MyPAM
{
  public:
    /**
       * Constructor.
       * Constructor for MyPAM Class
       */
    MyPAM();

    /**
       * Constructor.
       * Constructor for MyPAM Class
       */
    ~MyPAM();

    /**
       * Update method
       * Updates all time dependant variables and objects
       */
    void update();

  /**
       * Returns the position of the end effector
       * @return Matrix : Vector the the current end effector position
       */
    Matrix getPositionVector();

    /**
       * Returns the velocity of the end effector
       * @return Matrix : Vector the the current end effector velocity
       */
    Matrix getVelocityVector();

    SerialEncoder _serialEncoder;
    ServoMotor _servo0;
    ServoMotor _servo1;

  private:
      /**
       * Returns jacobian of the end effector 
       * @return Matrix : Vector the the current end effector position
       */
    Matrix getJacobian();

    MyPAMProperties _properties;
};

#endif
