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
#define PI 3.141592f

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
    Matrix getCurrentPositionVector();

    /**
       * Returns the current position setpoint
       * @return Matrix : Vector the current end effector setpoint
       */
    Matrix getSetPointPositionVector();

    /**
       * Computes the required motor angles to achieve a set of end effctor coordinates
       * @param Position : X,Y vector of desired end effector position
       * @return Vector of motor angles
       */
    Matrix get_inverseKinematicPosition(Matrix position);

    /**
       * Returns the velocity of the end effector
       * @return Matrix : Vector the the current end effector velocity
       */
    Matrix getVelocityVector();

    void set_position(int x,int y);

    SerialEncoder _serialEncoder;
    ServoMotor _servo0;
    ServoMotor _servo1;

    Matrix _setPoint;

  private:
  /**
       * Forces the MyPAM to use hard saved properties
       */
    MyPAMProperties loadDefaultProperties();

    /**
       * Returns jacobian of the end effector 
       * @return Matrix : Vector the the current end effector position
       */
    Matrix get_Jacobian();

   

    MyPAMProperties _properties;

    
};

#endif
