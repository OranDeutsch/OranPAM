#ifndef MYPAM_H
#define MYPAM_H

/*
 * Includes
 */
#include "mbed.h"
#include "ServoMotor.h"
#include "Matrix.h"
#include "USBHID.h"

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

    /**
       * Sets the setpoint position coodintes of the end effector
       * @param int X : x position of new position
       * @param int Y : y position of new porsition
       * @return True if successful, false if error is detected
       */
    bool set_position(int x, int y);

    void set_motorPower(bool enable);

    SerialEncoder _serialEncoder;
    ServoMotor _servo0;
    ServoMotor _servo1;

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

    /**
       * check a move to a new x,y position will not cause a crash
       * @param int X : x position of new position
       * @param int Y : y position of new porsition
       * @return True possable, false if impossable
       */
    bool checkCoordinateInput(int x, int y);

    /**
       * sends the current system state over USB
       */
    void send_HID();

    /**
       * checks if a new HID command has been recieved
       */
    void recv_HID();

    /**
       * setpoint coodintes vector
       */
    Matrix _setPoint;

    /**
       * _USBHID device controller
       */
    USBHID _hid;

    /**
       * outgoing HID report
       */
    HID_REPORT _send_report;

    /**
       * incoming HID report
       */
    HID_REPORT _recv_report;

    /**
       * The current device properties
       */
    MyPAMProperties _properties;

    bool _enabled;
};

#endif
