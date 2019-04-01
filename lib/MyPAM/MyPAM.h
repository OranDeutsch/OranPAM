#ifndef MYPAM_H
#define MYPAM_H

/*
 * Includes
 */
#include "mbed.h"
#include "ServoMotor.h"
#include "Matrix.h"
#include "USBHID.h"
#include "I2CEEBlockDevice.h"
#include "buzzer.h"
#include "Utilities.h"
/**
 * Defines
 */
#define PI 3.141592f
#define BLOCK_SIZE 32

struct MyPAMProperties
{
   int l1;
   int l0;

   float interval;

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

   /**
       * The current device properties
       */
   MyPAMProperties _properties;

/**
       * Sets the motors on or off
       * @param bool enable : true/false for motor power 
       */
   void set_motorPower(bool enable);

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
       * Gets the new setpoint from the last recieved HID report
       */
   void getHIDSetPoint();

   /**
       * enables/disables the motors based on the last recieved HID report
       */
   void getHIDMotorsEnabled();

   /**
       * Get the assistance factor from the last recieved HID report
       */
   void getHIDAssistanceFactor();

   /**
       * sends the global properties over a HID report
       */
   void sendGlobalProperties();

/**
       * sends the servo properties over a HID report
       * @param int ServoID : ID of the servo (0 or 1)
       */
   void sendServoProperties(int servoID);

 /**
       * Gets the global properties form the last recieved HID report
       */
   void getGlobalProperties();

/**
       * Gets the properties of a servo motor from the last recieved HID report
       * @param int ServoID : ID of the servo (0 or 1)
       */
   void getServoProperties(int servoID);

   /**
       * Save a properties stuct to EEPROM
       * @param MyPAMProperties properties: Properties struct to save to EEPROM
       */
   void saveProperties(MyPAMProperties properties);

/**
       * Calibrate the potentiometer-encoder model
       */
   void calibratePotentiometers();

/**
       * Load the properties from EEPROM
       * @return MyPAMProperties : Properties loaded from EEPROM
       */
   MyPAMProperties loadProperties();

   /**
       * Interface to encoder-serial sub-circuit
       */
   SerialEncoder _serialEncoder;

   /**
       * "servo motor" at joint 0
       */
   ServoMotor _servo0;

   /**
       * "servo motor" at joint 1
       */
   ServoMotor _servo1;

   /**
       * on circuit buzzer used for diagnwotics
       */
   Beep _buzzer;

   /**
       * on circuit EEPROM used for saving and loading the configuration
       */
   I2CEEBlockDevice _i2cEEPROM;

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

   bool _enabled;
   bool _hidSendENabled;

   int _assistanceFactor;
};

#endif
