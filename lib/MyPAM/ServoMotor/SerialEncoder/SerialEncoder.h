#ifndef SERIALENCODER_H
#define SERIALENCODER_H

/*
 * Includes
 */
#include "mbed.h"

/**
 * Defines
 */

struct serialSensorData
{
  long encoderCounts[2];
  long encoderCountRate[2];
  float potentiometer[2];
};

class SerialEncoder
{
public:
  /**
       * Constructor.
       * Constructor for serial encoder classs
       *  @param TX: PinName for serial TX pin
       *  @param RX: PinName for serial TX pin
       *  @param Baud: int for serial baud rate, defaulted at 115200
       */
  SerialEncoder(PinName TXPin, PinName RXPin, int baud = 115200);

  /**
       * Destructor.
       */
  ~SerialEncoder();

  /**
       * collects the current encoder values when called, returned as serialSensorData struct
       * @return serialSensorData : current encoder data
       */
  serialSensorData getSensorData();

  /**
       * Resets the encoder valus
       * @return boolean (true = success, false = failure)
       */
  bool reset();

/**
       * Starts calibration seqence
       */
  void calibrate();

private:
  /**
       * Empties the serial buffer
       */
  void flushSerialBuffer();

  Serial _encoderSerial;
};

#endif
