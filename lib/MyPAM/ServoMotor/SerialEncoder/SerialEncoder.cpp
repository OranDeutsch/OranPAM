#include "SerialEncoder.h"

SerialEncoder::SerialEncoder(PinName TXPin, PinName RXPin, int baud) : _encoderSerial(TXPin, RXPin)
{
        _encoderSerial.baud(baud);              //Set baud rate of serial link

        reset();
}

SerialEncoder::~SerialEncoder()
{
}

serialSensorData SerialEncoder::getSensorData()
{
        serialSensorData data;                  //Tempory data stuct to store data too

        _encoderSerial.putc('s');               //Send "send" command to encoder circuit

        while (!_encoderSerial.readable());     //Wait for response

        u_int8_t encoderSerialBuffer[16];       //Tempory buffer for incoming data

        for (int i = 0; i < 16; i++)            //Load serial data to buffer
        {
                encoderSerialBuffer[i] = _encoderSerial.getc();
        }

        flushSerialBuffer();

        //Load buffer data into variables
        //This could be improved upon but it works well
        data.encoderCounts[0] = 0 - (((unsigned long)encoderSerialBuffer[3] << 24) | ((unsigned long)encoderSerialBuffer[2] << 16) | ((unsigned long)encoderSerialBuffer[1] << 8) | ((unsigned long)encoderSerialBuffer[0]));
        data.encoderCounts[1] = (((unsigned long)encoderSerialBuffer[7] << 24) | ((unsigned long)encoderSerialBuffer[6] << 16) | ((unsigned long)encoderSerialBuffer[5] << 8) | ((unsigned long)encoderSerialBuffer[4]));
        data.encoderCountRate[0] = 0 - (((unsigned long)encoderSerialBuffer[11] << 24) | ((unsigned long)encoderSerialBuffer[10] << 16) | ((unsigned long)encoderSerialBuffer[9] << 8) | ((unsigned long)encoderSerialBuffer[8]));
        data.encoderCountRate[1] = (((unsigned long)encoderSerialBuffer[15] << 24) | ((unsigned long)encoderSerialBuffer[14] << 16) | ((unsigned long)encoderSerialBuffer[13] << 8) | ((unsigned long)encoderSerialBuffer[12]));

        return data;
}

void SerialEncoder::flushSerialBuffer()
{
        uint8_t byte = 0;                      //Dummy byte

        while (_encoderSerial.readable())      //Run through remaing buffer and unload
        {
                byte = _encoderSerial.getc();
        }
        return;
}

bool SerialEncoder::reset()
{
        _encoderSerial.putc('r');               //Send "reset" command to launchpad

        return true;                            //Error handling is yet to be implementated

}

void SerialEncoder::calibrate()
{
         _encoderSerial.putc('c');
}