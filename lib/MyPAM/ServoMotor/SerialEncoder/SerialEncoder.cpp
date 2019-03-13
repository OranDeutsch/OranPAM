#include "SerialEncoder.h"

SerialEncoder::SerialEncoder(PinName TXPin, PinName RXPin, int baud) : _encoderSerial(TXPin, RXPin)
{
        _encoderSerial.baud(baud);
}

SerialEncoder::~SerialEncoder()
{
}

serialSensorData SerialEncoder::getSensorData()
{
        serialSensorData data;

        _encoderSerial.putc('s');

        while (!_encoderSerial.readable());

        u_int8_t encoderSerialBuffer[16];

        for (int i = 0; i < 16; i++)
        {
                encoderSerialBuffer[i] = _encoderSerial.getc();
        }

        flushSerialBuffer();

        data.encoderCounts[0] = (((unsigned long)encoderSerialBuffer[3] << 24) | ((unsigned long)encoderSerialBuffer[2] << 16) | ((unsigned long)encoderSerialBuffer[1] << 8) | ((unsigned long)encoderSerialBuffer[0]));
        data.encoderCounts[1] = (((unsigned long)encoderSerialBuffer[7] << 24) | ((unsigned long)encoderSerialBuffer[6] << 16) | ((unsigned long)encoderSerialBuffer[5] << 8) | ((unsigned long)encoderSerialBuffer[4]));
        data.encoderCountRate[0] = (((unsigned long)encoderSerialBuffer[11] << 24) | ((unsigned long)encoderSerialBuffer[10] << 16) | ((unsigned long)encoderSerialBuffer[9] << 8) | ((unsigned long)encoderSerialBuffer[8]));
        data.encoderCountRate[1] = (((unsigned long)encoderSerialBuffer[15] << 24) | ((unsigned long)encoderSerialBuffer[14] << 16) | ((unsigned long)encoderSerialBuffer[13] << 8) | ((unsigned long)encoderSerialBuffer[12]));

        return data;
}

void SerialEncoder::flushSerialBuffer()
{
        char char1 = 0;

        while (_encoderSerial.readable())
        {
                char1 = _encoderSerial.getc();
        }
        return;
}

bool SerialEncoder::reset()
{
        _encoderSerial.putc('r');

        //while (!_encoderSerial.readable());

        //char feedback = _encoderSerial.getc();

        return true;

}