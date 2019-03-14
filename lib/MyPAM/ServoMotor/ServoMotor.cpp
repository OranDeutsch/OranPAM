#include "ServoMotor.h"

ServoMotor::ServoMotor(int jointID,SerialEncoder* serialEncoder): _serialEncoder(serialEncoder)
{
    _jointID = jointID;
}

ServoMotor::~ServoMotor()
{

}

void ServoMotor::setProperties(ServoMotorProperties newProperties)
{
    _properties = newProperties;
}

float ServoMotor::get_angle()
{
    return _angle;
}

float ServoMotor::get_angleV()
{
    return _angleV;
}

void ServoMotor::update()
{
    serialSensorData tempData = _serialEncoder->getSensorData();    //Loads the sensor data

    //Calculates the pulse per radian based on the encoder PPR and gear ratio used
    float pulsePerRadian = (((float)_properties.pulsesPerRevolution * 4.0f) * _properties.gearRatio) / 6.28318;

    //Calculates the angle and angle velocity in rads per second from the data
    _angle      = ((float)tempData.encoderCounts[_jointID] / pulsePerRadian) - _properties.offset;
    _angleV     = ((float)tempData.encoderCountRate[_jointID] / pulsePerRadian);
}