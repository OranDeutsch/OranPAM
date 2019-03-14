#include "ServoMotor.h"

ServoMotor::ServoMotor(int jointID,SerialEncoder* serialEncoder): _serialEncoder(serialEncoder)
{
    _jointID = jointID;

    //Data is filled into properties until EEPROM is setup
    _properties.CountsPerRevolution = 1024;
    _properties.gearRatio = 29.5f;
    
    if (_jointID == 0) _properties.offset = 1.57079f; //Offset is equal to a quater revolution for joint 0
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
    serialSensorData tempData = _serialEncoder->getSensorData();

    float pulsePerRadian = (((float)_properties.CountsPerRevolution * 4.0f) * _properties.gearRatio) / 6.28318;

    _angle = ((float)tempData.encoderCounts[_jointID] / pulsePerRadian) + _properties.offset;
    _angleV = ((float)tempData.encoderCountRate[_jointID] / pulsePerRadian);
}