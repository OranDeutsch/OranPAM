#include "ServoMotor.h"

ServoMotor::ServoMotor(int jointID,SerialEncoder* serialEncoder): _serialEncoder(serialEncoder), _anglePID(1,0,0,(1/60))
{
    _jointID = jointID;
}

ServoMotor::~ServoMotor()
{

}

void ServoMotor::setProperties(ServoMotorProperties newProperties)
{
    _properties = newProperties;

    _anglePID.reset();
    _anglePID.setTunings(_properties.p,_properties.i,_properties.d);
    _anglePID.setInterval(_properties.PIDinterval);
    _anglePID.setInputLimits(_properties.minAngle,_properties.maxAngle);
    _anglePID.setOutputLimits(-_properties.maxDutyCycle,_properties.maxDutyCycle);
    _anglePID.reset();

}

float ServoMotor::get_angle()
{
    return _angle;
}

void ServoMotor::set_angleSetpoint(float angleSetpoint)
{
    _angleSetpoint = angleSetpoint - _properties.offset;
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

    //Apply values to PID loop
    _anglePID.setSetPoint(_angleSetpoint);
    _anglePID.setProcessValue(_angle);

    //
    float dutyCycle = _anglePID.compute();
}