#include "ServoMotor.h"

ServoMotor::ServoMotor(
    int jointID,
    SerialEncoder *serialEncoder,
    PinName pwm, PinName dir, PinName slp, PinName cs)

    : _serialEncoder(serialEncoder),
      _anglePID(5, 0, 0, 0.016),
      _hbridge(pwm, dir, slp, cs)

{
    _jointID = jointID;
    _hbridge.enabled(false);
}

ServoMotor::~ServoMotor()
{
}

void ServoMotor::setProperties(ServoMotorProperties newProperties)
{
    _properties = newProperties;

    _anglePID.reset();
    _anglePID.setMode(1);
    _anglePID.setTunings(_properties.p, _properties.i, _properties.d);
    _anglePID.setInterval(_properties.PIDinterval);
    _anglePID.setBias(0);
    _anglePID.setInputLimits(_properties.minAngle, _properties.maxAngle);
    _anglePID.setOutputLimits((0 - _properties.maxDutyCycle), _properties.maxDutyCycle);
}

float ServoMotor::get_angle()
{
    return _angle;
}

void ServoMotor::set_angleSetpoint(float angleSetpoint)
{
    _angleSetpoint = angleSetpoint;

    _anglePID.setSetPoint(_angleSetpoint);
}

float ServoMotor::get_angleV()
{
    return _angleV;
}

void ServoMotor::update()
{
    serialSensorData tempData = _serialEncoder->getSensorData(); //Loads the sensor data

    //Calculates the pulse per radian based on the encoder PPR and gear ratio used
    float pulsePerRadian = (((float)_properties.pulsesPerRevolution * 4.0f) * _properties.gearRatio) / 6.28318;

    //Calculates the angle and angle velocity in rads per second from the data
    _angle = ((float)tempData.encoderCounts[_jointID] / pulsePerRadian) - _properties.offset;
    _angleV = ((float)tempData.encoderCountRate[_jointID] / pulsePerRadian);

    //Apply values to PID loop
    _anglePID.setProcessValue(_angle);

    //get dutycycle from pid loop
    _dutyCycle = _anglePID.compute();

    _hbridge.set_hbridge_duty(_dutyCycle);
}