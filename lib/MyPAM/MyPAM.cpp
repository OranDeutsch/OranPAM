#include "MyPAM.h"

MyPAM::MyPAM() : _serialEncoder(PTC17, PTC16),
                 _servo0(0, &_serialEncoder,PTC11,PTC12,PTC9,PTB2),
                 _servo1(1, &_serialEncoder,PTD5,PTD6,PTD3,PTB3),
                 _setPoint(2, 1)
{
    //Tempoary until EEPROm saving is implemented
    _properties = loadDefaultProperties();

    _servo0.setProperties(_properties.servo0Properties);
    _servo1.setProperties(_properties.servo1Properties);

    _setPoint << 0
              << -450;
}

MyPAM::~MyPAM()
{
}

void MyPAM::update()
{
    _servo0.update();
    _servo1.update();
}

Matrix MyPAM::getCurrentPositionVector()
{
    //Get current arm angles
    float a0 = _servo0.get_angle();
    float a1 = _servo1.get_angle();

    Matrix positionVector(2, 1); //Create empty matrix for position valuye

    //Calculate end effector position into empty matrix
    positionVector << ((_properties.l0 * cos(a0)) + _properties.l1 * cos(((a1 + a0))))
                   << ((_properties.l0 * sin(a0)) + _properties.l1 * sin(((a1 + a0))));

    return positionVector; //Returns vector of X-Y position of end effector
}

Matrix MyPAM::getSetPointPositionVector()
{
    return _setPoint;
}

Matrix MyPAM::getVelocityVector()
{
    //Create velocity vector and angular velocity vector
    Matrix velocityVector(2, 1);
    Matrix angularVelocityVector(2, 1);

    Matrix jacobianMatrix = get_Jacobian(); //Call the Jacobian matrix

    //Append angle velocities into the angular velocity vectors
    angularVelocityVector << _servo0.get_angleV()
                          << _servo1.get_angleV();

    //Calculate velocity vector
    velocityVector = jacobianMatrix * angularVelocityVector;

    return velocityVector;
}

Matrix MyPAM::get_Jacobian()
{
    Matrix Jee(2, 2); //Empty value to insert jacobian values in

    float a0 = _servo0.get_angle();
    float a1 = _servo1.get_angle();

    //See report for explentation of the Jacobian matrix
    Jee << (0 - (_properties.l0 * sin(a0)) - (_properties.l1 * sin((a0 + a1)))) << (0 - (_properties.l1 * sin((a0 + a1))))
        << ((_properties.l0 * cos(a0)) + (_properties.l1 * cos((a0 + a1)))) << (_properties.l1 * cos((a0 + a1)));

    return Jee; //Return calculated jacobian matrix
}

void MyPAM::set_position(int x, int y)
{
    _setPoint.Clear();
    _setPoint << x
              << y;

    Matrix angles = get_inverseKinematicPosition(_setPoint);

    _servo0.set_angleSetpoint(angles.getNumber(1, 1));
    _servo1.set_angleSetpoint(angles.getNumber(2, 1));
}

Matrix MyPAM::get_inverseKinematicPosition(Matrix position)
{
    Matrix angles(2, 1);

    float x = position.getNumber(1, 1);
    float y = position.getNumber(2, 1);

    float f1 = (pow(x, 2) + pow(y, 2) - pow(_properties.l0, 2) - pow(_properties.l1, 2)) / (2 * _properties.l0 * _properties.l1);
    float f2 = sqrt(1 - pow(f1, 2));

    float theta1 = atan2(f2, f1);

    float f3 = atan2(y, x);
    float f4 = _properties.l1 * f2;
    float f5 = _properties.l0 + _properties.l1 * f1;
    float f6 = atan2(f4, f5);

    float theta0 = f6 + f3;

    theta1 = 0 - theta1;

    angles << theta0
           << theta1;

    return angles;
}

MyPAMProperties MyPAM::loadDefaultProperties()
{
    MyPAMProperties tempProperties;

    //Provisionial link length values
    tempProperties.l0 = 340;
    tempProperties.l1 = 240;

    //Provisionial motor values
    tempProperties.servo0Properties.pulsesPerRevolution = 1024;
    tempProperties.servo0Properties.gearRatio = 29.5f;
    tempProperties.servo0Properties.offset = 1.57079f;

    tempProperties.servo1Properties.pulsesPerRevolution = 1024;
    tempProperties.servo1Properties.gearRatio = 29.5f;
    tempProperties.servo1Properties.offset = 0;

    //Provisionial PID values
    tempProperties.servo0Properties.p = 8;
    tempProperties.servo0Properties.i = 0;
    tempProperties.servo0Properties.d = 0;
    tempProperties.servo0Properties.PIDinterval = 1 / 60;
    tempProperties.servo0Properties.minAngle = -3.14;
    tempProperties.servo0Properties.maxAngle = -0;
    tempProperties.servo0Properties.maxDutyCycle = 0.3;

    tempProperties.servo1Properties.p = 12;
    tempProperties.servo1Properties.i = 0;
    tempProperties.servo1Properties.d = 0;
    tempProperties.servo1Properties.PIDinterval = 1 / 60;
    tempProperties.servo1Properties.minAngle = -3.14;
    tempProperties.servo1Properties.maxAngle = 3.14;
    tempProperties.servo1Properties.maxDutyCycle = 0.2;

    return tempProperties;
}
