#include "MyPAM.h"

MyPAM::MyPAM() : _serialEncoder(PTC17, PTC16), _servo0(0, &_serialEncoder), _servo1(1, &_serialEncoder)
{
    //Tempoary until EEPROm saving is implemented 
    _properties.l0 = 340;
    _properties.l1 = 240;

    _properties.servo0Properties.pulsesPerRevolution = 1024;
    _properties.servo0Properties.gearRatio = 29.5f;
    _properties.servo0Properties.offset = 1.57079f;

    _properties.servo1Properties.pulsesPerRevolution = 1024;
    _properties.servo1Properties.gearRatio = 29.5f;
    _properties.servo1Properties.offset = 0;

    _servo0.setProperties(_properties.servo0Properties);
    _servo1.setProperties(_properties.servo1Properties);
}

MyPAM::~MyPAM()
{
}

void MyPAM::update()
{

    _servo0.update();
    _servo1.update();
}

Matrix MyPAM::getPositionVector()
{
    //Get current arm angles
    float a0 = _servo0.get_angle();
    float a1 = _servo1.get_angle();

    Matrix positionVector(2, 1);        //Create empty matrix for position valuye

    //Calculate end effector position into empty matrix
    positionVector << ((_properties.l0 * cos(a0)) + _properties.l1 * cos(((a1 + a0))))
                   << ((_properties.l0 * sin(a0)) + _properties.l1 * sin(((a1 + a0))));

    return positionVector;              //Returns vector of X-Y position of end effector
}

Matrix MyPAM::getVelocityVector()
{
    //Create velocity vector and angular velocity vector
    Matrix velocityVector(2, 1);
    Matrix angularVelocityVector(2,1);

    Matrix jacobianMatrix = getJacobian();  //Call the Jacobian matrix

    //Append angle velocities into the angular velocity vectors
    angularVelocityVector   << _servo0.get_angleV()
                            << _servo1.get_angleV();
    
    //Calculate velocity vector
    velocityVector = jacobianMatrix * angularVelocityVector;

    return velocityVector;
}

Matrix MyPAM::getJacobian()
{
    Matrix Jee(2, 2);   //Empty value to insert jacobian values in   

    float a0 = _servo0.get_angle();
    float a1 = _servo1.get_angle();

    //See report for explentation of the Jacobian matrix
    Jee << (0 - (_properties.l0 * sin(a0)) - (_properties.l1 * sin((a0 + a1)))) << (0 - (_properties.l1 * sin((a0 + a1))))
        << ((_properties.l0 * cos(a0)) + (_properties.l1 * cos((a0 + a1))))     << (_properties.l1 * cos((a0 + a1)));

    return Jee;         //Return calculated jacobian matrix 
}
