#include "MyPAM.h"

MyPAM::MyPAM() : _serialEncoder(PTC17, PTC16), _servo0(0, &_serialEncoder), _servo1(1, &_serialEncoder)
{
    _properties.l0 = 340;
    _properties.l1 = 240;
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

    //Calculate end effector position
    float x = ((_properties.l0 * cos(a0)) + _properties.l1 * cos(((a1 + a0))));
    float y = ((_properties.l0 * sin(a0)) + _properties.l1 * sin(((a1 + a0))));

    Matrix positionMatrix(2, 1);

    positionMatrix << x
                   << y;

    return positionMatrix;
}

Matrix getVelocityVector()
{
    Matrix velocityMatrix(2, 1);
}
