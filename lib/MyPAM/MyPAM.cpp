#include "MyPAM.h"

MyPAM::MyPAM():serialEncoder(PTC17,PTC16),servo0(0, &serialEncoder),servo1(1, &serialEncoder)
{

}

MyPAM::~MyPAM()
{

}

void MyPAM::update()
{
    
    servo0.update();
    servo1.update();

}

