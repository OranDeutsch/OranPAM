#include "MyPAM.h"

MyPAM::MyPAM() : _serialEncoder(PTC17, PTC16),
                 _servo0(0, &_serialEncoder, PTC11, PTC12, PTC9, PTB2),
                 _servo1(1, &_serialEncoder, PTD5, PTD6, PTD3, PTB3),
                 _setPoint(2, 1),
                 _hid(64, 64)
{
    //Set send report length to maximum
    _send_report.length = 64;

    //Tempoary until EEPROm saving is implemented
    _properties = loadDefaultProperties();

    _servo0.setProperties(_properties.servo0Properties);
    _servo1.setProperties(_properties.servo1Properties);

    //dummy value until proper error handing is implemented
    _setPoint << 0
              << -(_properties.l0 + _properties.l1 / 1.5);

    //disable motors by default
    _enabled = false;
}

MyPAM::~MyPAM()
{
}

void MyPAM::update()
{
    recv_HID();

    set_motorPower(_enabled);

    _servo0.update();
    _servo1.update();

    send_HID();
}

Matrix MyPAM::getCurrentPositionVector()
{
    //Get current arm angles
    float a0 = _servo0.get_angle();
    float a1 = _servo1.get_angle();

    Matrix positionVector(2, 1); //Create empty matrix for position value

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

bool MyPAM::set_position(int x, int y)
{
    //check if operation is valid
    if (!checkCoordinateInput(x,y)) return false;

    //update setpoint vector
    _setPoint.Clear();
    _setPoint << x
              << y;

    //Calculate required motor angles
    Matrix angles = get_inverseKinematicPosition(_setPoint);

    //Set servomotors to move to corrisponding motor angles
    _servo0.set_angleSetpoint(angles.getNumber(1, 1));
    _servo1.set_angleSetpoint(angles.getNumber(2, 1));

    return true;
}

bool MyPAM::checkCoordinateInput(int x, int y)
{
    //calculate absolute distance from origin
    float mag = sqrt((float)(x*x + y*y));

    //check if arm cant reach out far enough
    if (mag > (_properties.l0 + _properties.l1)) return false;

    //check if coordinate is too close to centre
    if (mag < (_properties.l0 - _properties.l1)) return false;

    return true;
    
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

void MyPAM::send_HID()
{
    

    uint8_t controlByte = 0;

    if (_enabled) controlByte |= 0x01;

    //get position values
    Matrix Positionvector = getCurrentPositionVector();
    short x = (short)Positionvector.getNumber(1, 1);
    short y = (short)Positionvector.getNumber(2, 1);

    //get setpoint values
    Matrix setpointVector = getSetPointPositionVector();
    short sx = (short)(setpointVector.getNumber(1, 1));
    short sy = (short)(setpointVector.getNumber(2, 1));

    //get velocity values
    Matrix velocityVector = getVelocityVector();
    short vx = (short)(velocityVector.getNumber(1, 1));
    short vy = (short)(velocityVector.getNumber(2, 1));

    //get angle values
    short a0 = (short)(_servo0.get_angle() * 57.2958);
    short a1 = (short)(_servo1.get_angle() * 57.2958);

    //get angle velocity values
    short av0 = (short)(_servo0.get_angleV() * 57.2958);
    short av1 = (short)(_servo1.get_angleV() * 57.2958);

    //calculate angle setpoints
    Matrix angleSetpoint = get_inverseKinematicPosition(getSetPointPositionVector());
    short as0 = (short)(angleSetpoint.getNumber(1, 1) * 57.2958);
    short as1 = (short)(angleSetpoint.getNumber(2, 1) * 57.2958);

    //get current (and convert to mA)
    short i0 = (short)(_servo0._hbridge.get_current() * 1000.0f);
    short i1 = (short)(_servo1._hbridge.get_current() * 1000.0f);

    //Create report
    _send_report.data[0] = controlByte;

    _send_report.data[4] = x & 0xFF;
    _send_report.data[5] = (x >> 8) & 0xFF;

    _send_report.data[6] = y & 0xFF;
    _send_report.data[7] = (y >> 8) & 0xFF;

    _send_report.data[8] = sx & 0xFF;
    _send_report.data[9] = (sx >> 8) & 0xFF;

    _send_report.data[10] = sy & 0xFF;
    _send_report.data[11] = (sy >> 8) & 0xFF;

    _send_report.data[12] = vx & 0xFF;
    _send_report.data[13] = (vx >> 8) & 0xFF;

    _send_report.data[14] = vy & 0xFF;
    _send_report.data[15] = (vy >> 8) & 0xFF;

    _send_report.data[16] = a0 & 0xFF;
    _send_report.data[17] = (a0 >> 8) & 0xFF;

    _send_report.data[18] = a1 & 0xFF;
    _send_report.data[19] = (a1 >> 8) & 0xFF;

    _send_report.data[20] = av0 & 0xFF;
    _send_report.data[21] = (av0 >> 8) & 0xFF;

    _send_report.data[22] = av1 & 0xFF;
    _send_report.data[23] = (av1 >> 8) & 0xFF;

    _send_report.data[24] = as0 & 0xFF;
    _send_report.data[25] = (as0 >> 8) & 0xFF;

    _send_report.data[26] = as1 & 0xFF;
    _send_report.data[27] = (as1 >> 8) & 0xFF;

    _send_report.data[28] = i0 & 0xFF;
    _send_report.data[29] = (i0 >> 8) & 0xFF;

    _send_report.data[30] = i1 & 0xFF;
    _send_report.data[31] = (i1 >> 8) & 0xFF;

    //Send the report
    _hid.sendNB(&_send_report);
}

void MyPAM::recv_HID()
{
    //currently only accepts setpoint values as bytes 0 to 3
    if (_hid.readNB(&_recv_report))
    {
        bool motorEnable = (_recv_report.data[0] & 0x01);

       

        short testX = ((unsigned short)(_recv_report.data[4]) + (unsigned short)(_recv_report.data[5] << 8));
        short testY = ((unsigned short)(_recv_report.data[6]) + (unsigned short)(_recv_report.data[7] << 8));

        //SETTING UNREACHABLE LOCATIONS WILL CAUSE A CRASH
        set_position((int)testX, (int)testY);

        _enabled = motorEnable;
    }
}

void MyPAM::set_motorPower(bool enable)
{
    _servo0._hbridge.enabled(enable);
    _servo1._hbridge.enabled(enable);

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
