#include "MyPAM.h"

MyPAM::MyPAM() : _serialEncoder(PTC17, PTC16),
                 _servo0(0, &_serialEncoder, PTC11, PTC12, PTC9, PTB2),
                 _servo1(1, &_serialEncoder, PTD5, PTD6, PTD3, PTB3),
                 _setPoint(2, 1),
                 _hid(64, 64),
                 _i2cEEPROM(D14, D15, 0xa0, 32 * 1024),
                 _buzzer(PTB18)
{
    //Sent send report length to maximum
    _send_report.length = 64;

    //Tempoary until EEPROm saving is implemented
    //_properties = loadDefaultProperties();
    //saveProperties(loadDefaultProperties());

    //gets existing properties from EEPROM
    _properties = loadProperties();

    //send properties to servos
    _servo0.setProperties(_properties.servo0Properties);
    _servo1.setProperties(_properties.servo1Properties);

    //dummy value until proper error handing is implemented
    _setPoint << 0
              << -(_properties.l0 + _properties.l1 / 1.5);

    //disable motors by default
    _enabled = false;

    //only send HID when update command is recieved
    _hidSendENabled = false;

    //Initalise i2c EEPROM
    _i2cEEPROM.init();
}

MyPAM::~MyPAM()
{
}

void MyPAM::update()
{
    recv_HID(); //Check is a command has been recieved

    set_motorPower(_enabled); //disables motots immedatly if enabled is set false

    _servo0.update(); //Update servo
    _servo1.update();

    send_HID(); //send current status over HID
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
    if (!checkCoordinateInput(x, y))
        return false;

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
    float mag = sqrt((float)(x * x + y * y));

    //check if arm cant reach out far enough
    if (mag > (_properties.l0 + _properties.l1))
        return false;

    //check if coordinate is too close to centre
    if (mag < (_properties.l0 - _properties.l1))
        return false;

    return true;
}

Matrix MyPAM::get_inverseKinematicPosition(Matrix position)
{
    //Basic trigometric inverse kinematics
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

    //append calculated angles to matrix
    angles << theta0
           << theta1;

    return angles;
}

void MyPAM::send_HID()
{
    //sets the fist but of the control byte depending on if the motors are enabled
    uint8_t controlByte = 0;

    if (_enabled)
        controlByte |= 0x01;

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
    _send_report.data[3] = controlByte;
    shortToByteArray(_send_report.data, x, 4);
    shortToByteArray(_send_report.data, y, 6);
    shortToByteArray(_send_report.data, sx, 8);
    shortToByteArray(_send_report.data, sy, 10);
    shortToByteArray(_send_report.data, vx, 12);
    shortToByteArray(_send_report.data, vy, 14);
    shortToByteArray(_send_report.data, a0, 16);
    shortToByteArray(_send_report.data, a1, 18);
    shortToByteArray(_send_report.data, as0, 20);
    shortToByteArray(_send_report.data, as1, 22);
    shortToByteArray(_send_report.data, av0, 24);
    shortToByteArray(_send_report.data, av1, 26);
    shortToByteArray(_send_report.data, i0, 28);
    shortToByteArray(_send_report.data, i1, 30);
    _send_report.data[0] = 0;

    //Send the report
    _hid.sendNB(&_send_report);
}

void MyPAM::recv_HID()
{
    //currently only accepts setpoint values as bytes 0 to 3
    if (_hid.readNB(&_recv_report))
    {
        if (_recv_report.data[0] != 0)
            _buzzer.beep(1700, 0.1);

        //check first byte for what command is being sent
        switch (_recv_report.data[0])
        {
        case 0:
            getHIDMotorsEnabled(); //Executes "enable motors" command
            break;
        case 1:
            getHIDSetPoint(); //Executes "set set-point" command
            break;
        case 2:
            getHIDAssistanceFactor(); //Executes "set force assistance factor" command
            break;
        case 3:
            sendGlobalProperties(); //send the global (non-servo) properties over HID
            break;
        case 4:
            sendServoProperties(0); //Send servo 0 properties over HID
            break;
        case 5:
            sendServoProperties(1); //Send servo 1 properties over HID
            break;
        case 6:
            getGlobalProperties(); //Sets the global properties based on incoming HID report
            break;
        case 7:
            getServoProperties(0); //Sets servo 0 properties based on incoming HID report
            break;
        case 8:
            getServoProperties(1); //Sets servo 1 properties based on incoming HID report
            break;
        case 9:
        {
            //Loads the default properties
            _properties = loadDefaultProperties();

            _servo0.setProperties(_properties.servo0Properties);
            _servo1.setProperties(_properties.servo1Properties);
            break;
        }
        case 10:
            saveProperties(_properties); //Saves the current properties to EEPROM
            break;

        case 11:
            calibratePotentiometers();
            break;
        }
    }
}

void MyPAM::getHIDSetPoint()
{
    _hidSendENabled = true;

    //Bitshifts the incoming report data into X and Y as 16 bit ints
    short testX = ((unsigned short)(_recv_report.data[4]) + (unsigned short)(_recv_report.data[5] << 8));
    short testY = ((unsigned short)(_recv_report.data[6]) + (unsigned short)(_recv_report.data[7] << 8));

    set_position((int)testX, (int)testY);
}

void MyPAM::getHIDMotorsEnabled()
{
    bool motorEnable = (_recv_report.data[4] & 0x01);
    _enabled = motorEnable;
}

void MyPAM::getHIDAssistanceFactor()
{
    short f = ((unsigned short)(_recv_report.data[4]) + (unsigned short)(_recv_report.data[5] << 8));
    _assistanceFactor = f;

    _servo0._hbridge.setAssistanceFactor(f);
    _servo1._hbridge.setAssistanceFactor(f);
}

void MyPAM::getGlobalProperties()
{
    ///Byte 0 = command byte
    ///byte 4,5 = l0
    ///byte 6,7 = l1
    ///byte 8-11 = interval

    _properties.l0 = ByteArrayToShort(_recv_report.data, 4);
    _properties.l1 = ByteArrayToShort(_recv_report.data, 6);
    _properties.interval = ByteArrayToFloat(_recv_report.data, 8);
}

void MyPAM::getServoProperties(int servoID)
{

    ///byte 0 = command byte
    ///byte 4,5 = pulses per revolution
    ///byte 6,7 = offset (degrees)
    ///byte 8-11 = gear ratio
    ///byte 12-15 = P
    ///byte 16-19 = I
    ///byte 20-23 = D
    ///byte 24,25 = max angle (degrees)
    ///byte 26,27 = min angle (degrees)
    ///byte 28,31 = max duty cycle (decimal)

    //Properties are loaded from current ones for values that are not user-editable
    ServoMotorProperties tempProperties = _properties.servo0Properties;
    if (servoID == 1)
        tempProperties = _properties.servo1Properties;

    //append new properties to tempory ServoMotorProperties struct
    tempProperties.pulsesPerRevolution = ByteArrayToShort(_recv_report.data, 4);
    tempProperties.offset = (float)ByteArrayToShort(_recv_report.data, 6) / 57.296f;
    tempProperties.gearRatio = ByteArrayToFloat(_recv_report.data, 8);
    tempProperties.p = ByteArrayToFloat(_recv_report.data, 12);
    tempProperties.i = ByteArrayToFloat(_recv_report.data, 16);
    tempProperties.d = ByteArrayToFloat(_recv_report.data, 20);
    tempProperties.maxAngle = (float)ByteArrayToShort(_recv_report.data, 24) / 57.296f;
    tempProperties.minAngle = (float)ByteArrayToShort(_recv_report.data, 26) / 57.296f;
    tempProperties.maxDutyCycle = ByteArrayToFloat(_recv_report.data, 28);

    //Sets temperory properties to become the existing properties
    if (servoID == 0)
    {
        _properties.servo0Properties = tempProperties;
        _servo0.setProperties(_properties.servo0Properties);
    }
    if (servoID == 1)
    {
        _properties.servo1Properties = tempProperties;
        _servo1.setProperties(_properties.servo1Properties);
    }
}

void MyPAM::sendGlobalProperties()
{

    _send_report.data[0] = 1;

    shortToByteArray(_send_report.data, _properties.l0, 4);
    shortToByteArray(_send_report.data, _properties.l1, 6);
    floatToByteArray(_send_report.data, _properties.interval, 8);

    _hid.sendNB(&_send_report);
}

void MyPAM::sendServoProperties(int servoID)
{
    ServoMotorProperties sendProperties = _properties.servo0Properties;

    if (servoID == 1)
        sendProperties = _properties.servo1Properties;

    ///byte 0 = command byte
    ///byte 4,5 = pulses per revolution
    ///byte 6,7 = offset (degrees)
    ///byte 8-11 = gear ratio
    ///byte 12-15 = P
    ///byte 16-19 = I
    ///byte 20-23 = D
    ///byte 24,25 = max angle (degrees)
    ///byte 26,27 = min angle (degrees)
    ///byte 28,31 = max duty cycle (decimal)

    _send_report.data[0] = servoID + 2;
    shortToByteArray(_send_report.data, (short)sendProperties.pulsesPerRevolution, 4);
    shortToByteArray(_send_report.data, (short)(sendProperties.offset * 57.296), 6);
    floatToByteArray(_send_report.data, sendProperties.gearRatio, 8);
    floatToByteArray(_send_report.data, sendProperties.p, 12);
    floatToByteArray(_send_report.data, sendProperties.i, 16);
    floatToByteArray(_send_report.data, sendProperties.d, 20);
    shortToByteArray(_send_report.data, (short)(sendProperties.maxAngle * 57.296), 24);
    shortToByteArray(_send_report.data, (short)(sendProperties.minAngle * 57.296), 26);
    floatToByteArray(_send_report.data, sendProperties.maxDutyCycle, 28);

    _hid.sendNB(&_send_report);
}

void MyPAM::set_motorPower(bool enable)
{
    _servo0._hbridge.enabled(enable);
    _servo1._hbridge.enabled(enable);
}

void MyPAM::calibratePotentiometers()
{
    _serialEncoder.calibrate();
}

MyPAMProperties MyPAM::loadDefaultProperties()
{
    MyPAMProperties tempProperties;

    //Provisionial link length values
    tempProperties.l0 = 358;
    tempProperties.l1 = 228;
    tempProperties.interval = 1.0f / 60.0f;

    //Provisionial motor values
    tempProperties.servo0Properties.pulsesPerRevolution = 1024;
    tempProperties.servo0Properties.gearRatio = -40.0f;
    tempProperties.servo0Properties.offset = 0.0f;

    tempProperties.servo1Properties.pulsesPerRevolution = 1024;
    tempProperties.servo1Properties.gearRatio = 70.0f;
    tempProperties.servo1Properties.offset = 0.0f;

    //Provisionial PID values
    tempProperties.servo0Properties.p = 12;
    tempProperties.servo0Properties.i = 0;
    tempProperties.servo0Properties.d = 0;
    tempProperties.servo0Properties.PIDinterval = tempProperties.interval;
    tempProperties.servo0Properties.minAngle = -3.14;
    tempProperties.servo0Properties.maxAngle = 3.14;
    tempProperties.servo0Properties.maxDutyCycle = 0.3;

    tempProperties.servo1Properties.p = 12;
    tempProperties.servo1Properties.i = 0;
    tempProperties.servo1Properties.d = 0;
    tempProperties.servo1Properties.PIDinterval = tempProperties.interval;
    tempProperties.servo1Properties.minAngle = -3.14;
    tempProperties.servo1Properties.maxAngle = 3.14;
    tempProperties.servo1Properties.maxDutyCycle = 0.25;

    return tempProperties;
}

void MyPAM::saveProperties(MyPAMProperties properties)
{

    unsigned int setting_block_size = ceil(sizeof(MyPAMProperties) / (double)BLOCK_SIZE) * BLOCK_SIZE;
    char *buffer = (char *)malloc(setting_block_size);

    memcpy(buffer, &properties, sizeof(properties));

    _i2cEEPROM.program(buffer, 0, setting_block_size);

    wait_ms(6);
    //_buzzer.beep(1700, 0.02);
}

MyPAMProperties MyPAM::loadProperties()
{

    //need to implement outdated EEPROM detection, version number should be added or base on size of struct

    MyPAMProperties tempProperties;

    unsigned int setting_block_size = ceil(sizeof(tempProperties) / (double)BLOCK_SIZE) * BLOCK_SIZE;
    char *buffer = (char *)malloc(setting_block_size);
    memset(buffer, 0, sizeof(buffer));

    if (_i2cEEPROM.read(buffer, 0, setting_block_size) == 0)
    {
        memcpy(&tempProperties, buffer, sizeof(tempProperties));
        wait_ms(6);
    }
    else
    {
    }

    if (sizeof(tempProperties) != sizeof(loadDefaultProperties()))
    {
        _buzzer.beep(1700, 1);
    }
    return tempProperties;
}