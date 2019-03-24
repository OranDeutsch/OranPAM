
/////USB FUNCTION SWITCH, HIDTEST FOR NORMAL USE AND SERIAL FOR DEBUGGING///
/////ONLY USE HIDTEST OR SERIAL IN THIS SPACE////
#define HID
/////NEVER HAVE HID AND SERIAL ENABLED/////
//***************************************//

/*
 * Includes
 */
#include "mbed.h"
#include "MyPAM.h"
#include "buzzer.h"

#ifdef SERIAL
#include "USBSerial.h"
#endif

#ifdef HIDTEST
#include "USBHID.h"
#endif

DigitalOut led(PTB21);
DigitalOut relay(PTC15);

Beep buzzer(PTB18);
MyPAM OranPAM;



#ifdef SERIAL
USBSerial pc;

void testPrint()
{
  Matrix posM = OranPAM.getCurrentPositionVector();
  Matrix velM = OranPAM.getVelocityVector();
  //pc.printf("%f,%f,%f,%f,\n", posM.getNumber(1, 1), posM.getNumber(2, 1), velM.getNumber(1, 1), velM.getNumber(2, 1));
  //pc.printf("%f , %f \n",OranPAM._servo0.get_angle() * 57,OranPAM._servo1.get_angle() * 57);
  //pc.printf("%f,%f,%f \n",(OranPAM._servo1.get_angle()),OranPAM._servo1._angleSetpoint,OranPAM._servo1._dutyCycle);
  //pc.printf("%f, %f,%f \n", servo1.get_angle(), servo1._dutyCycle, servo1._angleSetpoint);

  pc.printf("%f,%f, \n",OranPAM._servo0._hbridge.get_current(),OranPAM._servo1._hbridge.get_current());
}

#endif

int main()
{
  led = 1;
  relay = 1;

  buzzer.beep(1700, 1);


  OranPAM.set_position(-100,-400);

  while (1)
  {
    Timer t;
    t.start();

    OranPAM.update();

#ifdef SERIAL
    testPrint();
#endif

    //play emergancy sound if update takes longer than 16ms
    if (t.read_ms() > 16) buzzer.beep(1700, 1);

    while (t.read_ms() < 16);
  }
}