
/////USB FUNCTION SWITCH, HIDTEST FOR NORMAL USE AND SERIAL FOR DEBUGGING///
/////ONLY USE HIDTEST OR SERIAL IN THIS SPACE////
#define HIDTEST
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
Beep buzzer(PTB18);
MyPAM OranPAM;

#ifdef SERIAL
USBSerial pc;

void testPrint()
{
  Matrix posM = OranPAM.getPositionVector();
  Matrix velM = OranPAM.getVelocityVector();
  pc.printf("%f,%f,%f,%f,\n", posM.getNumber(1, 1), posM.getNumber(2, 1), velM.getNumber(1, 1), velM.getNumber(2, 1));

  //pc.printf("%f,%f,%f,%f,\n",OranPAM._servo0.get_angle()*57,OranPAM._servo1.get_angle()*57,OranPAM._servo0.get_angleV(),OranPAM._servo1.get_angleV());
}

#endif

#ifdef HIDTEST

USBHID hid(8, 8);
HID_REPORT send_report;
HID_REPORT recv_report;

//Provisionial HID update method
void HIDTest()
{

  if (hid.readNB(&recv_report))
  {
   

    short testX = ((unsigned short)(recv_report.data[0]) + (unsigned short)(recv_report.data[1] << 8));
    short testY = ((unsigned short)(recv_report.data[2]) + (unsigned short)(recv_report.data[3] << 8));

    //SETTING UNREACHABLE LOCATIONS WILL CAUSE A CRASH
    OranPAM.set_position((int)testX,(int)testY);


     led = !led;

  }

  Matrix Positionvector = OranPAM.getCurrentPositionVector();
  Matrix setpointVector = OranPAM.get_inverseKinematicPosition(OranPAM.getSetPointPositionVector());
  
  
  send_report.length = 8;

  short x = (short)Positionvector.getNumber(1, 1);
  short y = (short)Positionvector.getNumber(2, 1);

  short sx = (short)(setpointVector.getNumber(1,1) * 57);
  short sy = (short)(setpointVector.getNumber(2,1) * 57);

  //Create report
  send_report.data[0] = x & 0xFF;
  send_report.data[1] = (x >> 8) & 0xFF;

  send_report.data[2] = y & 0xFF;
  send_report.data[3] = (y >> 8) & 0xFF;

  send_report.data[4] = sx & 0xFF;
  send_report.data[5] = (sx >> 8) & 0xFF;

  send_report.data[6] = sy & 0xFF;
  send_report.data[7] = (sy >> 8) & 0xFF;

   //Send the report
  hid.sendNB(&send_report);
}
#endif

int main()
{
  led = 1;

  buzzer.beep(1700, 1);

  while (1)
  {
    Timer t;
    t.start();

    OranPAM.update();
    

#ifdef SERIAL
    testPrint();
#endif

#ifdef HIDTEST
    HIDTest();
#endif

    if (t.read_ms() > 16)
    {
      buzzer.beep(1700, 1);
    }

    while (t.read_ms() < 16)
      ;
  }
}