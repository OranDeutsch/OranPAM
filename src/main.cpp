#include "mbed.h"
#include "MyPAM.h"
#include "buzzer.h"

#include "USBSerial.h"
//#include "USBHID.h"

DigitalOut led(PTB21);
Beep buzzer(PTB18);
MyPAM OranPAM;


USBSerial pc;

void testPrint()
{
  Matrix tempMatrix = OranPAM.getPositionVector();
  pc.printf("%f,%f,\n", tempMatrix.getNumber(1, 1), tempMatrix.getNumber(2, 1));

  //pc.printf("%f,%f,%f,%f,\n",OranPAM._servo0.get_angle()*57,OranPAM._servo1.get_angle()*57,OranPAM._servo0.get_angleV(),OranPAM._servo1.get_angleV());
}


/*
USBHID hid(8, 8);
HID_REPORT send_report;
HID_REPORT recv_report;

void HIDTest(Matrix vector)
{
  send_report.length = 8;
  
  int x = (int)tempMatrix.getNumber(1, 1);
  int y = -(int)tempMatrix.getNumber(2, 1);

  //Create report
  send_report.data[0] = x & 0xFF;
  send_report.data[1] = (x >> 8) & 0xFF;

  send_report.data[2] = y & 0xFF;
  send_report.data[3] = (y >> 8) & 0xFF;

  //Send the report
  hid.sendNB(&send_report);

  if (hid.readNB(&recv_report))
  {
  }
}

*/

int main()
{
  
  led = 1;

  buzzer.beep(1700, 1);

  while (1)
  {
    Timer t;
    t.start();

    OranPAM.update();
    Matrix tempMatrix = OranPAM.getPositionVector();

    //HIDTest(tempMatrix);
    testPrint();

    while (t.read_ms() < 16)
      ;
  }
}