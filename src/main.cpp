
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


DigitalOut led(PTB21);
DigitalOut relay(PTC15);


MyPAM OranPAM;


int main()
{
  led = 1;
  relay = 1;

  OranPAM.set_position(-100,-400);

  while (1)
  {
    Timer t;
    t.start();

    OranPAM.update();

    while (t.read_ms() < (OranPAM._properties.interval * 1000.0f));
  }
}