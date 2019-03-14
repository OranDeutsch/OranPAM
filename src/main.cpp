#include "mbed.h"
#include "MyPAM.h"
#include "USBSerial.h"
#include "buzzer.h"

USBSerial pc;
DigitalOut led(PTB21);
Beep buzzer(PTB18);

MyPAM OranPAM; 


int main()
{
  led = 1;

  buzzer.beep(1700, 1);

  while (1)
  {
    Timer t;
    t.start();

    OranPAM.update();

    pc.printf("%f,%f,%f,%f,\n",OranPAM.servo0.get_angle(),OranPAM.servo1.get_angle(),OranPAM.servo0.get_angleV(),OranPAM.servo1.get_angleV());


    while (t.read_ms() < 16);
  }
}