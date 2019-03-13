#include "mbed.h"
#include "SerialEncoder.h"
#include "USBSerial.h"
#include "buzzer.h"

USBSerial pc;
DigitalOut led(PTB21);
Beep buzzer(PTB18);
SerialEncoder serialEncoder(PTC17,PTC16);

int main()
{
  led = 1;

  buzzer.beep(1700, 1);

  serialEncoder.reset();

  while (1)
  {
    Timer t;
    t.start();

    serialSensorData tempData = serialEncoder.getSensorData();

    pc.printf("%i,%i,%i,%i, \n", tempData.encoderCounts[0], tempData.encoderCounts[1], tempData.encoderCountRate[0], tempData.encoderCountRate[1]);

    while (t.read_ms() < 16);
  }
}