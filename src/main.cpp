#include "mbed.h"
#include "USBSerial.h"
#include "buzzer.h"

USBSerial pc;
Serial dev(PTC17, PTC16);
DigitalOut led(PTB21);
Beep buzzer(PTB18);

struct serialSensorData
{
  long encoderCounts[2];
  long encoderCountRate[2];
  float potentiometer[2];
};

void flushSerialBuffer(void)
{
  char char1 = 0;
  while (dev.readable())
  {
    char1 = dev.getc();
  }
  int i = 1;
  
  return;
}

serialSensorData getSensorData()
{
  serialSensorData data;

  dev.putc('s');

  while (!dev.readable());
  u_int8_t encoderSerialBuffer[16];

  for (int i = 0; i < 16; i++)
  {
    encoderSerialBuffer[i] = dev.getc();
  }

  flushSerialBuffer();

  data.encoderCounts[0] = (((unsigned long)encoderSerialBuffer[3] << 24) | ((unsigned long)encoderSerialBuffer[2] << 16) | ((unsigned long)encoderSerialBuffer[1] << 8) | ((unsigned long)encoderSerialBuffer[0]));
  data.encoderCounts[1] = (((unsigned long)encoderSerialBuffer[7] << 24) | ((unsigned long)encoderSerialBuffer[6] << 16) | ((unsigned long)encoderSerialBuffer[5] << 8) | ((unsigned long)encoderSerialBuffer[4]));
  data.encoderCountRate[0] = (((unsigned long)encoderSerialBuffer[11] << 24) | ((unsigned long)encoderSerialBuffer[10] << 16) | ((unsigned long)encoderSerialBuffer[9] << 8) | ((unsigned long)encoderSerialBuffer[8]));
  data.encoderCountRate[1] = (((unsigned long)encoderSerialBuffer[15] << 24) | ((unsigned long)encoderSerialBuffer[14] << 16) | ((unsigned long)encoderSerialBuffer[13] << 8) | ((unsigned long)encoderSerialBuffer[12]));

  return data;

  //comment
}

int main()
{
  dev.baud(115200);
  led = 1;

  buzzer.beep(1700, 1);

  while (1)
  {
    Timer t;
    t.start();

    serialSensorData tempData = getSensorData();

    pc.printf("%i,%i,%i,%i, \n", tempData.encoderCounts[0], tempData.encoderCounts[1], tempData.encoderCountRate[0], tempData.encoderCountRate[1]);

    while (t.read_ms() < 16);
  }
}