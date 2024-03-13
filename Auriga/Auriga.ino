#include "MeAuriga.h"

#define LEDNUM  12
#define ALLLEDS 0

MeRGBLed led(0, LEDNUM);
MeUltrasonicSensor ultraSensor(PORT_7);

double d, factor;
int red, green;

void setup()
{
  led.setpin(44);
  Serial.begin(9600);
}

void loop()
{
  d = ultraSensor.distanceCm();

  if (d > 99) {
    red = 0;
    green = 255;
  } else {
    factor = d/100.0;
    red = (int)((1 - factor) * 255);
    green = (int)(factor * 255);
  }

  for (uint8_t t = 0; t < LEDNUM; t++ ) {
    led.setColorAt(t, red, green, 0);
  }
  led.show();
  
  Serial.print("Distance: ");
  Serial.print(d);
  Serial.println(" cm");

  delay(200);
}
