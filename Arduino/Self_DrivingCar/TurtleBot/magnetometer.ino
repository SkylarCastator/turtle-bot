/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Code to manage data from the magnetometer
**/
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

bool enableSerialPrint = false;

void setupCompass()
{
  compass.init();
}

void readCompass()
{
  int x, y, z, a, b;
	
	compass.read();
  
	x = compass.getX();
	y = compass.getY();
	z = compass.getZ();
	
	a = compass.getAzimuth();
	
	b = compass.getBearing(a);

	printData(x, y, z, a, b);
}

void printData(int x, int y, int z, int a, int b)
{
  if (enableSerialPrint)
  {
    Serial.print("X: ");
    Serial.print(x);

    Serial.print(" Y: ");
    Serial.print(y);

    Serial.print(" Z: ");
    Serial.print(z);

    Serial.print(" Azimuth: ");
    Serial.print(a);

    Serial.print(" Bearing: ");
    Serial.print(b);

    char directionArray[3];
    compass.getDirection(directionArray, a);
    Serial.print(" Direction: ");
    Serial.print(directionArray[0]);
    Serial.print(directionArray[1]);
    Serial.print(directionArray[2]);

    Serial.println();
  }
}