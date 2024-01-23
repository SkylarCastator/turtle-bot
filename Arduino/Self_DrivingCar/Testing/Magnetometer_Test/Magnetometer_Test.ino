/**
* Skylar Castator 2023
* skylar.castator@gmail.com
* Code to manage data from the magnetometer
**/
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup()
{
  Serial.begin(9600);
  compass.init();
}

void loop()
{
  int x, y, z, a, b;
	char directionArray[3];
	
	compass.read();
  
	x = compass.getX();
	y = compass.getY();
	z = compass.getZ();
	
	a = compass.getAzimuth();
	
	b = compass.getBearing(a);

	compass.getDirection(directionArray, a);
  
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

	Serial.print(" Direction: ");
	Serial.print(directionArray[0]);
	Serial.print(directionArray[1]);
	Serial.print(directionArray[2]);

	Serial.println();
  delay(300);
}