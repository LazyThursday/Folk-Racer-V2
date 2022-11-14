#include <Arduino.h>

#define dcInput1 5
#define dcInput2 6

#define minSteering 40  // Minumum Angle for steering
#define MaxSteering 150 // Maximum Angle for steering

void forward(int speed)
{
    digitalWrite(dcInput2, LOW);
    analogWrite(dcInput1, speed);
}

void reverse(int speed)
{
    digitalWrite(dcInput2, HIGH);
    analogWrite(dcInput1, speed);
}

void stopMotor()
{
    analogWrite(dcInput1, 0);
}

int normalizer(int x, int normalX, int normalY)
{
    return pow(x - normalX, 3) / 3000 + normalY;
}

int linearNormalizer(int x, int normalX, int normalY, int inversion = 1)
{
    return (x - normalX) * 2.5 * inversion + normalY;
}