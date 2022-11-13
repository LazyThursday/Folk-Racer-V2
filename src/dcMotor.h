#include <Arduino.h>
#include <Servo.h>

#define dcInput1 11
#define dcInput2 13

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

int normalizer(int x)
{
    return pow(x - 50, 3) / 3000 + 90;
}

int linearNormalizer(int x, int normalX, int normalY)
{
    return (x - normalX) * 2.5 + normalY;
}