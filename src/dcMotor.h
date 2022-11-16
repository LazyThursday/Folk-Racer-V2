#include <Arduino.h>

#define right 3
#define left 4
#define front 2
#define frontRight 0
#define frontLeft 1

#define dcInput1 12
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
