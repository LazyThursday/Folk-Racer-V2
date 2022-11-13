#include <Arduino.h>
#include "dcMotor.h"
#include "sensors.h"

Servo steering;

void handleSerial()
{
  String input;
  if (Serial.available() > 0)
  {
    input = Serial.readStringUntil('\n');
  }
  else
  {
    return;
  }

  String command = input.substring(0, 1);

  if (command == "a")
  {
    int angle = input.substring(1, 3).toInt();

    if (angle >= 0 && angle <= 180)
    {
      steering.write(angle);
      Serial.println("Success!");
    }
    else
    {
      Serial.println("Invalid angle");
    }
  }
  else if (command == "f")
  {
    int speed = input.substring(1, 4).toInt();

    forward(speed);

    Serial.println("speed Set to" + String(speed));
  }
  else if (command == "r")
  {
    int speed = input.substring(1, 4).toInt();

    reverse(speed);

    Serial.println("speed Set to" + String(speed));
  }
  else if (command == "s")
  {
    stopMotor();
    Serial.println("Motor stopped");
  }

  return;
}

void setup()
{
  Serial.begin(9600);
  steering.attach(A0);

  pinMode(trig, OUTPUT);

  pinMode(dcInput1, OUTPUT);
  pinMode(dcInput2, OUTPUT);

  attachInterrupts();

  // reverse(255);

  Serial.println("Starting");
}

void loop()
{
  doMeasurement();

  Serial1.println(String(distance[0]) + String(distance[1]));
}