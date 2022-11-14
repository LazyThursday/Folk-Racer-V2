#include <Arduino.h>
#include <Servo.h>
#include "sensors.h"
#include "dcMotor.h"

Servo steering;

int sensorToPrint = 0;

int myAngle = 90;

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
  else if (command == "u")
  {
    sensorToPrint = input.substring(1).toInt();

    Serial.println("\n\n Showing sensor " + String(sensorToPrint) + "\n\n");
    delay(1500);
  }

  return;
}

int getAverage(int arr[20])
{
  int total = 0;

  for (int i = 0; i <= 20; i++)
  {
    total += arr[i];
  }

  return total / 20;
}

void handleSteering()
{
  if (distance[1] > 100)
  {
    distance[1] = 100;
  }
  int temp_angle = linearNormalizer(distance[1], 30, 90, -1);
  if (abs(myAngle - temp_angle) < 10)
  {
    return;
  }
  if (temp_angle > 160)
  {
    temp_angle = 160;
  }
  else if (temp_angle < 20)
  {
    temp_angle = 20;
  }
  myAngle = temp_angle;
  steering.write(myAngle);
}

void setup()
{
  Serial.begin(9600);
  steering.attach(A1);

  initSensors();

  pinMode(dcInput1, OUTPUT);
  pinMode(dcInput2, OUTPUT);

  reverse(255);
}

void loop()
{
  handleSerial();

  if (millis() - lastPollMillis >= pingDelay)
  {
    doMeasurement();
    lastPollMillis = millis();
  }

  handleSteering();
  delay(50);
}