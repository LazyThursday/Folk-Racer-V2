#include <Arduino.h>
#include <Servo.h>
#include "sensors.h"
#include "dcMotor.h"

Servo steering;

#define maxDetectionDistance 130;

int sensorToPrint = 0;

long lastChange = 0;
int previousAngle = 90;
int tempAngle = 90;

bool on;

String distances[number_INT];

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
  else if ("k")
  {
  }

  return;
}

/*
@param x: your input
@param normalX: your center input
@param normalY: your center output
@param inversion: whether or not to flip the graph. -1 or 1. defaults to 1
*/
int angleNormalizer(int x, float normalX, int normalY, int inversion = 1)
{
  return (pow(x - normalX, 3) / pow(normalX, 1.75 + normalX / 240)) * inversion + normalY;
}

int speedNormalizer(int x)
{
  return exp(x / 7);
}

void handleMovement()
{
  if ((millis() - lastChange) < 350)
  {
    return;
  }
  if (distance[4] > 150)
  {
    distance[4] = 150;
  }
  if (distance[3] > 150)
  {
    distance[3] = 150;
  }
  tempAngle = 90 - ((distance[4] - distance[3]) / 2.5);

  // Serial.println(String(tempAngle) + " - " + String(distance[4]) + "cm");
  if ((tempAngle > 90 && previousAngle < 90) || (tempAngle < 90 && previousAngle > 90))
  {
    steering.write(90);
  }
  steering.write(tempAngle);

  if (distance[2] > 113)
  {
    distance[2] = 113;
  }
  // forward(speedNormalizer(distance[2]));

  Serial.println(tempAngle);
  lastChange = millis();
}

void setup()
{
  Serial.begin(9600);
  steering.attach(7);

  initSensors();

  pinMode(dcInput1, OUTPUT);
  pinMode(dcInput2, OUTPUT);
}

void loop()
{
  handleSerial();

  if (millis() - lastPollMillis >= pingDelay)
  {
    doMeasurement();

    lastPollMillis = millis();
  }

  handleMovement();
}