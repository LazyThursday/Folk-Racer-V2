#include <Arduino.h>
#include <Servo.h>

Servo steering;

#define number_INT 1     // Total number of interrupts
#define pingDelay 50     // How many milliseconds between each measurement ; keep > 5ms
#define debugDelay 200   // How many milliseconds between each Serial.print ; keep > 200ms
#define soundSpeed 343.0 // Speed of sound in m/s

#define dcInput1 11
#define dcInput2 13

#define minSteering 40  // Minumum Angle for steering
#define MaxSteering 150 // Maximum Angle for steering

#define echo 2
#define trig 8

volatile unsigned long travelTime[number_INT]; // Place to store traveltime of the pusle
volatile unsigned long startTime[number_INT];  // Place to store ping times (interrupt)
float distance[number_INT];                    // Calculated distances in cm
unsigned long lastPollMillis;
unsigned long lastDebugMillis;

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

// Common function for interrupts
void interruptHandler(bool pinState, int nIRQ)
{
  unsigned long currentTime = micros(); // Get current time (in µs)
  if (pinState)
  {
    // If pin state has changed to HIGH -> remember start time (in µs)
    startTime[nIRQ] = currentTime;
  }
  else
  {
    // If pin state has changed to LOW -> calculate time passed (in µs)
    travelTime[nIRQ] = currentTime - startTime[nIRQ];
  }
}

void call_INT0()
{
  byte pinRead;
  // pinRead = digitalRead(pin_INT0);     // Slower ; Read pin 21
  pinRead = PIND & B0001; // Faster ; Read pin 21/PD0
  interruptHandler(pinRead, 0);
}

void doMeasurement()
{
  // First read will be 0 (no distance  calculated yet)
  // Read the previous result (pause interrupts while doing so)
  noInterrupts(); // cli()
  for (int i = 0; i < number_INT; i++)
  {
    distance[i] = travelTime[i] / 2.0 * (float)soundSpeed / 10000.0; // in cm
  }
  interrupts(); // sei();
  // Initiate next trigger
  // digitalWrite(triggerPin, LOW);  // rest of loop already takes > 2µs
  // delayMicroseconds(2);
  digitalWrite(trig, HIGH); // HIGH pulse for at least 10µs
  delayMicroseconds(10);
  digitalWrite(trig, LOW); // Set LOW again
}

void setup()
{
  Serial.begin(9600);
  steering.attach(A0);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(dcInput1, OUTPUT);
  pinMode(dcInput2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(echo), call_INT0, CHANGE); // ISR for INT0

  // reverse(255);
}

void loop()
{
  handleSerial();

  if (millis() - lastPollMillis >= pingDelay)
  {
    doMeasurement();
    lastPollMillis = millis();
  }

  Serial.println(distance[0]);
}