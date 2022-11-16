#include <Arduino.h>

#define number_INT 5     // Total number of interrupts
#define pingDelay 50     // How many milliseconds between each measurement ; keep > 5ms
#define debugDelay 200   // How many milliseconds between each Serial.print ; keep > 200ms
#define soundSpeed 343.0 // Speed of sound in m/s

volatile unsigned long travelTime[number_INT]; // Place to store traveltime of the pusle
volatile unsigned long startTime[number_INT];  // Place to store ping times (interrupt)
int distance[number_INT];                      // Calculated distances in cm
unsigned long lastPollMillis;
unsigned long lastDebugMillis;

#define trig 15

#define sensor0 21
#define sensor1 20
#define sensor2 19
#define sensor3 18
#define sensor4 3

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

void call_INT1()
{
    byte pinRead;
    // pinRead = digitalRead(pin_INT0);     // Slower ; Read pin 21
    pinRead = PIND >> 1 & B0001; // Faster ; Read pin 21/PD0
    interruptHandler(pinRead, 1);
}

void call_INT2()
{
    byte pinRead;
    // pinRead = digitalRead(pin_INT0);     // Slower ; Read pin 21
    pinRead = PIND >> 2 & B0001; // Faster ; Read pin 21/PD0
    interruptHandler(pinRead, 2);
}

void call_INT3()
{
    byte pinRead;
    // pinRead = digitalRead(pin_INT0);     // Slower ; Read pin 21
    pinRead = PIND >> 3 & B0001; // Faster ; Read pin 21/PD0
    interruptHandler(pinRead, 3);
}

void call_INT4()
{
    byte pinRead;
    // pinRead = digitalRead(pin_INT0);     // Slower ; Read pin 21
    pinRead = PINE >> 5 & B0001; // Faster ; Read pin 21/PD0
    interruptHandler(pinRead, 4);
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

void initSensors()
{
    pinMode(trig, OUTPUT);
    pinMode(sensor0, INPUT);
    pinMode(sensor1, INPUT);
    pinMode(sensor2, INPUT);
    pinMode(sensor3, INPUT);
    pinMode(sensor4, INPUT);

    attachInterrupt(digitalPinToInterrupt(sensor0), call_INT0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(sensor1), call_INT1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(sensor2), call_INT2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(sensor3), call_INT3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(sensor4), call_INT4, CHANGE);
}