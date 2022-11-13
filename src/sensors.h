#include <Arduino.h>

#define number_INT 2     // Total number of interrupts
#define pingDelay 10     // How many milliseconds between each measurement ; keep > 5ms
#define debugDelay 250   // How many milliseconds between each Serial.print ; keep > 200ms
#define soundSpeed 343.0 // Speed of sound in m/s

#define trig 4

float distance[number_INT]; // Calculated distances in cm

volatile unsigned long travelTime[number_INT]; // Place to store traveltime of the pusle
volatile unsigned long startTime[number_INT];  // Place to store ping times (interrupt)
unsigned long lastPollMillis;
unsigned long lastDebugMillis;

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
    pinRead = PIND << 1 & B0001; // Faster ; Read pin 21/PD0
    interruptHandler(pinRead, 1);
}

void call_INT2()
{
    byte pinRead;
    // pinRead = digitalRead(pin_INT0);     // Slower ; Read pin 21
    pinRead = PIND << 2 & B0001; // Faster ; Read pin 21/PD0
    interruptHandler(pinRead, 2);
}

void call_INT3()
{
    byte pinRead;
    // pinRead = digitalRead(pin_INT0);     // Slower ; Read pin 21
    pinRead = PIND << 3 & B0001; // Faster ; Read pin 21/PD0
    interruptHandler(pinRead, 3);
}

void call_INT4()
{
    byte pinRead;
    // pinRead = digitalRead(pin_INT0);     // Slower ; Read pin 21
    pinRead = PINE & B0001; // Faster ; Read pin 21/PD0
    interruptHandler(pinRead, 4);
}

void doMeasurement()
{
    if (!(millis() - lastPollMillis >= pingDelay))
    {
        return;
    }
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

    lastPollMillis = millis();
}

void attachInterrupts()
{
    attachInterrupt(digitalPinToInterrupt(21), call_INT0, CHANGE); // ISR for INT0
    attachInterrupt(digitalPinToInterrupt(20), call_INT1, CHANGE); // ISR for INT1
    attachInterrupt(digitalPinToInterrupt(19), call_INT2, CHANGE); // ISR for INT2
    attachInterrupt(digitalPinToInterrupt(18), call_INT3, CHANGE); // ISR for INT2
}
