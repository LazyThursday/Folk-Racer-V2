#include <Arduino.h>

int speedNormalizer(float x, int b)
{
    return sqrt(x * (2 * b - x));
}

int angleNormalizer(float x)
{
    return (pow(x * -1.0, 3.0) / 0.007) + 90;
}