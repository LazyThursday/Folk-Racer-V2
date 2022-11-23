#include <Arduino.h>

int eclipseSpeedNormalizer(float x, int b, int a)
{
    return a * sqrt(x * (2 * b - x)) / b;
}

int logSpeedNormalizer(float x, int w, float b)
{
    return w * (log(x) / log(b));
}

int angleNormalizer(float x)
{
    return (pow(x * -1.0, 3.0) / 0.02) + 90;
}