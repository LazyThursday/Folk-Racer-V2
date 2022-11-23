#include <Arduino.h>

int eclipseSpeedNormalizer(float x, int c, int a)
{
    return a * sqrt(x * (2 * c - x)) / c;
}

int logSpeedNormalizer(float x, int w, float b)
{
    return w * (log(x) / log(b));
}

int angleNormalizer(float x)
{
    return (pow(x * -1.0, 3.0) / 0.01) + 90;
}