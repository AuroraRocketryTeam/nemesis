#pragma once

void non_blocking_delay(unsigned long delayTime)
{
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= delayTime)
    {
        previousMillis = currentMillis;
    }
}