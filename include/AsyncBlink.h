/// Helper functions to enable blinking the build-in LED without using delay

#ifndef ASYNCBLINK_H
#define ASYNCBLINK_H

#include <Arduino.h>

// This only works for system time + led on/off time < long uint (24 days)

namespace async_blink
{
  int numFlashes, millisOffOn[2];
  int currentState = LOW;
  unsigned long changeLedStateTime;

  void blink(int const nFlashes, int const millisOn = 500, int const millisOff = 500)
  {
    if (numFlashes == 0 && currentState == LOW)
    {
      async_blink::numFlashes = nFlashes;
      async_blink::millisOffOn[0] = millisOff;
      async_blink::millisOffOn[1] = millisOn;
      digitalWrite(LED_BUILTIN, HIGH);
      currentState = HIGH;
      changeLedStateTime = millis() + millisOn;
    }
    else
    {
      // ignore blink update for now
    }
  }

  void ledXSec(int const sec)
  {
    blink(1, sec * 1000);
  }

  void update()
  {
    unsigned long now = millis();
    if (numFlashes && (now >= changeLedStateTime))
    {
      currentState = !currentState;
      digitalWrite(LED_BUILTIN, currentState);
      changeLedStateTime = now + millisOffOn[currentState];
      if (!currentState)
      {
        --numFlashes;
      }
    }
  }
}

#endif
