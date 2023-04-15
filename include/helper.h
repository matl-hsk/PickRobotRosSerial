/// Helper functions for debugging etc.

#ifndef HELPER_H
#define HELPER_H

#include "Arduino.h"

/// @brief Print the buffer byte-wise as hex on the default Serial port
/// @param buffer Bytes that should be printed
/// @param size Length of the buffer/number of bytes that should be printed.
void printBytesAsHex(uint8_t const *const buffer, size_t size)
{
  for (int i = 0; i < size; ++i)
  {
    Serial.print(" 0x");
    if (buffer[i] < 16)
      Serial.print("0");
    Serial.print(buffer[i], HEX);
  }
  Serial.println();
}

#endif
