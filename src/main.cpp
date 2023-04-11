#include <Arduino.h>
#include <PacketSerial.h>
#include "PickRobot.h"
#include "PinDefinitions.h"

/// Serial communication with COBS encoded messages
PacketSerial packetSerial;

/// Portal robot used for picking tasks
PickRobot pickRobot;

/// @brief Send received command to the robot.
/// @param buffer Decoded command received via serial.
/// @param size Size of the decoded message in bytes.
void onPacketReceived(const uint8_t *buffer, size_t size)
{
  if (size < 13)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else // todo: rather check for size being exactly 13?!
  {
    PickRobot::Command cmd;
    memcpy(cmd.axisVel, buffer, 12); // todo: check float representation arduino vs linux PC
    cmd.activateGripper = static_cast<bool>(buffer[12]);
    pickRobot.set(cmd);
  }
}

/// @brief Setup serial communication using COBS and the robot.
void setup()
{
  packetSerial.begin(BAUDRATE);
  packetSerial.setPacketHandler(&onPacketReceived);

  pickRobot.setup();
}

/// @brief Read incoming serial messages and check for overflow
void loop()
{
  // Run the robot/stepper motors
  pickRobot.run();

  // Call update to receive, decode and process incoming packets.
  // if a new message was received, this will be handled in the callback
  // onPacketReceived
  packetSerial.update();

  // Check for a receive buffer overflow.
  if (packetSerial.overflow())
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}