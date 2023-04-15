#include <Arduino.h>
#include <PacketSerial.h>

#include <ucdr/microcdr.h>

#include "AsyncBlink.h"
#include "PickRobot.h"
#include "PinDefinitions.h"
#include "RosDefines.h"

/// Serial communication with COBS encoded messages
PacketSerial packetSerial;

/// Portal robot used for picking tasks
PickRobot pickRobot;

/// @brief Signal an error by turning the led on for 1sec. Optional parameter to
/// turn the led on multiple times (blink).
/// @param numFlashes  Number of times the led should be turned on
void signalError(int const numFlashes = 1)
{
  async_blink::blink(numFlashes, 1000, 500);
}

/// @brief Signal okay by briefly flashing the LED 3 times
void signalOkay()
{
  async_blink::blink(3, 50, 50);
}

/// @brief Deserialize (CDR unencode) a ROS2 msg that should containt a PickRobot::Command.
/// Does check if this is a valid ROS2 message.
/// @param[in] buffer ROS2 msg buffer (unencoded (COBS) already)
/// @param[in] size max size of the buffer
/// @param[out] cmd Deserialized robot command
/// @return integer signalling if message could be unencoded and matched to a valid PickRobot::Command
int deserializeROSRobotCmdMessage(uint8_t const* const buffer, size_t const size, PickRobot::Command &cmd)
{
  if (size <= sizeof(ros2::COBSHeader))
  {
    return -1;
  }
  ros2::COBSHeader *header = (ros2::COBSHeader *)buffer;

  uint8_t const robotCmdTopicId = 2;
  if (header->topic_ID != robotCmdTopicId)
  {
    return -2;
  }

  uint16_t payload_len = (uint16_t)header->payload_len_h << 8U | header->payload_len_l;
  if ((size - sizeof(ros2::COBSHeader)) < payload_len)
  {
    return -3;
  }

  uint16_t read_crc = (uint16_t)header->crc_h << 8U | header->crc_l;
  uint16_t calc_crc = ros2::crc16(&buffer[0] + sizeof(ros2::COBSHeader), payload_len);
  if (read_crc != calc_crc)
  {
    return -4;
  }

  ucdrBuffer reader;
  uint8_t ucdrDataBuffer[payload_len];
  memcpy(ucdrDataBuffer, buffer + sizeof(ros2::COBSHeader), payload_len);

  ucdr_init_buffer(&reader, ucdrDataBuffer, payload_len);
  ucdr_deserialize_array_float(&reader, cmd.axisVels, 3);

  ucdr_init_buffer(&reader, ucdrDataBuffer + sizeof(PickRobot::Command::axisVels), 1);
  ucdr_deserialize_bool(&reader, &cmd.activateGripper);
  return 1;
}

/// @brief Send received command to the robot.
/// @param buffer Decoded command received via serial.
/// @param size Size of the decoded message in bytes.
void onPacketReceived(const uint8_t *buffer, size_t size)
{
  PickRobot::Command cmd;
  int cmdReadReturnCode = deserializeROSRobotCmdMessage(buffer, size, cmd);
  if (cmdReadReturnCode)
  {
    pickRobot.set(cmd);
    signalOkay();
  }
  else
  {
    signalError(-cmdReadReturnCode);
  }
}

/// @brief Setup serial communication using COBS and the robot.
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  packetSerial.begin(BAUDRATE);
  packetSerial.setPacketHandler(&onPacketReceived);

  pickRobot.setup();

  signalOkay();
}

/// @brief Read incoming serial messages and check for overflow
void loop()
{
  // Run the robot/stepper motors
  pickRobot.update();

  // Call update to receive, decode and process incoming packets.
  // if a new message was received, this will be handled in the callback
  // onPacketReceived
  packetSerial.update();

  async_blink::update();

  // Check for a receive buffer overflow.
  if (packetSerial.overflow())
  {
    async_blink::ledXSec(3000);
  }
}