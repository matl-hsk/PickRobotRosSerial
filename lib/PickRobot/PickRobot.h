/// Header file for PickRobot class

#ifndef PICKROBOT_H
#define PICKROBOT_H

#include <AccelStepper.h>
#include <MultiStepper.h>
#include "PinDefinitions.h"

/// @brief Portal robot with stepper motors on each axis and vacuum gripper
///
/// No feedback of the position, only accel and gripper on/off commands.
class PickRobot
{
public:
  /// @brief Struct to save a robot command, contains axis accel and gripper activation
  struct Command
  {
    float axisAccels[3];
    bool activateGripper;
  };

  /// @brief Enumeration for index - axis association
  enum axisIndex
  {
    x = 0,
    y = 1,
    z = 2
  };

public:
  /// @brief Initialize the stepper objects.
  PickRobot();

  /// @brief Setup stepper so that the robot can be used.
  void setup();

  /// @brief Perform the movement command that was last set.
  /// This must be called regularly in the main loop.
  void update();

  /// @brief Set the specified command.
  /// @param cmd Accelerations in m/s^2 with which the axes shall move and bool
  /// indicating if the vacuum gripper shall be activated.
  /// If the maximum speed is reached, the acceleration will be limited to 0.
  void set(Command const &cmd);

  /// @brief Print the specified command to the specified stream
  /// @param cmd Command that should be printed
  /// @param stream Stream (e.g. Serial) to which the command shall be printed
  static void printCommand(Command const &cmd, Stream &stream);

private:
  /// @brief Checks if a limit has been hit and disables the movement on that
  /// axis if it has been hit.
  void enforceAxisLimits();

  /// @brief Set motor accelerations to desired values
  void setMotorAccels();

  /// @brief Stepper motor for x-axis
  AccelStepper xStepper;
  /// @brief Stepper motor for y-axis
  AccelStepper yStepper;
  /// @brief Stepper motor for z-axis
  AccelStepper zStepper;

  /// @brief  Desired speeds for each axis in steps/s
  float desiredAccels[3];
};

#endif
