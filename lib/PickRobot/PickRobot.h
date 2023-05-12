/// Header file for PickRobot class

#ifndef PICKROBOT_H
#define PICKROBOT_H

#include <AccelStepper.h>
#include <MultiStepper.h>
#include "PinDefinitions.h"

/// @brief Portal robot with stepper motors on each axis and vacuum gripper
///
/// No feedback of the position, only speed and gripper on/off commands.
class PickRobot
{
public:
  /// @brief Struct to save a robot command, contains axis velocity and gripper activation
  struct Command
  {
    float axisVels[3];
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
  /// @param cmd Velocities in m/s with which the axes shall move and bool
  /// indicating if the vacuum gripper shall be activated.
  /// Absolute value of the commanded velocity per axis will be limited to **todo** m/s.
  void set(Command const &cmd);

  /// @brief Print the specified command to the specified stream
  /// @param cmd Command that should be printed
  /// @param stream Stream (e.g. Serial) to which the command shall be printed
  static void printCommand(Command const &cmd, Stream &stream);

private:
  /// @brief Checks if a limit has been hit and disables the movement on that
  /// axis if it has been hit.
  void checkLimits();

  /// @brief Stepper motor for x-axis
  AccelStepper xStepper;
  /// @brief Stepper motor for y-axis
  AccelStepper yStepper;
  /// @brief Stepper motor for z-axis
  AccelStepper zStepper;
};

#endif
