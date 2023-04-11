/// Portal robot with stepper motors on each axis and vacuum gripper
///
/// No feedback of the position, only speed and gripper on/off commands.
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "PinDefinitions.h"

/// @brief Portal robot with vacuum gripper
class PickRobot
{
public:
  /// @brief Struct to save a robot command, contains axis velocity and gripper activation
  struct Command
  {
    float axisVel[3];
    bool activateGripper;
  };
  /// @brief Enumeration for index - axis association
  enum axisIndex
  {
    x = 0,
    y = 1,
    z = 3
  };

private:
  AccelStepper xStepper;
  AccelStepper yStepper;
  AccelStepper zStepper;

public:
  /// @brief Initialize the stepper objects.
  PickRobot(/* args */);

  /// @brief Setup stepper so that the robot can be used.
  void setup();
  /// @brief Set the specified command.
  /// @param cmd Velocities in m/s with which the axes shall move and bool
  /// indicating if the vacuum gripper shall be activated.
  /// Absolute value of the commanded velocity per axis will be limited to **todo** m/s.
  void set(Command const &cmd);

  /// @brief Perform the movement command that was last set.
  /// This must be called regularly in the main loop.
  void run();
};