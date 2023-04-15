/// Source file for PickRobot class

#include "PickRobot.h"

namespace
{
  /// @brief Setup stepper motor so that it can be used.
  /// Initialize position to 0, set max speed to 4000.
  /// @param stepper Stepper that should be set up.
  /// @param enablePin Enable pin of the stepper.
  void setupStepper(AccelStepper stepper, uint8_t enablePin)
  {
    stepper.setMaxSpeed(4000.F);
    stepper.setEnablePin(enablePin);
    stepper.setCurrentPosition(0);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
  }

  /// @brief Convert the specified velocity from m/s to steps/s
  /// @param[in] velMetersPerS Velocity in m/s. Must be an array of length 3, no error checking is done
  /// @param[out] velStepsPerS Velocity in steps/s. Must be an array of length 3, no error checking is done
  void convertMeterPerSecToStepsPerSec(float const velMeterPerS[3], float velStepsPerS[3])
  {
    // todo conversion
    float const gearRatio = 40.F;
    float const stepsPerRev = 200.F;
    float const revPerMeter = 300.F; // todo: Ganghoehe
    float const factor = revPerMeter * gearRatio * stepsPerRev;
    for (int i = 0; i < 3; ++i)
    {
      velStepsPerS[i] = velMeterPerS[i] * factor;
    }
  }
}

PickRobot::PickRobot(/* args */)
    : xStepper(AccelStepper::DRIVER, STEPX, DIRX),
      yStepper(AccelStepper::DRIVER, STEPY, DIRY),
      zStepper(AccelStepper::DRIVER, STEPZ, DIRZ)
{
}

void PickRobot::setup()
{
  pinMode(PIN_VAC, OUTPUT);

  setupStepper(xStepper, ENABLEX);
  setupStepper(yStepper, ENABLEY);
  setupStepper(zStepper, ENABLEZ);
}

void PickRobot::set(Command const &cmd)
{
  float velStepsPerS[3];
  convertMeterPerSecToStepsPerSec(cmd.axisVels, velStepsPerS);
  xStepper.setSpeed(velStepsPerS[x]);
  yStepper.setSpeed(velStepsPerS[y]);
  zStepper.setSpeed(velStepsPerS[z]);
  digitalWrite(PIN_VAC, cmd.activateGripper);
}


void PickRobot::printCommand(Command const &cmd, Stream& stream)
{
  stream.print(cmd.axisVels[0],10); stream.print(" ");
  stream.print(cmd.axisVels[1],10); stream.print(" ");
  stream.println(cmd.axisVels[2],10);
  stream.println(cmd.activateGripper);
}

void PickRobot::update()
{
  xStepper.runSpeed();
  yStepper.runSpeed();
  zStepper.runSpeed();
}
