/// Source file for PickRobot class

#include "PickRobot.h"

namespace
{
  /// @brief Setup stepper motor so that it can be used.
  /// Initialize position to 0, set max speed to 4000.
  /// @param stepper Stepper that should be set up.
  /// @param enablePin Enable pin of the stepper.
  void setupStepper(AccelStepper& stepper, uint8_t enablePin, float maxSpeed = 4000.F) //0,012323732 m/s
  {
    stepper.setMaxSpeed(maxSpeed);
    stepper.setEnablePin(enablePin);
    stepper.setCurrentPosition(0);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
  }

  /// @brief Convert the specified velocity from m/s to steps/s
  /// @param[in] velMetersPerS Velocity in m/s. Must be an array of length 3,
  /// no error checking is done
  /// @param[out] velStepsPerS Velocity in steps/s. Must be an array of length 3,
  /// no error checking is done
  void convertMeterPerSecToStepsPerSec(float const *const velMeterPerS,
                                       float *velStepsPerS)
  {
    float const stepsPerMeter = 154174.F; // gemessen mit alter Übersetzung und
                                          // Umrechnung nach Wechsel der Übersetzung
    for (int i = 0; i < 3; ++i)
    {
      velStepsPerS[i] = velMeterPerS[i] * stepsPerMeter;
    }
  }

  /// @brief  Set the speed to zero if the start or end position of an axis has been reached
  /// @param stepper Stepper to check
  /// @param endPin pin of the end-position switch
  /// @param isUpperLimit True if the pin corresponds to the switch at the upper end
  /// of the axis, false otherwise.
  void limitSpeedIfAtEnd(AccelStepper& stepper, uint8_t endPin, bool isUpperLimit)
  {
    if((digitalRead(endPin) == LOW) && ((stepper.speed() > 0.F) == isUpperLimit))
    {
      stepper.setSpeed(0.F);
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
  pinMode(PIN_MIN_ENDSTOP_X, INPUT_PULLUP);
  pinMode(PIN_MAX_ENDSTOP_X, INPUT_PULLUP);
  pinMode(PIN_MIN_ENDSTOP_Y, INPUT_PULLUP);
  pinMode(PIN_MAX_ENDSTOP_Y, INPUT_PULLUP);
  pinMode(PIN_MIN_ENDSTOP_Z, INPUT_PULLUP);
  pinMode(PIN_MAX_ENDSTOP_Z, INPUT_PULLUP);

  setupStepper(xStepper, ENABLEX, 3300.F);
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
  stream.print(cmd.axisVels[x],10); stream.print(" ");
  stream.print(cmd.axisVels[y],10); stream.print(" ");
  stream.println(cmd.axisVels[z],10);
  stream.println(cmd.activateGripper);
}

void PickRobot::checkLimits()
{
  limitSpeedIfAtEnd(xStepper, PIN_MAX_ENDSTOP_X, true);
  limitSpeedIfAtEnd(yStepper, PIN_MAX_ENDSTOP_Y, true);
  limitSpeedIfAtEnd(zStepper, PIN_MAX_ENDSTOP_Z, true);

  limitSpeedIfAtEnd(xStepper, PIN_MIN_ENDSTOP_X, false);
  limitSpeedIfAtEnd(yStepper, PIN_MIN_ENDSTOP_Y, false);
  limitSpeedIfAtEnd(zStepper, PIN_MIN_ENDSTOP_Z, false);
}

void PickRobot::update()
{
  checkLimits();
  xStepper.runSpeed();
  yStepper.runSpeed();
  zStepper.runSpeed();
}
