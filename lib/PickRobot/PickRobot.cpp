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
    float const speed = stepper.speed();
    if((digitalRead(endPin) == LOW) && (speed > 0.F) == isUpperLimit)
    {
        stepper.setSpeed(0.99F*speed);
        if (abs(stepper.speed()) < 1000)
        {
          stepper.setSpeed(0.F);
        }
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
  convertMeterPerSecToStepsPerSec(cmd.axisVels, desiredSpeeds);
  desiredSpeeds[x] = min(desiredSpeeds[x], xStepper.maxSpeed());
  desiredSpeeds[y] = min(desiredSpeeds[y], yStepper.maxSpeed());
  desiredSpeeds[z] = min(desiredSpeeds[z], zStepper.maxSpeed());
  desiredSpeeds[x] = max(desiredSpeeds[x], -xStepper.maxSpeed());
  desiredSpeeds[y] = max(desiredSpeeds[y], -yStepper.maxSpeed());
  desiredSpeeds[z] = max(desiredSpeeds[z], -zStepper.maxSpeed());

  digitalWrite(PIN_VAC, cmd.activateGripper);
}


void PickRobot::printCommand(Command const &cmd, Stream& stream)
{
  stream.print(cmd.axisVels[x],10); stream.print(" ");
  stream.print(cmd.axisVels[y],10); stream.print(" ");
  stream.println(cmd.axisVels[z],10);
  stream.println(cmd.activateGripper);
}

void PickRobot::enforceLimits()
{
  limitSpeedIfAtEnd(xStepper, PIN_MAX_ENDSTOP_X, true);
  limitSpeedIfAtEnd(yStepper, PIN_MAX_ENDSTOP_Y, true);
  limitSpeedIfAtEnd(zStepper, PIN_MAX_ENDSTOP_Z, true);

  limitSpeedIfAtEnd(xStepper, PIN_MIN_ENDSTOP_X, false);
  limitSpeedIfAtEnd(yStepper, PIN_MIN_ENDSTOP_Y, false);
  limitSpeedIfAtEnd(zStepper, PIN_MIN_ENDSTOP_Z, false);
}

void PickRobot::calcAndSetMotorSpeeds()
{
  AccelStepper* stepper[3] = {&xStepper, &yStepper, &zStepper};
  float const speedIncr = 5.F;
  for (int i = 0; i < 3; ++i)
  {
    float setSpeed;
    float const curSpeed = stepper[i]->speed();
    float const deltaSpeed = desiredSpeeds[i] - curSpeed;
    if (deltaSpeed > speedIncr)
    {
      setSpeed = curSpeed + speedIncr;
    }
    else if (deltaSpeed < -speedIncr)
    {
      setSpeed = curSpeed - speedIncr;
    }
    else
    {
      setSpeed = desiredSpeeds[i];
    }
    stepper[i]->setSpeed(setSpeed);
  }
}

void PickRobot::update()
{
  calcAndSetMotorSpeeds();
  enforceLimits();
  xStepper.runSpeed();
  yStepper.runSpeed();
  zStepper.runSpeed();
}
