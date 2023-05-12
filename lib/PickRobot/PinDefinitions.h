/// Define the pins on the board

#ifndef PINDEFINITIONS_H
#define PINDEFINITIONS_H

/// motor pins
#define STEPX 46
#define DIRX 48
#define ENABLEX 62

#define STEPY 60
#define DIRY 61
#define ENABLEY 56

#define STEPZ 54
#define DIRZ 55
#define ENABLEZ 38
    
/// endstop pins
#define PIN_MIN_ENDSTOP_X 18    
#define PIN_MAX_ENDSTOP_X 19

#define PIN_MIN_ENDSTOP_Y 14
#define PIN_MAX_ENDSTOP_Y 15

#define PIN_MIN_ENDSTOP_Z 3
#define PIN_MAX_ENDSTOP_Z 2

/// vacuum gripper pin
#define PIN_VAC 9

#endif
