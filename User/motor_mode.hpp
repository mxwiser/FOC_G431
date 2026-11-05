#pragma once

// Wire IDs are persistent. The FOC observer choice and six-step drive share
// one user-facing selector, but BLDC never runs the FOC observer/current loop.
enum MotorMode { MOTOR_FOC_ORTEGA = 0, MOTOR_FOC_MXLEMMING = 1, MOTOR_BLDC = 2 };

inline bool motor_mode_valid(unsigned mode) { return mode <= MOTOR_BLDC; }
