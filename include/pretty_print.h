#ifndef PRETTY_PRINT_H
#define PRETTY_PRINT_H

#include "unitree_legged_sdk/comm.h"

using namespace UNITREE_LEGGED_SDK;

void printLowState(const LowState& lowState);
void printMotorState(const std::array<MotorState, 20>& motorStates);
void printBmsState(const BmsState& bms);
void printIMU(const IMU& imu);

void printHighState(const HighState& highState);

#endif // PRETTY_PRINT_HS
