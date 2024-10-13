

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "pretty_print.h"
#include <iostream>
#include <iomanip>


void printIMU(const IMU& imu) {
    std::cout << std::fixed << std::setprecision(4); // Set precision for floating point numbers
    
    std::cout << "\nIMU Data:\n";
    std::cout << "Quaternion: (" 
              << imu.quaternion[0] << ", " 
              << imu.quaternion[1] << ", " 
              << imu.quaternion[2] << ", " 
              << imu.quaternion[3] << ")\n";
    
    std::cout << "Gyroscope: (" 
              << imu.gyroscope[0] << " rad/s, " 
              << imu.gyroscope[1] << " rad/s, " 
              << imu.gyroscope[2] << " rad/s)\n";
    
    std::cout << "Accelerometer: (" 
              << imu.accelerometer[0] << " m/s^2, " 
              << imu.accelerometer[1] << " m/s^2, " 
              << imu.accelerometer[2] << " m/s^2)\n";
    
    std::cout << "RPY (Euler angles): (" 
              << imu.rpy[0] << " rad, " 
              << imu.rpy[1] << " rad, " 
              << imu.rpy[2] << " rad)\n";
    
    std::cout << "Temperature: " << static_cast<int>(imu.temperature) << "\n";
}

void printMotorState(const std::array<MotorState, 20>& motorStates) {
    for (int i = 0; i < 12; i++) { // only first 12 have values
        std::cout << "Motor " << i << " State:\n";
        std::cout << "  Mode: " << static_cast<int>(motorStates[i].mode) << "\n";
        std::cout << "  Current Angle (q): " << motorStates[i].q << " rad\n";
        std::cout << "  Current Velocity (dq): " << motorStates[i].dq << " rad/s\n";
        std::cout << "  Current Acceleration (ddq): " << motorStates[i].ddq << " rad/s^2\n";
        std::cout << "  Estimated Output Torque (tauEst): " << motorStates[i].tauEst << " N.m\n";
        std::cout << "  Raw Current Angle (q_raw): " << motorStates[i].q_raw << " rad\n";
        std::cout << "  Raw Current Velocity (dq_raw): " << motorStates[i].dq_raw << " rad/s\n";
        std::cout << "  Raw Current Acceleration (ddq_raw): " << motorStates[i].ddq_raw << " rad/s^2\n";
        std::cout << "  Temperature: " << static_cast<int>(motorStates[i].temperature) << " °C\n";
        
        // std::cout << "  Reserve: [" << motorStates[i].reserve[0] << ", " << motorStates[i].reserve[1] << "]\n";
    }
}

void printBmsState(const BmsState& bms) {
    std::cout << "BmsState:\n";
    std::cout << "  Version: " << static_cast<int>(bms.version_h) << "."
              << static_cast<int>(bms.version_l) << "\n";
    std::cout << "  BMS Status: " << static_cast<int>(bms.bms_status) << " ("
              << (bms.bms_status == 1 ? "Normal" : "Abnormal") << ")\n";
    std::cout << "  State of Charge (SOC): " << static_cast<int>(bms.SOC) << "%\n";
    std::cout << "  Current: " << bms.current << " mA\n";
    std::cout << "  Cycle Count: " << bms.cycle << "\n";

    std::cout << "  BQ NTC Temperatures: [";
    for (size_t i = 0; i < bms.BQ_NTC.size(); ++i) {
        std::cout << static_cast<int>(bms.BQ_NTC[i]);
        if (i < bms.BQ_NTC.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "] °C\n";

    std::cout << "  MCU NTC Temperatures: [";
    for (size_t i = 0; i < bms.MCU_NTC.size(); ++i) {
        std::cout << static_cast<int>(bms.MCU_NTC[i]);
        if (i < bms.MCU_NTC.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "] °C\n";

    std::cout << "  Cell Voltages: [";
    for (size_t i = 0; i < bms.cell_vol.size(); ++i) {
        std::cout << bms.cell_vol[i] << " mV";
        if (i < bms.cell_vol.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]\n";
}

void printLowState(const LowState& lowState) {
    std::cout << "LowState:\n";
    
    std::cout << "  Head: [" << static_cast<int>(lowState.head[0]) << ", " 
              << static_cast<int>(lowState.head[1]) << "]\n";
    std::cout << "  Level Flag: " << static_cast<int>(lowState.levelFlag) << "\n";
    std::cout << "  Frame Reserve: " << static_cast<int>(lowState.frameReserve) << "\n";
    
    std::cout << "  SN: [" << lowState.SN[0] << ", " << lowState.SN[1] << "]\n";
    std::cout << "  Version: [" << lowState.version[0] << ", " << lowState.version[1] << "]\n";
    
    std::cout << "  Bandwidth: " << lowState.bandWidth << "\n";
    
    printIMU(lowState.imu);
    
    printMotorState(lowState.motorState);

    printBmsState(lowState.bms);
    
    std::cout << "  Foot Force: [" << lowState.footForce[0] << ", "
              << lowState.footForce[1] << ", " << lowState.footForce[2] << ", "
              << lowState.footForce[3] << "]\n";
              
    std::cout << "  Foot Force Estimate: [" << lowState.footForceEst[0] << ", "
              << lowState.footForceEst[1] << ", " << lowState.footForceEst[2] << ", "
              << lowState.footForceEst[3] << "]\n";
    
    std::cout << "  Tick: " << lowState.tick << " ms\n";
    
    std::cout << "  Wireless Remote: [";
    for (size_t i = 0; i < 40; ++i) {
        std::cout << static_cast<int>(lowState.wirelessRemote[i]);
        if (i < 39) {
            std::cout << ", ";
        }
    }
    std::cout << "]\n";
    
    std::cout << "  Reserve: " << lowState.reserve << "\n";
    std::cout << "  CRC: " << lowState.crc << "\n";
    
    std::cout << "=========================\n\n";
}


void printHighState(const HighState& highState) {
    std::cout << "HighState:\n";
    
    std::cout << "  Head: [" << static_cast<int>(highState.head[0]) << ", "
              << static_cast<int>(highState.head[1]) << "]\n";
    std::cout << "  Level Flag: " << static_cast<int>(highState.levelFlag) << "\n";
    std::cout << "  Frame Reserve: " << static_cast<int>(highState.frameReserve) << "\n";
    
    std::cout << "  SN: [" << highState.SN[0] << ", " << highState.SN[1] << "]\n";
    std::cout << "  Version: [" << highState.version[0] << ", " << highState.version[1] << "]\n";
    
    std::cout << "  Bandwidth: " << highState.bandWidth << "\n";
    
    printIMU(highState.imu);
    
    printMotorState(highState.motorState);

    printBmsState(highState.bms);
    
    std::cout << "  Foot Force: [" << highState.footForce[0] << ", "
              << highState.footForce[1] << ", " << highState.footForce[2] << ", "
              << highState.footForce[3] << "] N\n";
    
    std::cout << "  Foot Force Estimate: [" << highState.footForceEst[0] << ", "
              << highState.footForceEst[1] << ", " << highState.footForceEst[2] << ", "
              << highState.footForceEst[3] << "] N\n";

    std::cout << "  Mode: " << static_cast<int>(highState.mode) << "\n";
    std::cout << "  Progress: " << highState.progress << "\n";
    std::cout << "  Gait Type: " << static_cast<int>(highState.gaitType) << "\n";
    std::cout << "  Foot Raise Height: " << highState.footRaiseHeight << " m\n";
    
    std::cout << "  Position: [" << highState.position[0] << ", "
              << highState.position[1] << ", " << highState.position[2] << "] m\n";

    std::cout << "  Body Height: " << highState.bodyHeight << " m\n";
    
    std::cout << "  Velocity: [" << highState.velocity[0] << ", "
              << highState.velocity[1] << ", " << highState.velocity[2] << "] m/s\n";
    
    std::cout << "  Yaw Speed: " << highState.yawSpeed << " rad/s\n";
    
    std::cout << "  Range Obstacle: [" << highState.rangeObstacle[0] << ", " << highState.rangeObstacle[1]
              << highState.rangeObstacle[2] << ", " << highState.rangeObstacle[3] << "]\n";

    std::cout << "  Foot Position to Body:\n";
    for (const auto& footPos : highState.footPosition2Body) {
        std::cout << "    x: " << footPos.x << " y: "
                  << footPos.y << " z: " << footPos.z << "\n";
    }

    std::cout << "  Foot Speed to Body:\n";
    for (const auto& footSpeed : highState.footSpeed2Body) {
        std::cout << "    x: " << footSpeed.x << " y: "
            << footSpeed.y << " z: " << footSpeed.z << "\n";
    }

    std::cout << "  Wireless Remote: [";
    for (size_t i = 0; i < 40; ++i) {
        std::cout << static_cast<int>(highState.wirelessRemote[i]);
        if (i < 39) {
            std::cout << ", ";
        }
    }
    std::cout << "]\n";

    std::cout << "  Reserve: " << highState.reserve << "\n";
    std::cout << "  CRC: " << highState.crc << "\n";

    std::cout << "=========================\n\n";
}