
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "pretty_print.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <iomanip>
#include <boost/circular_buffer.hpp>
#include <fstream>

// #define IS_LOWLEVEL

#ifdef IS_LOWLEVEL
#define STATETYPE LowState
#define UDP_INIT udp(LOWLEVEL, 8090, "192.168.123.10", 8007)
#else
#define STATETYPE HighState
#define UDP_INIT udp(HIGHLEVEL, 8090, "192.168.123.220", 8082)
#endif


using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom() :
        accel_buffer(20),
        outFile("out.csv"),
        UDP_INIT
    {
        // udp.print = true;
    }
    void UDPUpdate();
    void RobotControl();

    UDP udp;
    STATETYPE state = {0};
    std::array<float, 3> pos = {0, 0, 0};
    std::array<float, 3> initial_state_pos = {0, 0, 0};
    bool setInitialStatePos = false;
    uint64_t loop_count = 0;
    float dt = 0.002; // 0.001~0.01
    boost::circular_buffer<int> accel_buffer;
    std::ofstream outFile;
};

void Custom::UDPUpdate()
{
    udp.Recv();
    udp.Send();
}


void Custom::RobotControl()
{
    loop_count++;
    udp.GetRecv(state);

    if (loop_count > 10 && !setInitialStatePos) {
        initial_state_pos[0] = state.position[0];
        initial_state_pos[1] = state.position[1];
        initial_state_pos[2] = state.position[2];
        setInitialStatePos = true;
    }

    pos[0] += state.imu.accelerometer[0] * dt * dt;
    pos[1] += state.imu.accelerometer[1] * dt * dt;
    pos[2] += (state.imu.accelerometer[2] - 9.81)* dt * dt;

    accel_buffer.push_back(state.imu.accelerometer[0]);

    // set floating point precision and + sign for positive
    std::cout << std::fixed << std::setprecision(3);
    std::cout << std::showpos;

    float sum = 0.0;
    for (uint8_t i = 0; i < accel_buffer.size(); i++) {
        sum += accel_buffer[i];
    }
    
    std::cout << "Accelerometer Acceleration: " << state.imu.accelerometer[0] << ", " << state.imu.accelerometer[1] << ", " << state.imu.accelerometer[2] << " \n";
    std::cout << "Accelerometer Accel avg:    " << (sum / accel_buffer.size()) << " \n";
    std::cout << "IMU Accumulated Position:   " << pos[0] << ", " << pos[1] << ", " << pos[2] << " \n";

    #ifdef IS_LOWLEVEL
    printLowState(state);
    #else
    std::cout << "High State Position:        " << (state.position[0] - initial_state_pos[0]) << ", " 
              << (state.position[1] - initial_state_pos[1]) << ", " << (state.position[2] - initial_state_pos[2]) << 
              "\nRaw: " << state.position[0] << ", " << state.position[1] << ", " << state.position[2] << 
              " \n ---------------------\n\n";

    // Writing to file
    if (outFile) {
        outFile << (state.position[0] - initial_state_pos[0]) << ", " << (state.position[1] - initial_state_pos[1]) << ", " << (state.position[2] - initial_state_pos[2]) << "\n";
        std::cout << "writing to file..." << std::endl;
    }

    // printHighState(state);
    #endif // IS_LOWLEVEL
}

int main(void)
{
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom = Custom();
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udp("udp_update", custom.dt, 3, boost::bind(&Custom::UDPUpdate, &custom));

    loop_udp.start();
    loop_control.start();

    while (1)
    {
        sleep(10);
    };

    return 0;
}
