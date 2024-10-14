
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "pretty_print.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

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
    float dt = 0.002; // 0.001~0.01
};

void Custom::UDPUpdate()
{
    udp.Recv();
    udp.Send();
}


void Custom::RobotControl()
{
    udp.GetRecv(state);

    if (!setInitialStatePos) {
        initial_state_pos[0] = state.position[0];
        initial_state_pos[1] = state.position[1];
        initial_state_pos[2] = state.position[2];
        setInitialStatePos = true;
    }

    pos[0] += state.imu.accelerometer[0] * dt * dt;
    pos[1] += state.imu.accelerometer[1] * dt * dt;
    pos[2] += state.imu.accelerometer[2] * dt * dt;

    std::cout << "IMU Accumulated Position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << " \n";

    #ifdef IS_LOWLEVEL
    printLowState(state);
    #else
    std::cout << "High State Position:      " << (state.position[0] - initial_state_pos[0]) << ", " 
              << (state.position[1] - initial_state_pos[1]) << ", " << (state.position[2] - initial_state_pos[2]) << " \n ---------------------\n\n";
    printHighState(state);
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
