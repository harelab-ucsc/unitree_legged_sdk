
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

    #ifdef IS_LOWLEVEL
    printLowState(state);
    #else
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
