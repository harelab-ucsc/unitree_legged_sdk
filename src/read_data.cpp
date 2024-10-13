
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <iomanip>
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


void Custom::RobotControl()
{
    udp.GetRecv(state);

    printIMU(state.imu);

    printf("\nMotor pos: ");
    for (uint8_t i = 0; i < 11; i++) {
        printf("%.2f, ", state.motorState[i].q);
    }
    printf("%.2f\n", state.motorState[11].q);
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
