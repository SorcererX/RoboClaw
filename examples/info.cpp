#include <roboclaw.h>
#include <iostream>

int main()
{
    RoboClaw test( "/dev/ttyACM0", 5, 0x80 );
    char version[512];
    test.begin( B1152000 );
    test.ReadVersion( version );
    std::cout << "ReadVersion: " << version << std::flush;
    float Kp, Ki, Kd;
    uint32_t qpps;
    test.ReadM1VelocityPID( Kp, Ki, Kd, qpps );
    std::cout << "M1 Velocity PID: (" << Kp << "," << Ki << "," << Kd << ") qpps:" << qpps << std::endl;
    test.ReadM2VelocityPID( Kp, Ki, Kd, qpps );
    std::cout << "M2 Velocity PID: (" << Kp << "," << Ki << "," << Kd << ") qpps:" << qpps << std::endl;
    test.end();
}
