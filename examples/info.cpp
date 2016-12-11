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
    std::cout << "M1 Velocity PID: (" << Kp << "," << Ki << "," << Kd << ") QPPS:" << qpps << std::endl;

    test.ReadM2VelocityPID( Kp, Ki, Kd, qpps );
    std::cout << "M2 Velocity PID: (" << Kp << "," << Ki << "," << Kd << ") QPPS:" << qpps << std::endl;

    bool valid;
    uint16_t voltage = test.ReadMainBatteryVoltage( &valid );
    std::cout << "Main Battery Voltage: " << voltage/10.0 << " V" << std::endl;

    uint16_t temperature;
    valid = test.ReadTemp( temperature );
    std::cout << "Temperature: " << temperature/10.0 << " C" << std::endl;

    test.end();
}
