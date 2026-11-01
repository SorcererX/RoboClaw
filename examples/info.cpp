#include <roboclaw.h>
#include <iostream>
#include <ostream>
#include <termios.h>

int main()
{
    RoboClaw test( "/dev/ttyACM0", 5, 0x80 );
    test.begin( B115200 );

    if( const auto version = test.readVersion() )
    {
        std::cout << "ReadVersion: " << *version << std::flush;
    }

    if( const auto pid = test.readM1VelocityPID() )
    {
        std::cout << "M1 Velocity PID: (" << pid->kp << "," << pid->ki << "," << pid->kd << ") QPPS:" << pid->qpps << '\n';
    }

    if( const auto pid = test.readM2VelocityPID() )
    {
        std::cout << "M2 Velocity PID: (" << pid->kp << "," << pid->ki << "," << pid->kd << ") QPPS:" << pid->qpps << '\n';
    }

    if( const auto voltage = test.readMainBatteryVoltage() )
    {
        std::cout << "Main Battery Voltage: " << *voltage / 10.0 << " V" << '\n';
    }

    if( const auto temperature = test.readTemp() )
    {
        std::cout << "Temperature: " << *temperature / 10.0 << " C" << '\n';
    }

    test.end();
}
