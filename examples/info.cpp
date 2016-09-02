#include <roboclaw.h>
#include <iostream>

int main()
{
    RoboClaw test( "/dev/ttyS0", 5, 0x80 );
    char version[32];
    test.begin( B1152000 );
    test.ReadVersion( version );
    std::cout << version << std::endl;
    test.end();
}
