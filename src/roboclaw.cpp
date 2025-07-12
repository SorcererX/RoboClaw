#include "roboclaw.h"
#include <fstream>
#include <chrono>

#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <cstring>
#include <cstdio>

#include <iostream>

#define MAXRETRY 2
#define SetDWORDval( arg ) ( uint8_t ) ( ( ( uint32_t ) arg ) >> 24 ), ( uint8_t ) ( ( ( uint32_t ) arg ) >> 16 ), ( uint8_t ) ( ( ( uint32_t ) arg ) >> 8 ), ( uint8_t ) arg
#define SetWORDval( arg ) ( uint8_t ) ( ( ( uint16_t ) arg ) >> 8 ), ( uint8_t ) arg

RoboClaw::RoboClaw()
    : m_portName( "/dev/ttyS0" )
    , m_timeout( 5 )
    , m_address( 0x80 ) // 0x80-0x87 is used to identify a RoboClaw on a serial port, but on USB this is ignored.
{
    init();
}

RoboClaw::RoboClaw( const std::string& a_portName, uint8_t a_timeout, uint8_t a_address )
    : m_portName( a_portName )
    , m_timeout( a_timeout )
    , m_address( a_address ) // 0x80-0x87 is used to identify a RoboClaw on a serial port, but on USB this is ignored.
{
    init();
}

//
// Destructor
//
RoboClaw::~RoboClaw()
{
}

void RoboClaw::init()
{
    m_serial = -1;
}

// NOTE: speed is not a regular integer, but B115200 etc.
void RoboClaw::begin( long speed )
{
    if( m_serial == -1 )
    {
        m_serial = open( m_portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC );

        if( m_serial == -1 )
        {
            perror( "Unable to open serial device." );
            return;
        }

        struct termios tty;
        memset( &tty, 0, sizeof( tty ) );

        if( tcgetattr( m_serial, &tty ) != 0 )
        {
            perror( "error from tcgetattr" );
            return;
        }

        cfsetospeed( &tty, speed );
        cfsetispeed( &tty, speed );

        // input
        tty.c_iflag &= ~IGNBRK;              // disable ignoring break
        tty.c_iflag &= ~( IGNPAR | PARMRK ); // disable parity checks
        tty.c_iflag &= ~INPCK;               // disable parity checking
        tty.c_iflag &= ~ISTRIP;              // disable stripping 8th bit
        tty.c_iflag &= ~( INLCR | ICRNL );   // disable translating NL <-> CR
        tty.c_iflag &= ~IGNCR;               // disable ignoring CR
        tty.c_iflag &= ~( IXON | IXOFF );    // disable XON/XOFF flow control

        // output
        tty.c_oflag &= ~OPOST;             // disable output processing
        tty.c_oflag &= ~( ONLCR | OCRNL ); // disable translating NL <-> CR
        tty.c_oflag &= ~OFILL;             // disable fill characters

        // control flags
        tty.c_cflag |= CLOCAL;   // prevent changing ownership
        tty.c_cflag |= CREAD;    // enable reciever
        tty.c_cflag &= ~PARENB;  // disable parity
        tty.c_cflag &= ~CSTOPB;  // disable 2 stop bits
        tty.c_cflag &= ~CSIZE;   // remove size flag...
        tty.c_cflag |= CS8;      // ...enable 8 bit characters
        tty.c_cflag |= HUPCL;    // enable lower control lines on close - hang up
        tty.c_cflag &= ~CRTSCTS; // disable hardware CTS/RTS flow control

        // local flags
        tty.c_lflag &= ~ISIG;   // disable generating signals
        tty.c_lflag &= ~ICANON; // disable canonical mode - line by line
        tty.c_lflag &= ~ECHO;   // disable echoing characters
        tty.c_lflag &= ~ECHONL; // ??
        tty.c_lflag &= ~NOFLSH; // disable flushing on SIGINT
        tty.c_lflag &= ~IEXTEN; // disable input processing

        if( tcsetattr( m_serial, TCSANOW, &tty ) != 0 )
        {
            perror( "tcsetattr failed with error:" );
            return;
        }

        tcflush( m_serial, TCIOFLUSH );

        while( available() > 0 )
        {
            read( m_timeout );
        }

        fcntl( m_serial, F_SETFL, 0 );
    }
}

void RoboClaw::end()
{
    if( m_serial != -1 )
    {
        close( m_serial );
        m_serial = -1;
    }
}
/*
int RoboClaw::peek()
{
    int value;
    if( m_serialFile )
    {
        value = fgetc( m_serialFile );
        ungetc( value, m_serialFile );
    }
}
*/

size_t RoboClaw::write( uint8_t byte )
{
    if( m_serial )
    {
        return ::write( m_serial, &byte, 1 );
    }
    return 0;
}

uint8_t RoboClaw::read()
{
    uint8_t value = 0;
    if( m_serial )
    {
        ::read( m_serial, &value, 1 );
    }
    return value;
}

int RoboClaw::available()
{
    int available_bytes = 0;
    ioctl( m_serial, FIONREAD, &available_bytes );
    return available_bytes;
}

void RoboClaw::flush()
{
    if( m_serial )
    {
        tcflush( m_serial, TCIOFLUSH );
    }
}

uint8_t RoboClaw::read( uint32_t timeout )
{
    if( m_serial )
    {
        auto start = std::chrono::steady_clock::now();

        // Empty buffer?
        while( !available() )
        {
            auto end = std::chrono::steady_clock::now();
            auto elapsed = end - start;
            if( elapsed.count() * 1000.0 / std::chrono::steady_clock::period().den >= timeout )
            {
                return -1;
            }
        }
        return read();
    }
}

void RoboClaw::clear()
{
    // read everything on serial port.
    if( m_serial )
    {
        while( available() )
        {
            read();
        }
    }
}

void RoboClaw::crc_clear()
{
    m_crc = 0;
}

void RoboClaw::crc_update( uint8_t data )
{
    int i;
    m_crc = m_crc ^ ( ( uint16_t ) data << 8 );
    for( i = 0; i < 8; i++ )
    {
        if( m_crc & 0x8000 )
            m_crc = ( m_crc << 1 ) ^ 0x1021;
        else
            m_crc <<= 1;
    }
}

uint16_t RoboClaw::crc_get()
{
    return m_crc;
}

bool RoboClaw::write_n( uint8_t byte, ... )
{
    uint8_t trys = MAXRETRY;
    do
    {
        crc_clear();
        // send data with crc
        va_list marker;
        va_start( marker, byte ); /* Initialize variable arguments. */
        for( uint8_t index = 0; index < byte; index++ )
        {
            uint8_t data = va_arg( marker, int );
            crc_update( data );
            write( data );
        }
        va_end( marker ); /* Reset variable arguments.      */
        uint16_t crc = crc_get();
        write( crc >> 8 );
        write( crc );
        if( read( m_timeout ) == 0xFF )
            return true;
    } while( trys-- );
    return false;
}

bool RoboClaw::read_n( uint8_t cnt, uint8_t address, uint8_t cmd, ... )
{
    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        data = 0;
        crc_clear();
        write( m_address );
        crc_update( m_address );
        write( cmd );
        crc_update( cmd );

        // send data with crc
        va_list marker;
        va_start( marker, cmd ); /* Initialize variable arguments. */
        for( uint8_t index = 0; index < cnt; index++ )
        {
            uint32_t* ptr = va_arg( marker, uint32_t* );

            if( data != -1 )
            {
                data = read( m_timeout );
                crc_update( data );
                value = ( uint32_t ) data << 24;
            }
            else
            {
                break;
            }

            if( data != -1 )
            {
                data = read( m_timeout );
                crc_update( data );
                value |= ( uint32_t ) data << 16;
            }
            else
            {
                break;
            }

            if( data != -1 )
            {
                data = read( m_timeout );
                crc_update( data );
                value |= ( uint32_t ) data << 8;
            }
            else
            {
                break;
            }

            if( data != -1 )
            {
                data = read( m_timeout );
                crc_update( data );
                value |= ( uint32_t ) data;
            }
            else
            {
                break;
            }

            *ptr = value;
        }
        va_end( marker ); /* Reset variable arguments.      */

        if( data != -1 )
        {
            uint16_t ccrc;
            data = read( m_timeout );
            if( data != -1 )
            {
                ccrc = data << 8;
                data = read( m_timeout );
                if( data != -1 )
                {
                    ccrc |= data;
                    return crc_get() == ccrc;
                }
            }
        }
    } while( trys-- );

    return false;
}

uint8_t RoboClaw::Read1( uint8_t address, uint8_t cmd, bool* valid )
{
    uint8_t crc;

    if( valid )
        *valid = false;

    uint8_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write( m_address );
        crc_update( m_address );
        write( cmd );
        crc_update( cmd );

        data = read( m_timeout );
        crc_update( data );
        value = data;

        if( data != -1 )
        {
            uint16_t ccrc;
            data = read( m_timeout );
            if( data != -1 )
            {
                ccrc = data << 8;
                data = read( m_timeout );
                if( data != -1 )
                {
                    ccrc |= data;
                    if( crc_get() == ccrc )
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while( trys-- );

    return false;
}

uint16_t RoboClaw::Read2( uint8_t address, uint8_t cmd, bool* valid )
{
    uint8_t crc;

    if( valid )
        *valid = false;

    uint16_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write( address );
        crc_update( address );
        write( cmd );
        crc_update( cmd );

        data = read( m_timeout );
        crc_update( data );
        value = ( uint16_t ) data << 8;

        if( data != -1 )
        {
            data = read( m_timeout );
            crc_update( data );
            value |= ( uint16_t ) data;
        }

        if( data != -1 )
        {
            uint16_t ccrc;
            data = read( m_timeout );
            if( data != -1 )
            {
                ccrc = data << 8;
                data = read( m_timeout );
                if( data != -1 )
                {
                    ccrc |= data;
                    if( crc_get() == ccrc )
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while( trys-- );

    return false;
}

uint32_t RoboClaw::Read4( uint8_t address, uint8_t cmd, bool* valid )
{
    uint8_t crc;

    if( valid )
        *valid = false;

    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write( address );
        crc_update( address );
        write( cmd );
        crc_update( cmd );

        data = read( m_timeout );
        crc_update( data );
        value = ( uint32_t ) data << 24;

        if( data != -1 )
        {
            data = read( m_timeout );
            crc_update( data );
            value |= ( uint32_t ) data << 16;
        }

        if( data != -1 )
        {
            data = read( m_timeout );
            crc_update( data );
            value |= ( uint32_t ) data << 8;
        }

        if( data != -1 )
        {
            data = read( m_timeout );
            crc_update( data );
            value |= ( uint32_t ) data;
        }

        if( data != -1 )
        {
            uint16_t ccrc;
            data = read( m_timeout );
            if( data != -1 )
            {
                ccrc = data << 8;
                data = read( m_timeout );
                if( data != -1 )
                {
                    ccrc |= data;
                    if( crc_get() == ccrc )
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while( trys-- );

    return false;
}

uint32_t RoboClaw::Read4_1( uint8_t address, uint8_t cmd, uint8_t* status, bool* valid )
{
    uint8_t crc;

    if( valid )
        *valid = false;

    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write( address );
        crc_update( address );
        write( cmd );
        crc_update( cmd );

        data = read( m_timeout );
        crc_update( data );
        value = ( uint32_t ) data << 24;

        if( data != -1 )
        {
            data = read( m_timeout );
            crc_update( data );
            value |= ( uint32_t ) data << 16;
        }

        if( data != -1 )
        {
            data = read( m_timeout );
            crc_update( data );
            value |= ( uint32_t ) data << 8;
        }

        if( data != -1 )
        {
            data = read( m_timeout );
            crc_update( data );
            value |= ( uint32_t ) data;
        }

        if( data != -1 )
        {
            data = read( m_timeout );
            crc_update( data );
            if( status )
                *status = data;
        }

        if( data != -1 )
        {
            uint16_t ccrc;
            data = read( m_timeout );
            if( data != -1 )
            {
                ccrc = data << 8;
                data = read( m_timeout );
                if( data != -1 )
                {
                    ccrc |= data;
                    if( crc_get() == ccrc )
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while( trys-- );

    return false;
}

bool RoboClaw::ForwardM1( uint8_t speed )
{
    return write_n( 3, m_address, M1FORWARD, speed );
}

bool RoboClaw::BackwardM1( uint8_t speed )
{
    return write_n( 3, m_address, M1BACKWARD, speed );
}

bool RoboClaw::SetMinVoltageMainBattery( uint8_t voltage )
{
    return write_n( 3, m_address, SETMINMB, voltage );
}

bool RoboClaw::SetMaxVoltageMainBattery( uint8_t voltage )
{
    return write_n( 3, m_address, SETMAXMB, voltage );
}

bool RoboClaw::ForwardM2( uint8_t speed )
{
    return write_n( 3, m_address, M2FORWARD, speed );
}

bool RoboClaw::BackwardM2( uint8_t speed )
{
    return write_n( 3, m_address, M2BACKWARD, speed );
}

bool RoboClaw::ForwardBackwardM1( uint8_t speed )
{
    return write_n( 3, m_address, M17BIT, speed );
}

bool RoboClaw::ForwardBackwardM2( uint8_t speed )
{
    return write_n( 3, m_address, M27BIT, speed );
}

bool RoboClaw::ForwardMixed( uint8_t speed )
{
    return write_n( 3, m_address, MIXEDFORWARD, speed );
}

bool RoboClaw::BackwardMixed( uint8_t speed )
{
    return write_n( 3, m_address, MIXEDBACKWARD, speed );
}

bool RoboClaw::TurnRightMixed( uint8_t speed )
{
    return write_n( 3, m_address, MIXEDRIGHT, speed );
}

bool RoboClaw::TurnLeftMixed( uint8_t speed )
{
    return write_n( 3, m_address, MIXEDLEFT, speed );
}

bool RoboClaw::ForwardBackwardMixed( uint8_t speed )
{
    return write_n( 3, m_address, MIXEDFB, speed );
}

bool RoboClaw::LeftRightMixed( uint8_t speed )
{
    return write_n( 3, m_address, MIXEDLR, speed );
}

uint32_t RoboClaw::ReadEncM1( uint8_t* status, bool* valid )
{
    return Read4_1( m_address, GETM1ENC, status, valid );
}

uint32_t RoboClaw::ReadEncM2( uint8_t* status, bool* valid )
{
    return Read4_1( m_address, GETM2ENC, status, valid );
}

uint32_t RoboClaw::ReadSpeedM1( uint8_t* status, bool* valid )
{
    return Read4_1( m_address, GETM1SPEED, status, valid );
}

uint32_t RoboClaw::ReadSpeedM2( uint8_t* status, bool* valid )
{
    return Read4_1( m_address, GETM2SPEED, status, valid );
}

bool RoboClaw::ResetEncoders( uint8_t address )
{
    return write_n( 2, m_address, RESETENC );
}

bool RoboClaw::ReadVersion( char* version )
{
    uint8_t data;
    uint8_t trys = MAXRETRY;
    do
    {
        flush();

        data = 0;

        crc_clear();
        write( m_address );
        crc_update( m_address );
        write( GETVERSION );
        crc_update( GETVERSION );
        uint8_t i;
        for( i = 0; i < 48; i++ )
        {
            if( data != -1 )
            {
                data = read( m_timeout );
                version[ i ] = data;
                crc_update( version[ i ] );
                if( version[ i ] == 0 )
                {
                    uint16_t ccrc;
                    data = read( m_timeout );
                    if( data != -1 )
                    {
                        ccrc = data << 8;
                        data = read( m_timeout );
                        if( data != -1 )
                        {
                            ccrc |= data;
                            return crc_get() == ccrc;
                        }
                    }
                    break;
                }
            }
            else
            {
                break;
            }
        }
    } while( trys-- );

    return false;
}

bool RoboClaw::SetEncM1( int32_t val )
{
    return write_n( 6, m_address, SETM1ENCCOUNT, SetDWORDval( val ) );
}

bool RoboClaw::SetEncM2( int32_t val )
{
    return write_n( 6, m_address, SETM2ENCCOUNT, SetDWORDval( val ) );
}

uint16_t RoboClaw::ReadMainBatteryVoltage( bool* valid )
{
    return Read2( m_address, GETMBATT, valid );
}

uint16_t RoboClaw::ReadLogicBatteryVoltage( bool* valid )
{
    return Read2( m_address, GETLBATT, valid );
}

bool RoboClaw::SetMinVoltageLogicBattery( uint8_t voltage )
{
    return write_n( 3, m_address, SETMINLB, voltage );
}

bool RoboClaw::SetMaxVoltageLogicBattery( uint8_t voltage )
{
    return write_n( 3, m_address, SETMAXLB, voltage );
}

bool RoboClaw::SetM1VelocityPID( float kp_fp, float ki_fp, float kd_fp, uint32_t qpps )
{
    uint32_t kp = kp_fp * 65536;
    uint32_t ki = ki_fp * 65536;
    uint32_t kd = kd_fp * 65536;
    return write_n( 18, m_address, SETM1PID, SetDWORDval( kd ), SetDWORDval( kp ), SetDWORDval( ki ), SetDWORDval( qpps ) );
}

bool RoboClaw::SetM2VelocityPID( float kp_fp, float ki_fp, float kd_fp, uint32_t qpps )
{
    uint32_t kp = kp_fp * 65536;
    uint32_t ki = ki_fp * 65536;
    uint32_t kd = kd_fp * 65536;
    return write_n( 18, m_address, SETM2PID, SetDWORDval( kd ), SetDWORDval( kp ), SetDWORDval( ki ), SetDWORDval( qpps ) );
}

uint32_t RoboClaw::ReadISpeedM1( uint8_t* status, bool* valid )
{
    return Read4_1( m_address, GETM1ISPEED, status, valid );
}

uint32_t RoboClaw::ReadISpeedM2( uint8_t* status, bool* valid )
{
    return Read4_1( m_address, GETM2ISPEED, status, valid );
}

bool RoboClaw::DutyM1( uint16_t duty )
{
    return write_n( 4, m_address, M1DUTY, SetWORDval( duty ) );
}

bool RoboClaw::DutyM2( uint16_t duty )
{
    return write_n( 4, m_address, M2DUTY, SetWORDval( duty ) );
}

bool RoboClaw::DutyM1M2( uint16_t duty1, uint16_t duty2 )
{
    return write_n( 6, m_address, MIXEDDUTY, SetWORDval( duty1 ), SetWORDval( duty2 ) );
}

bool RoboClaw::SpeedM1( uint32_t speed )
{
    return write_n( 6, m_address, M1SPEED, SetDWORDval( speed ) );
}

bool RoboClaw::SpeedM2( uint32_t speed )
{
    return write_n( 6, m_address, M2SPEED, SetDWORDval( speed ) );
}

bool RoboClaw::SpeedM1M2( uint32_t speed1, uint32_t speed2 )
{
    return write_n( 10, m_address, MIXEDSPEED, SetDWORDval( speed1 ), SetDWORDval( speed2 ) );
}

bool RoboClaw::SpeedAccelM1( uint32_t accel, uint32_t speed )
{
    return write_n( 10, m_address, M1SPEEDACCEL, SetDWORDval( accel ), SetDWORDval( speed ) );
}

bool RoboClaw::SpeedAccelM2( uint32_t accel, uint32_t speed )
{
    return write_n( 10, m_address, M2SPEEDACCEL, SetDWORDval( accel ), SetDWORDval( speed ) );
}
bool RoboClaw::SpeedAccelM1M2( uint32_t accel, uint32_t speed1, uint32_t speed2 )
{
    return write_n( 14, m_address, MIXEDSPEEDACCEL, SetDWORDval( accel ), SetDWORDval( speed1 ), SetDWORDval( speed2 ) );
}

bool RoboClaw::SpeedDistanceM1( uint32_t speed, uint32_t distance, uint8_t flag )
{
    return write_n( 11, m_address, M1SPEEDDIST, SetDWORDval( speed ), SetDWORDval( distance ), flag );
}

bool RoboClaw::SpeedDistanceM2( uint32_t speed, uint32_t distance, uint8_t flag )
{
    return write_n( 11, m_address, M2SPEEDDIST, SetDWORDval( speed ), SetDWORDval( distance ), flag );
}

bool RoboClaw::SpeedDistanceM1M2( uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag )
{
    return write_n( 19, m_address, MIXEDSPEEDDIST, SetDWORDval( speed1 ), SetDWORDval( distance1 ), SetDWORDval( speed2 ), SetDWORDval( distance2 ), flag );
}

bool RoboClaw::SpeedAccelDistanceM1( uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag )
{
    return write_n( 15, m_address, M1SPEEDACCELDIST, SetDWORDval( accel ), SetDWORDval( speed ), SetDWORDval( distance ), flag );
}

bool RoboClaw::SpeedAccelDistanceM2( uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag )
{
    return write_n( 15, m_address, M2SPEEDACCELDIST, SetDWORDval( accel ), SetDWORDval( speed ), SetDWORDval( distance ), flag );
}

bool RoboClaw::SpeedAccelDistanceM1M2( uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag )
{
    return write_n( 23, m_address, MIXEDSPEEDACCELDIST, SetDWORDval( accel ), SetDWORDval( speed1 ), SetDWORDval( distance1 ), SetDWORDval( speed2 ), SetDWORDval( distance2 ), flag );
}

bool RoboClaw::ReadBuffers( uint8_t& depth1, uint8_t& depth2 )
{
    bool valid;
    uint16_t value = Read2( m_address, GETBUFFERS, &valid );
    if( valid )
    {
        depth1 = value >> 8;
        depth2 = value;
    }
    return valid;
}

bool RoboClaw::ReadPWMs( int16_t& pwm1, int16_t& pwm2 )
{
    bool valid;
    uint32_t value = Read4( m_address, GETPWMS, &valid );
    if( valid )
    {
        pwm1 = value >> 16;
        pwm2 = value & 0xFFFF;
    }
    return valid;
}

bool RoboClaw::ReadCurrents( int16_t& current1, int16_t& current2 )
{
    bool valid;
    uint32_t value = Read4( m_address, GETCURRENTS, &valid );
    if( valid )
    {
        current1 = value >> 16;
        current2 = value & 0xFFFF;
    }
    return valid;
}

bool RoboClaw::SpeedAccelM1M2_2( uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2 )
{
    return write_n( 18, m_address, MIXEDSPEED2ACCEL, SetDWORDval( accel1 ), SetDWORDval( speed1 ), SetDWORDval( accel2 ), SetDWORDval( speed2 ) );
}

bool RoboClaw::SpeedAccelDistanceM1M2_2( uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag )
{
    return write_n( 27, m_address, MIXEDSPEED2ACCELDIST, SetDWORDval( accel1 ), SetDWORDval( speed1 ), SetDWORDval( distance1 ), SetDWORDval( accel2 ), SetDWORDval( speed2 ), SetDWORDval( distance2 ), flag );
}

bool RoboClaw::DutyAccelM1( uint16_t duty, uint32_t accel )
{
    return write_n( 8, m_address, M1DUTYACCEL, SetWORDval( duty ), SetDWORDval( accel ) );
}

bool RoboClaw::DutyAccelM2( uint16_t duty, uint32_t accel )
{
    return write_n( 8, m_address, M2DUTYACCEL, SetWORDval( duty ), SetDWORDval( accel ) );
}

bool RoboClaw::DutyAccelM1M2( uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2 )
{
    return write_n( 14, m_address, MIXEDDUTYACCEL, SetWORDval( duty1 ), SetDWORDval( accel1 ), SetWORDval( duty2 ), SetDWORDval( accel2 ) );
}

bool RoboClaw::ReadM1VelocityPID( float& Kp_fp, float& Ki_fp, float& Kd_fp, uint32_t& qpps )
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n( 4, m_address, READM1PID, &Kp, &Ki, &Kd, &qpps );
    Kp_fp = ( ( float ) Kp ) / 65536;
    Ki_fp = ( ( float ) Ki ) / 65536;
    Kd_fp = ( ( float ) Kd ) / 65536;
    return valid;
}

bool RoboClaw::ReadM2VelocityPID( float& Kp_fp, float& Ki_fp, float& Kd_fp, uint32_t& qpps )
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n( 4, m_address, READM2PID, &Kp, &Ki, &Kd, &qpps );
    Kp_fp = ( ( float ) Kp ) / 65536;
    Ki_fp = ( ( float ) Ki ) / 65536;
    Kd_fp = ( ( float ) Kd ) / 65536;
    return valid;
}

bool RoboClaw::SetMainVoltages( uint16_t min, uint16_t max )
{
    return write_n( 6, m_address, SETMAINVOLTAGES, SetWORDval( min ), SetWORDval( max ) );
}

bool RoboClaw::SetLogicVoltages( uint16_t min, uint16_t max )
{
    return write_n( 6, m_address, SETLOGICVOLTAGES, SetWORDval( min ), SetWORDval( max ) );
}

bool RoboClaw::ReadMinMaxMainVoltages( uint16_t& min, uint16_t& max )
{
    bool valid;
    uint32_t value = Read4( m_address, GETMINMAXMAINVOLTAGES, &valid );
    if( valid )
    {
        min = value >> 16;
        max = value & 0xFFFF;
    }
    return valid;
}

bool RoboClaw::ReadMinMaxLogicVoltages( uint16_t& min, uint16_t& max )
{
    bool valid;
    uint32_t value = Read4( m_address, GETMINMAXLOGICVOLTAGES, &valid );
    if( valid )
    {
        min = value >> 16;
        max = value & 0xFFFF;
    }
    return valid;
}

bool RoboClaw::SetM1PositionPID( float kp_fp, float ki_fp, float kd_fp, uint32_t kiMax, uint32_t deadzone, uint32_t min, uint32_t max )
{
    uint32_t kp = kp_fp * 1024;
    uint32_t ki = ki_fp * 1024;
    uint32_t kd = kd_fp * 1024;
    return write_n( 30, m_address, SETM1POSPID, SetDWORDval( kd ), SetDWORDval( kp ), SetDWORDval( ki ), SetDWORDval( kiMax ), SetDWORDval( deadzone ), SetDWORDval( min ), SetDWORDval( max ) );
}

bool RoboClaw::SetM2PositionPID( float kp_fp, float ki_fp, float kd_fp, uint32_t kiMax, uint32_t deadzone, uint32_t min, uint32_t max )
{
    uint32_t kp = kp_fp * 1024;
    uint32_t ki = ki_fp * 1024;
    uint32_t kd = kd_fp * 1024;
    return write_n( 30, m_address, SETM2POSPID, SetDWORDval( kd ), SetDWORDval( kp ), SetDWORDval( ki ), SetDWORDval( kiMax ), SetDWORDval( deadzone ), SetDWORDval( min ), SetDWORDval( max ) );
}

bool RoboClaw::ReadM1PositionPID( float& Kp_fp, float& Ki_fp, float& Kd_fp, uint32_t& KiMax, uint32_t& DeadZone, uint32_t& Min, uint32_t& Max )
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n( 7, m_address, READM1POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max );
    Kp_fp = ( ( float ) Kp ) / 1024;
    Ki_fp = ( ( float ) Ki ) / 1024;
    Kd_fp = ( ( float ) Kd ) / 1024;
    return valid;
}

bool RoboClaw::ReadM2PositionPID( float& Kp_fp, float& Ki_fp, float& Kd_fp, uint32_t& KiMax, uint32_t& DeadZone, uint32_t& Min, uint32_t& Max )
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n( 7, m_address, READM2POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max );
    Kp_fp = ( ( float ) Kp ) / 1024;
    Ki_fp = ( ( float ) Ki ) / 1024;
    Kd_fp = ( ( float ) Kd ) / 1024;
    return valid;
}

bool RoboClaw::SpeedAccelDeccelPositionM1( uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag )
{
    return write_n( 19, m_address, M1SPEEDACCELDECCELPOS, SetDWORDval( accel ), SetDWORDval( speed ), SetDWORDval( deccel ), SetDWORDval( position ), flag );
}

bool RoboClaw::SpeedAccelDeccelPositionM2( uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag )
{
    return write_n( 19, m_address, M2SPEEDACCELDECCELPOS, SetDWORDval( accel ), SetDWORDval( speed ), SetDWORDval( deccel ), SetDWORDval( position ), flag );
}

bool RoboClaw::SpeedAccelDeccelPositionM1M2( uint32_t accel1, uint32_t speed1, uint32_t deccel1, uint32_t position1, uint32_t accel2, uint32_t speed2, uint32_t deccel2, uint32_t position2, uint8_t flag )
{
    return write_n( 35, m_address, MIXEDSPEEDACCELDECCELPOS, SetDWORDval( accel1 ), SetDWORDval( speed1 ), SetDWORDval( deccel1 ), SetDWORDval( position1 ), SetDWORDval( accel2 ), SetDWORDval( speed2 ), SetDWORDval( deccel2 ), SetDWORDval( position2 ), flag );
}

bool RoboClaw::SetM1DefaultAccel( uint32_t accel )
{
    return write_n( 6, m_address, SETM1DEFAULTACCEL, SetDWORDval( accel ) );
}

bool RoboClaw::SetM2DefaultAccel( uint32_t accel )
{
    return write_n( 6, m_address, SETM2DEFAULTACCEL, SetDWORDval( accel ) );
}

bool RoboClaw::SetPinFunctions( uint8_t S3mode, uint8_t S4mode, uint8_t S5mode )
{
    return write_n( 5, m_address, SETPINFUNCTIONS, S3mode, S4mode, S5mode );
}

bool RoboClaw::GetPinFunctions( uint8_t& S3mode, uint8_t& S4mode, uint8_t& S5mode )
{
    uint8_t crc;
    bool valid = false;
    uint8_t val1, val2, val3;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write( m_address );
        crc_update( m_address );
        write( GETPINFUNCTIONS );
        crc_update( GETPINFUNCTIONS );

        data = read( m_timeout );
        crc_update( data );
        val1 = data;

        if( data != -1 )
        {
            data = read( m_timeout );
            crc_update( data );
            val2 = data;
        }

        if( data != -1 )
        {
            data = read( m_timeout );
            crc_update( data );
            val3 = data;
        }

        if( data != -1 )
        {
            uint16_t ccrc;
            data = read( m_timeout );
            if( data != -1 )
            {
                ccrc = data << 8;
                data = read( m_timeout );
                if( data != -1 )
                {
                    ccrc |= data;
                    if( crc_get() == ccrc )
                    {
                        S3mode = val1;
                        S4mode = val2;
                        S5mode = val3;
                        return true;
                    }
                }
            }
        }
    } while( trys-- );

    return false;
}

bool RoboClaw::SetDeadBand( uint8_t Min, uint8_t Max )
{
    return write_n( 4, m_address, SETDEADBAND, Min, Max );
}

bool RoboClaw::GetDeadBand( uint8_t& Min, uint8_t& Max )
{
    bool valid;
    uint16_t value = Read2( m_address, GETDEADBAND, &valid );
    if( valid )
    {
        Min = value >> 8;
        Max = value;
    }
    return valid;
}

bool RoboClaw::ReadEncoders( uint32_t& enc1, uint32_t& enc2 )
{
    bool valid = read_n( 2, m_address, GETENCODERS, &enc1, &enc2 );
    return valid;
}

bool RoboClaw::ReadISpeeds( uint32_t& ispeed1, uint32_t& ispeed2 )
{
    bool valid = read_n( 2, m_address, GETISPEEDS, &ispeed1, &ispeed2 );
    return valid;
}

bool RoboClaw::RestoreDefaults( uint8_t address )
{
    return write_n( 2, m_address, RESTOREDEFAULTS );
}

bool RoboClaw::ReadTemp( uint16_t& temp )
{
    bool valid;
    temp = Read2( m_address, GETTEMP, &valid );
    return valid;
}

bool RoboClaw::ReadTemp2( uint16_t& temp )
{
    bool valid;
    temp = Read2( m_address, GETTEMP2, &valid );
    return valid;
}

uint16_t RoboClaw::ReadError( bool* valid )
{
    return Read2( m_address, GETERROR, valid );
}

bool RoboClaw::ReadEncoderModes( uint8_t& M1mode, uint8_t& M2mode )
{
    bool valid;
    uint16_t value = Read2( m_address, GETENCODERMODE, &valid );
    if( valid )
    {
        M1mode = value >> 8;
        M2mode = value;
    }
    return valid;
}

bool RoboClaw::SetM1EncoderMode( uint8_t mode )
{
    return write_n( 3, m_address, SETM1ENCODERMODE, mode );
}

bool RoboClaw::SetM2EncoderMode( uint8_t mode )
{
    return write_n( 3, m_address, SETM2ENCODERMODE, mode );
}

bool RoboClaw::WriteNVM( uint8_t address )
{
    return write_n( 6, m_address, WRITENVM, SetDWORDval( 0xE22EAB7A ) );
}

bool RoboClaw::ReadNVM( uint8_t address )
{
    return write_n( 2, m_address, READNVM );
}

bool RoboClaw::SetConfig( uint16_t config )
{
    return write_n( 4, m_address, SETCONFIG, SetWORDval( config ) );
}

bool RoboClaw::GetConfig( uint16_t& config )
{
    bool valid;
    uint16_t value = Read2( m_address, GETCONFIG, &valid );
    if( valid )
    {
        config = value;
    }
    return valid;
}

bool RoboClaw::SetM1MaxCurrent( uint32_t max )
{
    return write_n( 10, m_address, SETM1MAXCURRENT, SetDWORDval( max ), SetDWORDval( 0 ) );
}

bool RoboClaw::SetM2MaxCurrent( uint32_t max )
{
    return write_n( 10, m_address, SETM2MAXCURRENT, SetDWORDval( max ), SetDWORDval( 0 ) );
}

bool RoboClaw::ReadM1MaxCurrent( uint32_t& max )
{
    uint32_t tmax, dummy;
    bool valid = read_n( 2, m_address, GETM1MAXCURRENT, &tmax, &dummy );
    if( valid )
        max = tmax;
    return valid;
}

bool RoboClaw::ReadM2MaxCurrent( uint32_t& max )
{
    uint32_t tmax, dummy;
    bool valid = read_n( 2, m_address, GETM2MAXCURRENT, &tmax, &dummy );
    if( valid )
        max = tmax;
    return valid;
}

bool RoboClaw::SetPWMMode( uint8_t mode )
{
    return write_n( 3, m_address, SETPWMMODE, mode );
}

bool RoboClaw::GetPWMMode( uint8_t& mode )
{
    bool valid;
    uint8_t value = Read1( m_address, GETPWMMODE, &valid );
    if( valid )
    {
        mode = value;
    }
    return valid;
}
