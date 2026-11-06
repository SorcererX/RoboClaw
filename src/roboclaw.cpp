#include "roboclaw.h"
#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
#include <string>
#include <utility>

#include <asm-generic/ioctls.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace
{
    constexpr int maxRetry = 2;
    constexpr float velocityPidScale = 65536.0F;
    constexpr float positionPidScale = 1024.0F;

    // Each value is sent big-endian with the width of its type.
    template< std::unsigned_integral... Values >
    auto payload( Values... a_values )
    {
        std::array< uint8_t, ( sizeof( Values ) + ... ) > bytes{};
        size_t next = 0;
        const auto put = [ & ]< typename Value >( Value a_value )
        {
            for( size_t remaining = sizeof( Value ); remaining > 0; remaining-- )
            {
                bytes[ next++ ] = static_cast< uint8_t >( a_value >> ( ( remaining - 1 ) * 8 ) );
            }
        };
        ( put( a_values ), ... );
        return bytes;
    }

    uint32_t getDword( std::span< const uint8_t > a_data, size_t a_offset = 0 )
    {
        return ( static_cast< uint32_t >( a_data[ a_offset ] ) << 24 ) | ( static_cast< uint32_t >( a_data[ a_offset + 1 ] ) << 16 ) | ( static_cast< uint32_t >( a_data[ a_offset + 2 ] ) << 8 ) | a_data[ a_offset + 3 ];
    }

    uint16_t getWord( std::span< const uint8_t > a_data, size_t a_offset = 0 )
    {
        return static_cast< uint16_t >( ( a_data[ a_offset ] << 8 ) | a_data[ a_offset + 1 ] );
    }

    uint16_t toWord( int16_t a_high, int16_t a_low )
    {
        return static_cast< uint16_t >( ( a_high << 8 ) | a_low );
    }

    uint32_t toFixed( float a_value, float a_scale )
    {
        return static_cast< uint32_t >( a_value * a_scale );
    }

    float fromFixed( uint32_t a_value, float a_scale )
    {
        return static_cast< float >( a_value ) / a_scale;
    }
}

// 0x80-0x87 is used to identify a RoboClaw on a serial port, but on USB this is ignored.
RoboClaw::RoboClaw()
    : m_portName( "/dev/ttyACM0" )
    , m_timeout( 5 )
    , m_address( 0x80 )
{
}

RoboClaw::RoboClaw( std::string a_portName, uint8_t a_timeout, uint8_t a_address )
    : m_portName( std::move( a_portName ) )
    , m_timeout( a_timeout )
    , m_address( a_address )
{
}

void RoboClaw::begin( speed_t a_speed )
{
    if( m_serial == -1 )
    {
        m_serial = open( m_portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC ); // NOLINT(cppcoreguidelines-pro-type-vararg): C API

        if( m_serial == -1 )
        {
            perror( "Unable to open serial device." );
            return;
        }

        // Two processes on one port take each other's replies, so only one may have it.
        if( flock( m_serial, LOCK_EX | LOCK_NB ) != 0 )
        {
            perror( "Serial device is already in use." );
            close( m_serial );
            m_serial = -1;
            return;
        }

        struct termios tty{};

        if( tcgetattr( m_serial, &tty ) != 0 )
        {
            perror( "error from tcgetattr" );
            return;
        }

        cfsetospeed( &tty, a_speed );
        cfsetispeed( &tty, a_speed );

        // input
        tty.c_iflag &= ~static_cast< tcflag_t >( IGNBRK );          // disable ignoring break
        tty.c_iflag &= ~static_cast< tcflag_t >( IGNPAR | PARMRK ); // disable parity checks
        tty.c_iflag &= ~static_cast< tcflag_t >( INPCK );           // disable parity checking
        tty.c_iflag &= ~static_cast< tcflag_t >( ISTRIP );          // disable stripping 8th bit
        tty.c_iflag &= ~static_cast< tcflag_t >( INLCR | ICRNL );   // disable translating NL <-> CR
        tty.c_iflag &= ~static_cast< tcflag_t >( IGNCR );           // disable ignoring CR
        tty.c_iflag &= ~static_cast< tcflag_t >( IXON | IXOFF );    // disable XON/XOFF flow control

        // output
        tty.c_oflag &= ~static_cast< tcflag_t >( OPOST );         // disable output processing
        tty.c_oflag &= ~static_cast< tcflag_t >( ONLCR | OCRNL ); // disable translating NL <-> CR
        tty.c_oflag &= ~static_cast< tcflag_t >( OFILL );         // disable fill characters

        // control flags
        tty.c_cflag |= CLOCAL;                              // prevent changing ownership
        tty.c_cflag |= CREAD;                               // enable reciever
        tty.c_cflag &= ~static_cast< tcflag_t >( PARENB );  // disable parity
        tty.c_cflag &= ~static_cast< tcflag_t >( CSTOPB );  // disable 2 stop bits
        tty.c_cflag &= ~static_cast< tcflag_t >( CSIZE );   // remove size flag...
        tty.c_cflag |= CS8;                                 // ...enable 8 bit characters
        tty.c_cflag |= HUPCL;                               // enable lower control lines on close - hang up
        tty.c_cflag &= ~static_cast< tcflag_t >( CRTSCTS ); // disable hardware CTS/RTS flow control

        // local flags
        tty.c_lflag &= ~static_cast< tcflag_t >( ISIG );   // disable generating signals
        tty.c_lflag &= ~static_cast< tcflag_t >( ICANON ); // disable canonical mode - line by line
        tty.c_lflag &= ~static_cast< tcflag_t >( ECHO );   // disable echoing characters
        tty.c_lflag &= ~static_cast< tcflag_t >( ECHONL ); // ??
        tty.c_lflag &= ~static_cast< tcflag_t >( NOFLSH ); // disable flushing on SIGINT
        tty.c_lflag &= ~static_cast< tcflag_t >( IEXTEN ); // disable input processing

        if( tcsetattr( m_serial, TCSANOW, &tty ) != 0 )
        {
            perror( "tcsetattr failed with error:" );
            return;
        }

        tcflush( m_serial, TCIOFLUSH );

        while( available() > 0 )
        {
            static_cast< void >( read( m_timeout ) );
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

size_t RoboClaw::write( uint8_t a_byte )
{
    if( m_serial != -1 )
    {
        return ::write( m_serial, &a_byte, 1 );
    }
    return 0;
}

uint8_t RoboClaw::read() const
{
    uint8_t value = 0;
    if( m_serial != -1 )
    {
        if( ::read( m_serial, &value, 1 ) != 1 )
        {
            return 0;
        }
    }
    return value;
}

int RoboClaw::available() const
{
    int available_bytes = 0;
    ioctl( m_serial, FIONREAD, &available_bytes ); // NOLINT(cppcoreguidelines-pro-type-vararg): C API
    return available_bytes;
}

void RoboClaw::flush()
{
    if( m_serial != -1 )
    {
        tcflush( m_serial, TCIOFLUSH );
    }
}

int16_t RoboClaw::read( uint32_t a_timeout ) const
{
    if( m_serial != -1 )
    {
        const auto start = std::chrono::steady_clock::now();

        // Empty buffer?
        while( available() == 0 )
        {
            if( std::chrono::steady_clock::now() - start >= std::chrono::milliseconds( a_timeout ) )
            {
                return -1;
            }
        }
        return read();
    }
    return -1;
}

void RoboClaw::clear() const
{
    // read everything on serial port.
    if( m_serial != -1 )
    {
        while( available() != 0 )
        {
            static_cast< void >( read() );
        }
    }
}

void RoboClaw::crcClear()
{
    m_crc = 0;
}

void RoboClaw::crcUpdate( uint8_t a_data )
{
    m_crc = static_cast< uint16_t >( m_crc ^ ( a_data << 8 ) );
    for( int bit = 0; bit < 8; bit++ )
    {
        if( ( m_crc & 0x8000 ) != 0 )
        {
            m_crc = static_cast< uint16_t >( ( m_crc << 1 ) ^ 0x1021 );
        }
        else
        {
            m_crc = static_cast< uint16_t >( m_crc << 1 );
        }
    }
}

uint16_t RoboClaw::crcGet() const
{
    return m_crc;
}

void RoboClaw::sendByte( uint8_t a_byte )
{
    crcUpdate( a_byte );
    write( a_byte );
}

void RoboClaw::sendCommand( Command a_cmd )
{
    flush();
    crcClear();
    sendByte( m_address );
    sendByte( std::to_underlying( a_cmd ) );
}

void RoboClaw::sendPacket( Command a_cmd, std::span< const uint8_t > a_payload )
{
    crcClear();
    sendByte( m_address );
    sendByte( std::to_underlying( a_cmd ) );
    for( const uint8_t byte : a_payload )
    {
        sendByte( byte );
    }
    const uint16_t crc = crcGet();
    write( static_cast< uint8_t >( crc >> 8 ) );
    write( static_cast< uint8_t >( crc ) );
}

bool RoboClaw::writeN( Command a_cmd, std::span< const uint8_t > a_payload )
{
    for( int attempt = 0; attempt <= maxRetry; attempt++ )
    {
        sendPacket( a_cmd, a_payload );
        if( read( m_timeout ) == 0xFF )
        {
            return true;
        }
    }
    return false;
}

// These commands touch the flash and ack late, so wait longer instead of sending them again.
bool RoboClaw::writeSlow( Command a_cmd, std::span< const uint8_t > a_payload )
{
    sendPacket( a_cmd, a_payload );
    return read( 2000 ) == 0xFF;
}

bool RoboClaw::readBytes( Command a_cmd, std::span< uint8_t > a_data, bool a_retryBadCrc )
{
    for( int attempt = 0; attempt <= maxRetry; attempt++ )
    {
        sendCommand( a_cmd );

        bool complete = true;
        for( size_t index = 0; index < a_data.size() && complete; index++ )
        {
            const int16_t value = read( m_timeout );
            complete = value != -1;
            a_data[ index ] = static_cast< uint8_t >( value );
            crcUpdate( a_data[ index ] );
        }
        if( complete )
        {
            const int16_t high = read( m_timeout );
            const int16_t low = high == -1 ? int16_t{ -1 } : read( m_timeout );
            if( low != -1 )
            {
                if( crcGet() == toWord( high, low ) )
                {
                    return true;
                }
                if( !a_retryBadCrc )
                {
                    return false;
                }
            }
        }
    }
    return false;
}

template< size_t Count >
std::optional< std::array< uint8_t, Count > > RoboClaw::readArray( Command a_cmd, bool a_retryBadCrc )
{
    std::array< uint8_t, Count > data{};
    if( !readBytes( a_cmd, data, a_retryBadCrc ) )
    {
        return std::nullopt;
    }
    return data;
}

std::optional< RoboClaw::status_value > RoboClaw::readStatusValue( Command a_cmd )
{
    return readArray< 5 >( a_cmd ).transform( []( const auto& a_data )
                                              { return status_value{ getDword( a_data ), a_data[ 4 ] }; } );
}

std::optional< RoboClaw::velocity_pid > RoboClaw::readVelocityPID( Command a_cmd )
{
    return readArray< 16 >( a_cmd, false ).transform( []( const auto& a_data )
                                                      { return velocity_pid{ fromFixed( getDword( a_data ), velocityPidScale ), fromFixed( getDword( a_data, 4 ), velocityPidScale ), fromFixed( getDword( a_data, 8 ), velocityPidScale ), getDword( a_data, 12 ) }; } );
}

std::optional< RoboClaw::position_pid > RoboClaw::readPositionPID( Command a_cmd )
{
    return readArray< 28 >( a_cmd, false ).transform( []( const auto& a_data )
                                                      { return position_pid{ fromFixed( getDword( a_data ), positionPidScale ), fromFixed( getDword( a_data, 4 ), positionPidScale ), fromFixed( getDword( a_data, 8 ), positionPidScale ), getDword( a_data, 12 ), getDword( a_data, 16 ), getDword( a_data, 20 ), getDword( a_data, 24 ) }; } );
}

bool RoboClaw::forwardM1( uint8_t a_speed )
{
    return writeN( Command::M1FORWARD, payload( a_speed ) );
}

bool RoboClaw::backwardM1( uint8_t a_speed )
{
    return writeN( Command::M1BACKWARD, payload( a_speed ) );
}

bool RoboClaw::setMinVoltageMainBattery( uint8_t a_voltage )
{
    return writeN( Command::SETMINMB, payload( a_voltage ) );
}

bool RoboClaw::setMaxVoltageMainBattery( uint8_t a_voltage )
{
    return writeN( Command::SETMAXMB, payload( a_voltage ) );
}

bool RoboClaw::forwardM2( uint8_t a_speed )
{
    return writeN( Command::M2FORWARD, payload( a_speed ) );
}

bool RoboClaw::backwardM2( uint8_t a_speed )
{
    return writeN( Command::M2BACKWARD, payload( a_speed ) );
}

bool RoboClaw::forwardBackwardM1( uint8_t a_speed )
{
    return writeN( Command::M17BIT, payload( a_speed ) );
}

bool RoboClaw::forwardBackwardM2( uint8_t a_speed )
{
    return writeN( Command::M27BIT, payload( a_speed ) );
}

bool RoboClaw::forwardMixed( uint8_t a_speed )
{
    return writeN( Command::MIXEDFORWARD, payload( a_speed ) );
}

bool RoboClaw::backwardMixed( uint8_t a_speed )
{
    return writeN( Command::MIXEDBACKWARD, payload( a_speed ) );
}

bool RoboClaw::turnRightMixed( uint8_t a_speed )
{
    return writeN( Command::MIXEDRIGHT, payload( a_speed ) );
}

bool RoboClaw::turnLeftMixed( uint8_t a_speed )
{
    return writeN( Command::MIXEDLEFT, payload( a_speed ) );
}

bool RoboClaw::forwardBackwardMixed( uint8_t a_speed )
{
    return writeN( Command::MIXEDFB, payload( a_speed ) );
}

bool RoboClaw::leftRightMixed( uint8_t a_speed )
{
    return writeN( Command::MIXEDLR, payload( a_speed ) );
}

std::optional< RoboClaw::status_value > RoboClaw::readEncM1()
{
    return readStatusValue( Command::GETM1ENC );
}

std::optional< RoboClaw::status_value > RoboClaw::readEncM2()
{
    return readStatusValue( Command::GETM2ENC );
}

std::optional< RoboClaw::status_value > RoboClaw::readSpeedM1()
{
    return readStatusValue( Command::GETM1SPEED );
}

std::optional< RoboClaw::status_value > RoboClaw::readSpeedM2()
{
    return readStatusValue( Command::GETM2SPEED );
}

bool RoboClaw::resetEncoders()
{
    return writeN( Command::RESETENC );
}

std::optional< std::string > RoboClaw::readVersion()
{
    for( int attempt = 0; attempt <= maxRetry; attempt++ )
    {
        sendCommand( Command::GETVERSION );
        std::string version;
        for( int index = 0; index < 48; index++ )
        {
            const int16_t data = read( m_timeout );
            if( data == -1 )
            {
                break;
            }
            crcUpdate( static_cast< uint8_t >( data ) );
            if( data == 0 )
            {
                const int16_t high = read( m_timeout );
                const int16_t low = high == -1 ? int16_t{ -1 } : read( m_timeout );
                if( low != -1 )
                {
                    return crcGet() == toWord( high, low ) ? std::optional( version ) : std::nullopt;
                }
                break;
            }
            version.push_back( static_cast< char >( data ) );
        }
    }
    return std::nullopt;
}

bool RoboClaw::setEncM1( int32_t a_val )
{
    return writeN( Command::SETM1ENCCOUNT, payload( static_cast< uint32_t >( a_val ) ) );
}

bool RoboClaw::setEncM2( int32_t a_val )
{
    return writeN( Command::SETM2ENCCOUNT, payload( static_cast< uint32_t >( a_val ) ) );
}

std::optional< uint16_t > RoboClaw::readMainBatteryVoltage()
{
    return readArray< 2 >( Command::GETMBATT ).transform( []( const auto& a_data )
                                                          { return getWord( a_data ); } );
}

std::optional< uint16_t > RoboClaw::readLogicBatteryVoltage()
{
    return readArray< 2 >( Command::GETLBATT ).transform( []( const auto& a_data )
                                                          { return getWord( a_data ); } );
}

bool RoboClaw::setMinVoltageLogicBattery( uint8_t a_voltage )
{
    return writeN( Command::SETMINLB, payload( a_voltage ) );
}

bool RoboClaw::setMaxVoltageLogicBattery( uint8_t a_voltage )
{
    return writeN( Command::SETMAXLB, payload( a_voltage ) );
}

bool RoboClaw::setM1VelocityPID( float a_kp, float a_ki, float a_kd, uint32_t a_qpps )
{
    return writeN( Command::SETM1PID, payload( toFixed( a_kd, velocityPidScale ), toFixed( a_kp, velocityPidScale ), toFixed( a_ki, velocityPidScale ), a_qpps ) );
}

bool RoboClaw::setM2VelocityPID( float a_kp, float a_ki, float a_kd, uint32_t a_qpps )
{
    return writeN( Command::SETM2PID, payload( toFixed( a_kd, velocityPidScale ), toFixed( a_kp, velocityPidScale ), toFixed( a_ki, velocityPidScale ), a_qpps ) );
}

std::optional< RoboClaw::status_value > RoboClaw::readISpeedM1()
{
    return readStatusValue( Command::GETM1ISPEED );
}

std::optional< RoboClaw::status_value > RoboClaw::readISpeedM2()
{
    return readStatusValue( Command::GETM2ISPEED );
}

bool RoboClaw::dutyM1( uint16_t a_duty )
{
    return writeN( Command::M1DUTY, payload( a_duty ) );
}

bool RoboClaw::dutyM2( uint16_t a_duty )
{
    return writeN( Command::M2DUTY, payload( a_duty ) );
}

bool RoboClaw::dutyM1M2( uint16_t a_duty1, uint16_t a_duty2 )
{
    return writeN( Command::MIXEDDUTY, payload( a_duty1, a_duty2 ) );
}

bool RoboClaw::speedM1( uint32_t a_speed )
{
    return writeN( Command::M1SPEED, payload( a_speed ) );
}

bool RoboClaw::speedM2( uint32_t a_speed )
{
    return writeN( Command::M2SPEED, payload( a_speed ) );
}

bool RoboClaw::speedM1M2( uint32_t a_speed1, uint32_t a_speed2 )
{
    return writeN( Command::MIXEDSPEED, payload( a_speed1, a_speed2 ) );
}

bool RoboClaw::speedAccelM1( uint32_t a_accel, uint32_t a_speed )
{
    return writeN( Command::M1SPEEDACCEL, payload( a_accel, a_speed ) );
}

bool RoboClaw::speedAccelM2( uint32_t a_accel, uint32_t a_speed )
{
    return writeN( Command::M2SPEEDACCEL, payload( a_accel, a_speed ) );
}

bool RoboClaw::speedAccelM1M2( uint32_t a_accel, uint32_t a_speed1, uint32_t a_speed2 )
{
    return writeN( Command::MIXEDSPEEDACCEL, payload( a_accel, a_speed1, a_speed2 ) );
}

bool RoboClaw::speedDistanceM1( uint32_t a_speed, uint32_t a_distance, uint8_t a_flag )
{
    return writeN( Command::M1SPEEDDIST, payload( a_speed, a_distance, a_flag ) );
}

bool RoboClaw::speedDistanceM2( uint32_t a_speed, uint32_t a_distance, uint8_t a_flag )
{
    return writeN( Command::M2SPEEDDIST, payload( a_speed, a_distance, a_flag ) );
}

bool RoboClaw::speedDistanceM1M2( uint32_t a_speed1, uint32_t a_distance1, uint32_t a_speed2, uint32_t a_distance2, uint8_t a_flag )
{
    return writeN( Command::MIXEDSPEEDDIST, payload( a_speed1, a_distance1, a_speed2, a_distance2, a_flag ) );
}

bool RoboClaw::speedAccelDistanceM1( uint32_t a_accel, uint32_t a_speed, uint32_t a_distance, uint8_t a_flag )
{
    return writeN( Command::M1SPEEDACCELDIST, payload( a_accel, a_speed, a_distance, a_flag ) );
}

bool RoboClaw::speedAccelDistanceM2( uint32_t a_accel, uint32_t a_speed, uint32_t a_distance, uint8_t a_flag )
{
    return writeN( Command::M2SPEEDACCELDIST, payload( a_accel, a_speed, a_distance, a_flag ) );
}

bool RoboClaw::speedAccelDistanceM1M2( uint32_t a_accel, uint32_t a_speed1, uint32_t a_distance1, uint32_t a_speed2, uint32_t a_distance2, uint8_t a_flag )
{
    return writeN( Command::MIXEDSPEEDACCELDIST, payload( a_accel, a_speed1, a_distance1, a_speed2, a_distance2, a_flag ) );
}

std::optional< RoboClaw::motor_values< uint8_t > > RoboClaw::readBuffers()
{
    return readArray< 2 >( Command::GETBUFFERS ).transform( []( const auto& a_data )
                                                            { return motor_values< uint8_t >{ a_data[ 0 ], a_data[ 1 ] }; } );
}

std::optional< RoboClaw::motor_values< int16_t > > RoboClaw::readPWMs()
{
    return readArray< 4 >( Command::GETPWMS ).transform( []( const auto& a_data )
                                                         { return motor_values< int16_t >{ .m1 = static_cast< int16_t >( getWord( a_data ) ), .m2 = static_cast< int16_t >( getWord( a_data, 2 ) ) }; } );
}

std::optional< RoboClaw::motor_values< int16_t > > RoboClaw::readCurrents()
{
    return readArray< 4 >( Command::GETCURRENTS ).transform( []( const auto& a_data )
                                                             { return motor_values< int16_t >{ .m1 = static_cast< int16_t >( getWord( a_data ) ), .m2 = static_cast< int16_t >( getWord( a_data, 2 ) ) }; } );
}

bool RoboClaw::speedAccelM1M2Separate( uint32_t a_accel1, uint32_t a_speed1, uint32_t a_accel2, uint32_t a_speed2 )
{
    return writeN( Command::MIXEDSPEED2ACCEL, payload( a_accel1, a_speed1, a_accel2, a_speed2 ) );
}

bool RoboClaw::speedAccelDistanceM1M2Separate( uint32_t a_accel1, uint32_t a_speed1, uint32_t a_distance1, uint32_t a_accel2, uint32_t a_speed2, uint32_t a_distance2, uint8_t a_flag )
{
    return writeN( Command::MIXEDSPEED2ACCELDIST, payload( a_accel1, a_speed1, a_distance1, a_accel2, a_speed2, a_distance2, a_flag ) );
}

bool RoboClaw::dutyAccelM1( uint16_t a_duty, uint32_t a_accel )
{
    return writeN( Command::M1DUTYACCEL, payload( a_duty, a_accel ) );
}

bool RoboClaw::dutyAccelM2( uint16_t a_duty, uint32_t a_accel )
{
    return writeN( Command::M2DUTYACCEL, payload( a_duty, a_accel ) );
}

bool RoboClaw::dutyAccelM1M2( uint16_t a_duty1, uint32_t a_accel1, uint16_t a_duty2, uint32_t a_accel2 )
{
    return writeN( Command::MIXEDDUTYACCEL, payload( a_duty1, a_accel1, a_duty2, a_accel2 ) );
}

std::optional< RoboClaw::velocity_pid > RoboClaw::readM1VelocityPID()
{
    return readVelocityPID( Command::READM1PID );
}

std::optional< RoboClaw::velocity_pid > RoboClaw::readM2VelocityPID()
{
    return readVelocityPID( Command::READM2PID );
}

bool RoboClaw::setMainVoltages( uint16_t a_min, uint16_t a_max )
{
    return writeN( Command::SETMAINVOLTAGES, payload( a_min, a_max ) );
}

bool RoboClaw::setLogicVoltages( uint16_t a_min, uint16_t a_max )
{
    return writeN( Command::SETLOGICVOLTAGES, payload( a_min, a_max ) );
}

std::optional< RoboClaw::range< uint16_t > > RoboClaw::readMinMaxMainVoltages()
{
    return readArray< 4 >( Command::GETMINMAXMAINVOLTAGES ).transform( []( const auto& a_data )
                                                                       { return range< uint16_t >{ getWord( a_data ), getWord( a_data, 2 ) }; } );
}

std::optional< RoboClaw::range< uint16_t > > RoboClaw::readMinMaxLogicVoltages()
{
    return readArray< 4 >( Command::GETMINMAXLOGICVOLTAGES ).transform( []( const auto& a_data )
                                                                        { return range< uint16_t >{ getWord( a_data ), getWord( a_data, 2 ) }; } );
}

bool RoboClaw::setM1PositionPID( float a_kp, float a_ki, float a_kd, uint32_t a_kiMax, uint32_t a_deadzone, uint32_t a_min, uint32_t a_max )
{
    return writeN( Command::SETM1POSPID, payload( toFixed( a_kd, positionPidScale ), toFixed( a_kp, positionPidScale ), toFixed( a_ki, positionPidScale ), a_kiMax, a_deadzone, a_min, a_max ) );
}

bool RoboClaw::setM2PositionPID( float a_kp, float a_ki, float a_kd, uint32_t a_kiMax, uint32_t a_deadzone, uint32_t a_min, uint32_t a_max )
{
    return writeN( Command::SETM2POSPID, payload( toFixed( a_kd, positionPidScale ), toFixed( a_kp, positionPidScale ), toFixed( a_ki, positionPidScale ), a_kiMax, a_deadzone, a_min, a_max ) );
}

std::optional< RoboClaw::position_pid > RoboClaw::readM1PositionPID()
{
    return readPositionPID( Command::READM1POSPID );
}

std::optional< RoboClaw::position_pid > RoboClaw::readM2PositionPID()
{
    return readPositionPID( Command::READM2POSPID );
}

bool RoboClaw::speedAccelDeccelPositionM1( uint32_t a_accel, uint32_t a_speed, uint32_t a_deccel, uint32_t a_position, uint8_t a_flag )
{
    return writeN( Command::M1SPEEDACCELDECCELPOS, payload( a_accel, a_speed, a_deccel, a_position, a_flag ) );
}

bool RoboClaw::speedAccelDeccelPositionM2( uint32_t a_accel, uint32_t a_speed, uint32_t a_deccel, uint32_t a_position, uint8_t a_flag )
{
    return writeN( Command::M2SPEEDACCELDECCELPOS, payload( a_accel, a_speed, a_deccel, a_position, a_flag ) );
}

bool RoboClaw::speedAccelDeccelPositionM1M2( uint32_t a_accel1, uint32_t a_speed1, uint32_t a_deccel1, uint32_t a_position1, uint32_t a_accel2, uint32_t a_speed2, uint32_t a_deccel2, uint32_t a_position2, uint8_t a_flag )
{
    return writeN( Command::MIXEDSPEEDACCELDECCELPOS, payload( a_accel1, a_speed1, a_deccel1, a_position1, a_accel2, a_speed2, a_deccel2, a_position2, a_flag ) );
}

bool RoboClaw::setM1DefaultAccel( uint32_t a_accel, uint32_t a_deccel )
{
    return writeN( Command::SETM1DEFAULTACCEL, payload( a_accel, a_deccel ) );
}

bool RoboClaw::setM2DefaultAccel( uint32_t a_accel, uint32_t a_deccel )
{
    return writeN( Command::SETM2DEFAULTACCEL, payload( a_accel, a_deccel ) );
}

bool RoboClaw::setPinFunctions( uint8_t a_s3mode, uint8_t a_s4mode, uint8_t a_s5mode )
{
    return writeN( Command::SETPINFUNCTIONS, payload( a_s3mode, a_s4mode, a_s5mode ) );
}

std::optional< RoboClaw::pin_functions > RoboClaw::getPinFunctions()
{
    return readArray< 3 >( Command::GETPINFUNCTIONS ).transform( []( const auto& a_data )
                                                                 { return pin_functions{ a_data[ 0 ], a_data[ 1 ], a_data[ 2 ] }; } );
}

bool RoboClaw::setDeadBand( uint8_t a_min, uint8_t a_max )
{
    return writeN( Command::SETDEADBAND, payload( a_min, a_max ) );
}

std::optional< RoboClaw::range< uint8_t > > RoboClaw::getDeadBand()
{
    return readArray< 2 >( Command::GETDEADBAND ).transform( []( const auto& a_data )
                                                             { return range< uint8_t >{ a_data[ 0 ], a_data[ 1 ] }; } );
}

std::optional< RoboClaw::motor_values< uint32_t > > RoboClaw::readEncoders()
{
    return readArray< 8 >( Command::GETENCODERS, false ).transform( []( const auto& a_data )
                                                                    { return motor_values< uint32_t >{ getDword( a_data ), getDword( a_data, 4 ) }; } );
}

std::optional< RoboClaw::motor_values< uint32_t > > RoboClaw::readISpeeds()
{
    return readArray< 8 >( Command::GETISPEEDS, false ).transform( []( const auto& a_data )
                                                                   { return motor_values< uint32_t >{ getDword( a_data ), getDword( a_data, 4 ) }; } );
}

bool RoboClaw::restoreDefaults()
{
    return writeSlow( Command::RESTOREDEFAULTS );
}

std::optional< uint16_t > RoboClaw::readTemp()
{
    return readArray< 2 >( Command::GETTEMP ).transform( []( const auto& a_data )
                                                         { return getWord( a_data ); } );
}

std::optional< uint16_t > RoboClaw::readTemp2()
{
    return readArray< 2 >( Command::GETTEMP2 ).transform( []( const auto& a_data )
                                                          { return getWord( a_data ); } );
}

std::optional< uint32_t > RoboClaw::readError()
{
    return readArray< 4 >( Command::GETERROR ).transform( []( const auto& a_data )
                                                          { return getDword( a_data ); } );
}

std::optional< RoboClaw::motor_values< uint8_t > > RoboClaw::readEncoderModes()
{
    return readArray< 2 >( Command::GETENCODERMODE ).transform( []( const auto& a_data )
                                                                { return motor_values< uint8_t >{ a_data[ 0 ], a_data[ 1 ] }; } );
}

bool RoboClaw::setM1EncoderMode( uint8_t a_mode )
{
    return writeN( Command::SETM1ENCODERMODE, payload( a_mode ) );
}

bool RoboClaw::setM2EncoderMode( uint8_t a_mode )
{
    return writeN( Command::SETM2ENCODERMODE, payload( a_mode ) );
}

bool RoboClaw::writeNVM()
{
    return writeSlow( Command::WRITENVM, payload( uint32_t{ 0xE22EAB7A } ) );
}

bool RoboClaw::readNVM()
{
    return writeSlow( Command::READNVM );
}

bool RoboClaw::setConfig( uint16_t a_config )
{
    return writeN( Command::SETCONFIG, payload( a_config ) );
}

std::optional< uint16_t > RoboClaw::getConfig()
{
    return readArray< 2 >( Command::GETCONFIG ).transform( []( const auto& a_data )
                                                           { return getWord( a_data ); } );
}

bool RoboClaw::setM1MaxCurrent( uint32_t a_max )
{
    return writeN( Command::SETM1MAXCURRENT, payload( a_max, uint32_t{ 0 } ) );
}

bool RoboClaw::setM2MaxCurrent( uint32_t a_max )
{
    return writeN( Command::SETM2MAXCURRENT, payload( a_max, uint32_t{ 0 } ) );
}

std::optional< uint32_t > RoboClaw::readM1MaxCurrent()
{
    return readArray< 8 >( Command::GETM1MAXCURRENT, false ).transform( []( const auto& a_data )
                                                                        { return getDword( a_data ); } );
}

std::optional< uint32_t > RoboClaw::readM2MaxCurrent()
{
    return readArray< 8 >( Command::GETM2MAXCURRENT, false ).transform( []( const auto& a_data )
                                                                        { return getDword( a_data ); } );
}

bool RoboClaw::setPWMMode( uint8_t a_mode )
{
    return writeN( Command::SETPWMMODE, payload( a_mode ) );
}

std::optional< uint8_t > RoboClaw::getPWMMode()
{
    return readArray< 1 >( Command::GETPWMMODE ).transform( []( const auto& a_data )
                                                            { return a_data[ 0 ]; } );
}

std::optional< RoboClaw::motor_values< RoboClaw::acceleration > > RoboClaw::getDefaultAccels()
{
    return readArray< 16 >( Command::GETDEFAULTACCELS ).transform( []( const auto& a_data )
                                                                   { return motor_values< acceleration >{ { getDword( a_data ), getDword( a_data, 4 ) }, { getDword( a_data, 8 ), getDword( a_data, 12 ) } }; } );
}

bool RoboClaw::setTimeout( uint8_t a_timeout )
{
    return writeN( Command::SETTIMEOUT, payload( a_timeout ) );
}

std::optional< uint8_t > RoboClaw::getTimeout()
{
    return readArray< 1 >( Command::GETTIMEOUT ).transform( []( const auto& a_data )
                                                            { return a_data[ 0 ]; } );
}

bool RoboClaw::setM1DefaultSpeed( uint16_t a_speed )
{
    return writeN( Command::SETM1DEFAULTSPEED, payload( a_speed ) );
}

bool RoboClaw::setM2DefaultSpeed( uint16_t a_speed )
{
    return writeN( Command::SETM2DEFAULTSPEED, payload( a_speed ) );
}

std::optional< RoboClaw::motor_values< uint16_t > > RoboClaw::getDefaultSpeeds()
{
    return readArray< 4 >( Command::GETDEFAULTSPEEDS ).transform( []( const auto& a_data )
                                                                  { return motor_values< uint16_t >{ getWord( a_data ), getWord( a_data, 2 ) }; } );
}

bool RoboClaw::setCtrlModes( uint8_t a_ctrl1, uint8_t a_ctrl2 )
{
    return writeN( Command::SETCTRLMODES, payload( a_ctrl1, a_ctrl2 ) );
}

std::optional< RoboClaw::motor_values< uint8_t > > RoboClaw::getCtrlModes()
{
    return readArray< 2 >( Command::GETCTRLMODES ).transform( []( const auto& a_data )
                                                              { return motor_values< uint8_t >{ a_data[ 0 ], a_data[ 1 ] }; } );
}

bool RoboClaw::setCtrl1( uint16_t a_value )
{
    return writeN( Command::SETCTRL1, payload( a_value ) );
}

bool RoboClaw::setCtrl2( uint16_t a_value )
{
    return writeN( Command::SETCTRL2, payload( a_value ) );
}

std::optional< RoboClaw::motor_values< uint16_t > > RoboClaw::getCtrls()
{
    return readArray< 4 >( Command::GETCTRLS ).transform( []( const auto& a_data )
                                                          { return motor_values< uint16_t >{ getWord( a_data ), getWord( a_data, 2 ) }; } );
}

bool RoboClaw::setAuto1( uint16_t a_duty, uint32_t a_timeout )
{
    return writeN( Command::SETAUTO1, payload( a_duty, a_timeout ) );
}

bool RoboClaw::setAuto2( uint16_t a_duty, uint32_t a_timeout )
{
    return writeN( Command::SETAUTO2, payload( a_duty, a_timeout ) );
}

std::optional< RoboClaw::motor_values< RoboClaw::auto_setting > > RoboClaw::getAutos()
{
    return readArray< 12 >( Command::GETAUTOS ).transform( []( const auto& a_data )
                                                           { return motor_values< auto_setting >{ { getWord( a_data ), getDword( a_data, 2 ) }, { getWord( a_data, 6 ), getDword( a_data, 8 ) } }; } );
}

std::optional< RoboClaw::motor_values< int32_t > > RoboClaw::readSpeeds()
{
    return readArray< 8 >( Command::GETSPEEDS ).transform( []( const auto& a_data )
                                                           { return motor_values< int32_t >{ .m1 = static_cast< int32_t >( getDword( a_data ) ), .m2 = static_cast< int32_t >( getDword( a_data, 4 ) ) }; } );
}

bool RoboClaw::setSpeedErrorLimits( uint32_t a_limit1, uint32_t a_limit2 )
{
    return writeN( Command::SETSPEEDERRORLIMIT, payload( a_limit1, a_limit2 ) );
}

std::optional< RoboClaw::motor_values< uint32_t > > RoboClaw::getSpeedErrorLimits()
{
    return readArray< 8 >( Command::GETSPEEDERRORLIMIT ).transform( []( const auto& a_data )
                                                                    { return motor_values< uint32_t >{ getDword( a_data ), getDword( a_data, 4 ) }; } );
}

std::optional< RoboClaw::motor_values< int32_t > > RoboClaw::readSpeedErrors()
{
    return readArray< 8 >( Command::GETSPEEDERRORS ).transform( []( const auto& a_data )
                                                                { return motor_values< int32_t >{ .m1 = static_cast< int32_t >( getDword( a_data ) ), .m2 = static_cast< int32_t >( getDword( a_data, 4 ) ) }; } );
}

bool RoboClaw::setPosErrorLimits( uint32_t a_limit1, uint32_t a_limit2 )
{
    return writeN( Command::SETPOSERRORLIMIT, payload( a_limit1, a_limit2 ) );
}

std::optional< RoboClaw::motor_values< uint32_t > > RoboClaw::getPosErrorLimits()
{
    return readArray< 8 >( Command::GETPOSERRORLIMIT ).transform( []( const auto& a_data )
                                                                  { return motor_values< uint32_t >{ getDword( a_data ), getDword( a_data, 4 ) }; } );
}

std::optional< RoboClaw::motor_values< int32_t > > RoboClaw::readPosErrors()
{
    return readArray< 8 >( Command::GETPOSERRORS ).transform( []( const auto& a_data )
                                                              { return motor_values< int32_t >{ .m1 = static_cast< int32_t >( getDword( a_data ) ), .m2 = static_cast< int32_t >( getDword( a_data, 4 ) ) }; } );
}

bool RoboClaw::setVoltageOffsets( int8_t a_main, int8_t a_logic )
{
    return writeN( Command::SETVOLTAGEOFFSETS, payload( static_cast< uint8_t >( a_main ), static_cast< uint8_t >( a_logic ) ) );
}

std::optional< RoboClaw::voltage_offsets > RoboClaw::getVoltageOffsets()
{
    return readArray< 2 >( Command::GETVOLTAGEOFFSETS ).transform( []( const auto& a_data )
                                                                   { return voltage_offsets{ .main = static_cast< int8_t >( a_data[ 0 ] ), .logic = static_cast< int8_t >( a_data[ 1 ] ) }; } );
}

bool RoboClaw::setCurrentBlanking( uint16_t a_blanking1, uint16_t a_blanking2 )
{
    return writeN( Command::SETCURRENTBLANKING, payload( a_blanking1, a_blanking2 ) );
}

std::optional< RoboClaw::motor_values< uint16_t > > RoboClaw::getCurrentBlanking()
{
    return readArray< 4 >( Command::GETCURRENTBLANKING ).transform( []( const auto& a_data )
                                                                    { return motor_values< uint16_t >{ getWord( a_data ), getWord( a_data, 2 ) }; } );
}
