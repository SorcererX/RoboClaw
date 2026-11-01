/// @file roboclaw.h
/// @brief Packet serial driver for Basicmicro RoboClaw motor controllers.

#ifndef RoboClaw_h
#define RoboClaw_h

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <termios.h>

/// @brief Packet serial driver for a RoboClaw motor controller.
///
/// Tested against a RoboClaw 2x30A running firmware 4.1.34.
///
/// Settings are written to RAM until writeNVM() saves them to flash.
class RoboClaw
{
protected:
    /// @cond INTERNAL
    enum class Command : uint8_t
    {
        M1FORWARD = 0,
        M1BACKWARD = 1,
        SETMINMB = 2,
        SETMAXMB = 3,
        M2FORWARD = 4,
        M2BACKWARD = 5,
        M17BIT = 6,
        M27BIT = 7,
        MIXEDFORWARD = 8,
        MIXEDBACKWARD = 9,
        MIXEDRIGHT = 10,
        MIXEDLEFT = 11,
        MIXEDFB = 12,
        MIXEDLR = 13,
        SETTIMEOUT = 14,
        GETTIMEOUT = 15,
        GETM1ENC = 16,
        GETM2ENC = 17,
        GETM1SPEED = 18,
        GETM2SPEED = 19,
        RESETENC = 20,
        GETVERSION = 21,
        SETM1ENCCOUNT = 22,
        SETM2ENCCOUNT = 23,
        GETMBATT = 24,
        GETLBATT = 25,
        SETMINLB = 26,
        SETMAXLB = 27,
        SETM1PID = 28,
        SETM2PID = 29,
        GETM1ISPEED = 30,
        GETM2ISPEED = 31,
        M1DUTY = 32,
        M2DUTY = 33,
        MIXEDDUTY = 34,
        M1SPEED = 35,
        M2SPEED = 36,
        MIXEDSPEED = 37,
        M1SPEEDACCEL = 38,
        M2SPEEDACCEL = 39,
        MIXEDSPEEDACCEL = 40,
        M1SPEEDDIST = 41,
        M2SPEEDDIST = 42,
        MIXEDSPEEDDIST = 43,
        M1SPEEDACCELDIST = 44,
        M2SPEEDACCELDIST = 45,
        MIXEDSPEEDACCELDIST = 46,
        GETBUFFERS = 47,
        GETPWMS = 48,
        GETCURRENTS = 49,
        MIXEDSPEED2ACCEL = 50,
        MIXEDSPEED2ACCELDIST = 51,
        M1DUTYACCEL = 52,
        M2DUTYACCEL = 53,
        MIXEDDUTYACCEL = 54,
        READM1PID = 55,
        READM2PID = 56,
        SETMAINVOLTAGES = 57,
        SETLOGICVOLTAGES = 58,
        GETMINMAXMAINVOLTAGES = 59,
        GETMINMAXLOGICVOLTAGES = 60,
        SETM1POSPID = 61,
        SETM2POSPID = 62,
        READM1POSPID = 63,
        READM2POSPID = 64,
        M1SPEEDACCELDECCELPOS = 65,
        M2SPEEDACCELDECCELPOS = 66,
        MIXEDSPEEDACCELDECCELPOS = 67,
        SETM1DEFAULTACCEL = 68,
        SETM2DEFAULTACCEL = 69,
        SETM1DEFAULTSPEED = 70,
        SETM2DEFAULTSPEED = 71,
        GETDEFAULTSPEEDS = 72,
        SETPINFUNCTIONS = 74,
        GETPINFUNCTIONS = 75,
        SETDEADBAND = 76,
        GETDEADBAND = 77,
        GETENCODERS = 78,
        GETISPEEDS = 79,
        RESTOREDEFAULTS = 80,
        GETDEFAULTACCELS = 81,
        GETTEMP = 82,
        GETTEMP2 = 83, // Only valid on some models
        GETERROR = 90,
        GETENCODERMODE = 91,
        SETM1ENCODERMODE = 92,
        SETM2ENCODERMODE = 93,
        WRITENVM = 94,
        READNVM = 95, // Reloads values from Flash into Ram
        SETCONFIG = 98,
        GETCONFIG = 99,
        SETCTRLMODES = 100,
        GETCTRLMODES = 101,
        SETCTRL1 = 102,
        SETCTRL2 = 103,
        GETCTRLS = 104,
        SETAUTO1 = 105,
        SETAUTO2 = 106,
        GETAUTOS = 107,
        GETSPEEDS = 108,
        SETSPEEDERRORLIMIT = 109,
        GETSPEEDERRORLIMIT = 110,
        GETSPEEDERRORS = 111,
        SETPOSERRORLIMIT = 112,
        GETPOSERRORLIMIT = 113,
        GETPOSERRORS = 114,
        SETVOLTAGEOFFSETS = 115,
        GETVOLTAGEOFFSETS = 116,
        SETCURRENTBLANKING = 117,
        GETCURRENTBLANKING = 118,
        SETM1MAXCURRENT = 133,
        SETM2MAXCURRENT = 134,
        GETM1MAXCURRENT = 135,
        GETM2MAXCURRENT = 136,
        SETPWMMODE = 148,
        GETPWMMODE = 149,
        FLAGBOOTLOADER = 255 // Only available via USB communications
    };
    /// @endcond

public:
    /// @brief One value for each motor.
    /// @tparam Value Type of the values.
    template< typename Value >
    struct motor_values
    {
        /// M1 value.
        Value m1;
        /// M2 value.
        Value m2;
    };

    /// @brief A minimum and a maximum.
    /// @tparam Value Type of the limits.
    template< typename Value >
    struct range
    {
        /// Minimum.
        Value min;
        /// Maximum.
        Value max;
    };

    /// @brief A 32-bit reading and the status byte sent with it. The meaning of the status bits depends on the command, see the manual.
    struct status_value
    {
        /// The reading.
        uint32_t value;
        /// The status byte.
        uint8_t status;
    };

    /// @brief Velocity PID settings.
    struct velocity_pid
    {
        /// Proportional gain.
        float kp;
        /// Integral gain.
        float ki;
        /// Derivative gain.
        float kd;
        /// Encoder speed at full power, in pulses per second.
        uint32_t qpps;
    };

    /// @brief Position PID settings.
    struct position_pid
    {
        /// Proportional gain.
        float kp;
        /// Integral gain.
        float ki;
        /// Derivative gain.
        float kd;
        /// Integral limit.
        uint32_t kiMax;
        /// Deadzone in pulses.
        uint32_t deadZone;
        /// Minimum position in pulses.
        uint32_t min;
        /// Maximum position in pulses.
        uint32_t max;
    };

    /// @brief Acceleration and deceleration of one motor.
    struct acceleration
    {
        /// Acceleration in pulses per second squared.
        uint32_t accel;
        /// Deceleration in pulses per second squared.
        uint32_t deccel;
    };

    /// @brief Auto duty and timeout of one motor.
    struct auto_setting
    {
        /// Duty value.
        uint16_t duty;
        /// Timeout.
        uint32_t timeout;
    };

    /// @brief Modes of the S3, S4 and S5 pins.
    struct pin_functions
    {
        /// S3 mode.
        uint8_t s3;
        /// S4 mode.
        uint8_t s4;
        /// S5 mode.
        uint8_t s5;
    };

    /// @brief Battery reading offsets in tenths of a volt.
    struct voltage_offsets
    {
        /// Main battery offset.
        int8_t main;
        /// Logic battery offset.
        int8_t logic;
    };

    /// @brief Creates an object for /dev/ttyACM0 with a 5 ms reply timeout and address 0x80. Call begin() to open the port.
    RoboClaw();
    /// @brief Creates an object for a serial port. Call begin() to open it.
    /// @param a_portName Serial device path.
    /// @param a_timeout Reply timeout in milliseconds.
    /// @param a_address Packet serial address, 0x80 to 0x87. Ignored on USB.
    RoboClaw( std::string a_portName, uint8_t a_timeout, uint8_t a_address );
    RoboClaw( const RoboClaw& ) = delete;
    RoboClaw& operator=( const RoboClaw& ) = delete;
    RoboClaw( RoboClaw&& ) = delete;
    RoboClaw& operator=( RoboClaw&& ) = delete;

    virtual ~RoboClaw() = default;

    /// @brief Drives M1 forward.
    /// @param a_speed Speed from 0 (stopped) to 127 (full speed).
    /// @return True if the controller acknowledged the command.
    bool forwardM1( uint8_t a_speed );
    /// @brief Drives M1 backward.
    /// @param a_speed Speed from 0 (stopped) to 127 (full speed).
    /// @return True if the controller acknowledged the command.
    bool backwardM1( uint8_t a_speed );
    /// @brief Sets the main battery minimum with the older one-byte command. Prefer setMainVoltages().
    /// @param a_voltage Raw voltage limit in the controller's command format, see the manual.
    /// @return True if the controller acknowledged the command.
    bool setMinVoltageMainBattery( uint8_t a_voltage );
    /// @brief Sets the main battery maximum with the older one-byte command. Prefer setMainVoltages().
    /// @param a_voltage Raw voltage limit in the controller's command format, see the manual.
    /// @return True if the controller acknowledged the command.
    bool setMaxVoltageMainBattery( uint8_t a_voltage );
    /// @brief Drives M2 forward.
    /// @param a_speed Speed from 0 (stopped) to 127 (full speed).
    /// @return True if the controller acknowledged the command.
    bool forwardM2( uint8_t a_speed );
    /// @brief Drives M2 backward.
    /// @param a_speed Speed from 0 (stopped) to 127 (full speed).
    /// @return True if the controller acknowledged the command.
    bool backwardM2( uint8_t a_speed );
    /// @brief Drives M1 in 7-bit mode.
    /// @param a_speed 0 is full reverse or left, 64 is stop, 127 is full forward or right.
    /// @return True if the controller acknowledged the command.
    bool forwardBackwardM1( uint8_t a_speed );
    /// @brief Drives M2 in 7-bit mode.
    /// @param a_speed 0 is full reverse or left, 64 is stop, 127 is full forward or right.
    /// @return True if the controller acknowledged the command.
    bool forwardBackwardM2( uint8_t a_speed );
    /// @brief Drives both motors forward in mixed mode.
    /// @param a_speed Speed from 0 (stopped) to 127 (full speed).
    /// @return True if the controller acknowledged the command.
    bool forwardMixed( uint8_t a_speed );
    /// @brief Drives both motors backward in mixed mode.
    /// @param a_speed Speed from 0 (stopped) to 127 (full speed).
    /// @return True if the controller acknowledged the command.
    bool backwardMixed( uint8_t a_speed );
    /// @brief Turns right in mixed mode.
    /// @param a_speed Speed from 0 (stopped) to 127 (full speed).
    /// @return True if the controller acknowledged the command.
    bool turnRightMixed( uint8_t a_speed );
    /// @brief Turns left in mixed mode.
    /// @param a_speed Speed from 0 (stopped) to 127 (full speed).
    /// @return True if the controller acknowledged the command.
    bool turnLeftMixed( uint8_t a_speed );
    /// @brief Drives forward or backward in 7-bit mixed mode.
    /// @param a_speed 0 is full reverse or left, 64 is stop, 127 is full forward or right.
    /// @return True if the controller acknowledged the command.
    bool forwardBackwardMixed( uint8_t a_speed );
    /// @brief Turns in 7-bit mixed mode.
    /// @param a_speed 0 is full reverse or left, 64 is stop, 127 is full forward or right.
    /// @return True if the controller acknowledged the command.
    bool leftRightMixed( uint8_t a_speed );
    /// @brief Reads the M1 encoder count and its status byte.
    /// @return The value, or empty if the read failed.
    std::optional< status_value > readEncM1();
    /// @brief Reads the M2 encoder count and its status byte.
    /// @return The value, or empty if the read failed.
    std::optional< status_value > readEncM2();
    /// @brief Sets the M1 encoder count.
    /// @param a_val New encoder count.
    /// @return True if the controller acknowledged the command.
    bool setEncM1( int32_t a_val );
    /// @brief Sets the M2 encoder count.
    /// @param a_val New encoder count.
    /// @return True if the controller acknowledged the command.
    bool setEncM2( int32_t a_val );
    /// @brief Reads the M1 speed in pulses per second and its status byte.
    /// @return The value, or empty if the read failed.
    std::optional< status_value > readSpeedM1();
    /// @brief Reads the M2 speed in pulses per second and its status byte.
    /// @return The value, or empty if the read failed.
    std::optional< status_value > readSpeedM2();
    /// @brief Sets both encoder counts to zero.
    /// @return True if the controller acknowledged the command.
    bool resetEncoders();
    /// @brief Reads the firmware version text.
    /// @return The value, or empty if the read failed.
    std::optional< std::string > readVersion();
    /// @brief Reads the main battery voltage in tenths of a volt.
    /// @return The value, or empty if the read failed.
    std::optional< uint16_t > readMainBatteryVoltage();
    /// @brief Reads the logic battery voltage in tenths of a volt.
    /// @return The value, or empty if the read failed.
    std::optional< uint16_t > readLogicBatteryVoltage();
    /// @brief Sets the logic battery minimum with the older one-byte command. Prefer setLogicVoltages().
    /// @param a_voltage Raw voltage limit in the controller's command format, see the manual.
    /// @return True if the controller acknowledged the command.
    bool setMinVoltageLogicBattery( uint8_t a_voltage );
    /// @brief Sets the logic battery maximum with the older one-byte command. Prefer setLogicVoltages().
    /// @param a_voltage Raw voltage limit in the controller's command format, see the manual.
    /// @return True if the controller acknowledged the command.
    bool setMaxVoltageLogicBattery( uint8_t a_voltage );
    /// @brief Sets the M1 velocity PID in RAM. The gains are stored as fixed point in 1/65536 steps.
    /// @param a_kp Proportional gain.
    /// @param a_ki Integral gain.
    /// @param a_kd Derivative gain.
    /// @param a_qpps Encoder speed at full power, in quadrature pulses per second.
    /// @return True if the controller acknowledged the command.
    bool setM1VelocityPID( float a_kp, float a_ki, float a_kd, uint32_t a_qpps );
    /// @brief Sets the M2 velocity PID in RAM. The gains are stored as fixed point in 1/65536 steps.
    /// @param a_kp Proportional gain.
    /// @param a_ki Integral gain.
    /// @param a_kd Derivative gain.
    /// @param a_qpps Encoder speed at full power, in quadrature pulses per second.
    /// @return True if the controller acknowledged the command.
    bool setM2VelocityPID( float a_kp, float a_ki, float a_kd, uint32_t a_qpps );
    /// @brief Reads the M1 instantaneous speed and its status byte.
    /// @return The value, or empty if the read failed.
    std::optional< status_value > readISpeedM1();
    /// @brief Reads the M2 instantaneous speed and its status byte.
    /// @return The value, or empty if the read failed.
    std::optional< status_value > readISpeedM2();
    /// @brief Sets the M1 duty cycle.
    /// @param a_duty Signed duty cycle sent as two's complement, where 32767 is 100 % forward.
    /// @return True if the controller acknowledged the command.
    bool dutyM1( uint16_t a_duty );
    /// @brief Sets the M2 duty cycle.
    /// @param a_duty Signed duty cycle sent as two's complement, where 32767 is 100 % forward.
    /// @return True if the controller acknowledged the command.
    bool dutyM2( uint16_t a_duty );
    /// @brief Sets both duty cycles in one command.
    /// @param a_duty1 M1 duty cycle, see dutyM1().
    /// @param a_duty2 M2 duty cycle, see dutyM1().
    /// @return True if the controller acknowledged the command.
    bool dutyM1M2( uint16_t a_duty1, uint16_t a_duty2 );
    /// @brief Runs M1 at a signed speed.
    /// @param a_speed Signed speed in pulses per second, sent as two's complement.
    /// @return True if the controller acknowledged the command.
    bool speedM1( uint32_t a_speed );
    /// @brief Runs M2 at a signed speed.
    /// @param a_speed Signed speed in pulses per second, sent as two's complement.
    /// @return True if the controller acknowledged the command.
    bool speedM2( uint32_t a_speed );
    /// @brief Runs both motors at signed speeds in one command.
    /// @param a_speed1 M1 signed speed in pulses per second, sent as two's complement.
    /// @param a_speed2 M2 signed speed in pulses per second, sent as two's complement.
    /// @return True if the controller acknowledged the command.
    bool speedM1M2( uint32_t a_speed1, uint32_t a_speed2 );
    /// @brief Runs M1 at a signed speed with the given acceleration.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @param a_speed Signed speed in pulses per second, sent as two's complement.
    /// @return True if the controller acknowledged the command.
    bool speedAccelM1( uint32_t a_accel, uint32_t a_speed );
    /// @brief Runs M2 at a signed speed with the given acceleration.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @param a_speed Signed speed in pulses per second, sent as two's complement.
    /// @return True if the controller acknowledged the command.
    bool speedAccelM2( uint32_t a_accel, uint32_t a_speed );
    /// @brief Runs both motors at signed speeds with one shared acceleration.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @param a_speed1 M1 signed speed in pulses per second, sent as two's complement.
    /// @param a_speed2 M2 signed speed in pulses per second, sent as two's complement.
    /// @return True if the controller acknowledged the command.
    bool speedAccelM1M2( uint32_t a_accel, uint32_t a_speed1, uint32_t a_speed2 );
    /// @brief Runs M1 at a speed for a distance.
    /// @param a_speed Speed in pulses per second.
    /// @param a_distance Distance in pulses.
    /// @param a_flag 0 adds the command to the buffer, 1 replaces what is running.
    /// @return True if the controller acknowledged the command.
    bool speedDistanceM1( uint32_t a_speed, uint32_t a_distance, uint8_t a_flag = 0 );
    /// @brief Runs M2 at a speed for a distance.
    /// @param a_speed Speed in pulses per second.
    /// @param a_distance Distance in pulses.
    /// @param a_flag 0 adds the command to the buffer, 1 replaces what is running.
    /// @return True if the controller acknowledged the command.
    bool speedDistanceM2( uint32_t a_speed, uint32_t a_distance, uint8_t a_flag = 0 );
    /// @brief Runs both motors at speeds for distances.
    /// @param a_speed1 M1 speed in pulses per second.
    /// @param a_distance1 M1 distance in pulses.
    /// @param a_speed2 M2 speed in pulses per second.
    /// @param a_distance2 M2 distance in pulses.
    /// @param a_flag 0 adds the command to the buffer, 1 replaces what is running.
    /// @return True if the controller acknowledged the command.
    bool speedDistanceM1M2( uint32_t a_speed1, uint32_t a_distance1, uint32_t a_speed2, uint32_t a_distance2, uint8_t a_flag = 0 );
    /// @brief Runs M1 at a speed for a distance with the given acceleration.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @param a_speed Speed in pulses per second.
    /// @param a_distance Distance in pulses.
    /// @param a_flag 0 adds the command to the buffer, 1 replaces what is running.
    /// @return True if the controller acknowledged the command.
    bool speedAccelDistanceM1( uint32_t a_accel, uint32_t a_speed, uint32_t a_distance, uint8_t a_flag = 0 );
    /// @brief Runs M2 at a speed for a distance with the given acceleration.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @param a_speed Speed in pulses per second.
    /// @param a_distance Distance in pulses.
    /// @param a_flag 0 adds the command to the buffer, 1 replaces what is running.
    /// @return True if the controller acknowledged the command.
    bool speedAccelDistanceM2( uint32_t a_accel, uint32_t a_speed, uint32_t a_distance, uint8_t a_flag = 0 );
    /// @brief Runs both motors at speeds for distances with one shared acceleration.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @param a_speed1 M1 speed in pulses per second.
    /// @param a_distance1 M1 distance in pulses.
    /// @param a_speed2 M2 speed in pulses per second.
    /// @param a_distance2 M2 distance in pulses.
    /// @param a_flag 0 adds the command to the buffer, 1 replaces what is running.
    /// @return True if the controller acknowledged the command.
    bool speedAccelDistanceM1M2( uint32_t a_accel, uint32_t a_speed1, uint32_t a_distance1, uint32_t a_speed2, uint32_t a_distance2, uint8_t a_flag = 0 );
    /// @brief Reads how many commands are buffered for each motor.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< uint8_t > > readBuffers();
    /// @brief Reads the PWM duty of each motor, where 32767 is 100 %.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< int16_t > > readPWMs();
    /// @brief Reads the current of each motor in units of 10 mA.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< int16_t > > readCurrents();
    /// @brief Runs both motors at signed speeds with an acceleration for each.
    /// @param a_accel1 M1 acceleration in pulses per second squared.
    /// @param a_speed1 M1 signed speed in pulses per second, sent as two's complement.
    /// @param a_accel2 M2 acceleration in pulses per second squared.
    /// @param a_speed2 M2 signed speed in pulses per second, sent as two's complement.
    /// @return True if the controller acknowledged the command.
    bool speedAccelM1M2Separate( uint32_t a_accel1, uint32_t a_speed1, uint32_t a_accel2, uint32_t a_speed2 );
    /// @brief Runs both motors at speeds for distances with an acceleration for each.
    /// @param a_accel1 M1 acceleration in pulses per second squared.
    /// @param a_speed1 M1 speed in pulses per second.
    /// @param a_distance1 M1 distance in pulses.
    /// @param a_accel2 M2 acceleration in pulses per second squared.
    /// @param a_speed2 M2 speed in pulses per second.
    /// @param a_distance2 M2 distance in pulses.
    /// @param a_flag 0 adds the command to the buffer, 1 replaces what is running.
    /// @return True if the controller acknowledged the command.
    bool speedAccelDistanceM1M2Separate( uint32_t a_accel1, uint32_t a_speed1, uint32_t a_distance1, uint32_t a_accel2, uint32_t a_speed2, uint32_t a_distance2, uint8_t a_flag = 0 );
    /// @brief Ramps M1 to a duty cycle.
    /// @param a_duty Signed duty cycle sent as two's complement, where 32767 is 100 % forward.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @return True if the controller acknowledged the command.
    bool dutyAccelM1( uint16_t a_duty, uint32_t a_accel );
    /// @brief Ramps M2 to a duty cycle.
    /// @param a_duty Signed duty cycle sent as two's complement, where 32767 is 100 % forward.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @return True if the controller acknowledged the command.
    bool dutyAccelM2( uint16_t a_duty, uint32_t a_accel );
    /// @brief Ramps both motors to duty cycles in one command.
    /// @param a_duty1 M1 duty cycle, see dutyM1().
    /// @param a_accel1 M1 acceleration in pulses per second squared.
    /// @param a_duty2 M2 duty cycle, see dutyM1().
    /// @param a_accel2 M2 acceleration in pulses per second squared.
    /// @return True if the controller acknowledged the command.
    bool dutyAccelM1M2( uint16_t a_duty1, uint32_t a_accel1, uint16_t a_duty2, uint32_t a_accel2 );
    /// @brief Reads the M1 velocity PID.
    /// @return The value, or empty if the read failed.
    std::optional< velocity_pid > readM1VelocityPID();
    /// @brief Reads the M2 velocity PID.
    /// @return The value, or empty if the read failed.
    std::optional< velocity_pid > readM2VelocityPID();
    /// @brief Sets the main battery limits.
    /// @param a_min Minimum in tenths of a volt.
    /// @param a_max Maximum in tenths of a volt.
    /// @return True if the controller acknowledged the command.
    bool setMainVoltages( uint16_t a_min, uint16_t a_max );
    /// @brief Sets the logic battery limits.
    /// @param a_min Minimum in tenths of a volt.
    /// @param a_max Maximum in tenths of a volt.
    /// @return True if the controller acknowledged the command.
    bool setLogicVoltages( uint16_t a_min, uint16_t a_max );
    /// @brief Reads the main battery limits in tenths of a volt.
    /// @return The value, or empty if the read failed.
    std::optional< range< uint16_t > > readMinMaxMainVoltages();
    /// @brief Reads the logic battery limits in tenths of a volt.
    /// @return The value, or empty if the read failed.
    std::optional< range< uint16_t > > readMinMaxLogicVoltages();
    /// @brief Sets the M1 position PID in RAM. The gains are stored as fixed point in 1/1024 steps.
    /// @param a_kp Proportional gain.
    /// @param a_ki Integral gain.
    /// @param a_kd Derivative gain.
    /// @param a_kiMax Integral limit.
    /// @param a_deadzone Position deadzone in pulses.
    /// @param a_min Minimum position in pulses.
    /// @param a_max Maximum position in pulses.
    /// @return True if the controller acknowledged the command.
    bool setM1PositionPID( float a_kp, float a_ki, float a_kd, uint32_t a_kiMax, uint32_t a_deadzone, uint32_t a_min, uint32_t a_max );
    /// @brief Sets the M2 position PID in RAM. The gains are stored as fixed point in 1/1024 steps.
    /// @param a_kp Proportional gain.
    /// @param a_ki Integral gain.
    /// @param a_kd Derivative gain.
    /// @param a_kiMax Integral limit.
    /// @param a_deadzone Position deadzone in pulses.
    /// @param a_min Minimum position in pulses.
    /// @param a_max Maximum position in pulses.
    /// @return True if the controller acknowledged the command.
    bool setM2PositionPID( float a_kp, float a_ki, float a_kd, uint32_t a_kiMax, uint32_t a_deadzone, uint32_t a_min, uint32_t a_max );
    /// @brief Reads the M1 position PID.
    /// @return The value, or empty if the read failed.
    std::optional< position_pid > readM1PositionPID();
    /// @brief Reads the M2 position PID.
    /// @return The value, or empty if the read failed.
    std::optional< position_pid > readM2PositionPID();
    /// @brief Moves M1 to a position.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @param a_speed Speed in pulses per second.
    /// @param a_deccel Deceleration in pulses per second squared.
    /// @param a_position Target position in pulses.
    /// @param a_flag 0 adds the command to the buffer, 1 replaces what is running.
    /// @return True if the controller acknowledged the command.
    bool speedAccelDeccelPositionM1( uint32_t a_accel, uint32_t a_speed, uint32_t a_deccel, uint32_t a_position, uint8_t a_flag );
    /// @brief Moves M2 to a position.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @param a_speed Speed in pulses per second.
    /// @param a_deccel Deceleration in pulses per second squared.
    /// @param a_position Target position in pulses.
    /// @param a_flag 0 adds the command to the buffer, 1 replaces what is running.
    /// @return True if the controller acknowledged the command.
    bool speedAccelDeccelPositionM2( uint32_t a_accel, uint32_t a_speed, uint32_t a_deccel, uint32_t a_position, uint8_t a_flag );
    /// @brief Moves both motors to positions in one command.
    /// @param a_accel1 M1 acceleration in pulses per second squared.
    /// @param a_speed1 M1 speed in pulses per second.
    /// @param a_deccel1 M1 deceleration in pulses per second squared.
    /// @param a_position1 M1 target position in pulses.
    /// @param a_accel2 M2 acceleration in pulses per second squared.
    /// @param a_speed2 M2 speed in pulses per second.
    /// @param a_deccel2 M2 deceleration in pulses per second squared.
    /// @param a_position2 M2 target position in pulses.
    /// @param a_flag 0 adds the command to the buffer, 1 replaces what is running.
    /// @return True if the controller acknowledged the command.
    bool speedAccelDeccelPositionM1M2( uint32_t a_accel1, uint32_t a_speed1, uint32_t a_deccel1, uint32_t a_position1, uint32_t a_accel2, uint32_t a_speed2, uint32_t a_deccel2, uint32_t a_position2, uint8_t a_flag );
    /// @brief Sets the M1 default duty acceleration. Values above 655360 were ignored by firmware 4.1.34, even though it acknowledged them.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @param a_deccel Deceleration in pulses per second squared.
    /// @return True if the controller acknowledged the command.
    bool setM1DefaultAccel( uint32_t a_accel, uint32_t a_deccel );
    /// @brief Sets the M2 default duty acceleration. Values above 655360 were ignored by firmware 4.1.34, even though it acknowledged them.
    /// @param a_accel Acceleration in pulses per second squared.
    /// @param a_deccel Deceleration in pulses per second squared.
    /// @return True if the controller acknowledged the command.
    bool setM2DefaultAccel( uint32_t a_accel, uint32_t a_deccel );
    /// @brief Reads the default duty acceleration and deceleration of each motor.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< acceleration > > getDefaultAccels();
    /// @brief Sets the modes of the S3, S4 and S5 pins.
    /// @param a_s3mode S3 pin mode.
    /// @param a_s4mode S4 pin mode.
    /// @param a_s5mode S5 pin mode.
    /// @return True if the controller acknowledged the command.
    bool setPinFunctions( uint8_t a_s3mode, uint8_t a_s4mode, uint8_t a_s5mode );
    /// @brief Reads the modes of the S3, S4 and S5 pins.
    /// @return The value, or empty if the read failed.
    std::optional< pin_functions > getPinFunctions();
    /// @brief Sets the RC and analog deadband.
    /// @param a_min Reverse deadband.
    /// @param a_max Forward deadband.
    /// @return True if the controller acknowledged the command.
    bool setDeadBand( uint8_t a_min, uint8_t a_max );
    /// @brief Reads the RC and analog deadband.
    /// @return The value, or empty if the read failed.
    std::optional< range< uint8_t > > getDeadBand();
    /// @brief Reads both encoder counts.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< uint32_t > > readEncoders();
    /// @brief Reads both instantaneous speeds.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< uint32_t > > readISpeeds();
    /// @brief Restores factory defaults, which also erases tuned PID values.
    /// @return True if the controller acknowledged the command.
    bool restoreDefaults();
    /// @brief Reads the temperature in tenths of a degree Celsius.
    /// @return The value, or empty if the read failed.
    std::optional< uint16_t > readTemp();
    /// @brief Reads the second temperature sensor in tenths of a degree Celsius. Only available on some models.
    /// @return The value, or empty if the read failed.
    std::optional< uint16_t > readTemp2();
    /// @brief Reads the status bit field.
    /// @return The value, or empty if the read failed.
    std::optional< uint32_t > readError();
    /// @brief Reads the encoder mode of each motor.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< uint8_t > > readEncoderModes();
    /// @brief Sets the M1 encoder mode.
    /// @param a_mode Mode number, see the manual.
    /// @return True if the controller acknowledged the command.
    bool setM1EncoderMode( uint8_t a_mode );
    /// @brief Sets the M2 encoder mode.
    /// @param a_mode Mode number, see the manual.
    /// @return True if the controller acknowledged the command.
    bool setM2EncoderMode( uint8_t a_mode );
    /// @brief Saves the current settings to flash.
    /// @return True if the controller acknowledged the command.
    bool writeNVM();
    /// @brief Reloads the settings from flash, discarding unsaved changes.
    /// @return True if the controller acknowledged the command.
    bool readNVM();
    /// @brief Sets the configuration, including control mode, baud rate and packet address.
    /// @param a_config Configuration bit field, see the manual.
    /// @return True if the controller acknowledged the command.
    bool setConfig( uint16_t a_config );
    /// @brief Reads the configuration bit field.
    /// @return The value, or empty if the read failed.
    std::optional< uint16_t > getConfig();
    /// @brief Sets the M1 current limit.
    /// @param a_max Limit in units of 10 mA.
    /// @return True if the controller acknowledged the command.
    bool setM1MaxCurrent( uint32_t a_max );
    /// @brief Sets the M2 current limit.
    /// @param a_max Limit in units of 10 mA.
    /// @return True if the controller acknowledged the command.
    bool setM2MaxCurrent( uint32_t a_max );
    /// @brief Reads the M1 current limit in units of 10 mA.
    /// @return The value, or empty if the read failed.
    std::optional< uint32_t > readM1MaxCurrent();
    /// @brief Reads the M2 current limit in units of 10 mA.
    /// @return The value, or empty if the read failed.
    std::optional< uint32_t > readM2MaxCurrent();
    /// @brief Sets the PWM drive mode.
    /// @param a_mode Mode number, see the manual.
    /// @return True if the controller acknowledged the command.
    bool setPWMMode( uint8_t a_mode );
    /// @brief Reads the PWM drive mode.
    /// @return The value, or empty if the read failed.
    std::optional< uint8_t > getPWMMode();
    /// @brief Sets the serial timeout.
    /// @param a_timeout Timeout in tenths of a second, 0 disables it.
    /// @return True if the controller acknowledged the command.
    bool setTimeout( uint8_t a_timeout );
    /// @brief Reads the serial timeout in tenths of a second.
    /// @return The value, or empty if the read failed.
    std::optional< uint8_t > getTimeout();
    /// @brief Sets the M1 default speed.
    /// @param a_speed Default speed.
    /// @return True if the controller acknowledged the command.
    bool setM1DefaultSpeed( uint16_t a_speed );
    /// @brief Sets the M2 default speed.
    /// @param a_speed Default speed.
    /// @return True if the controller acknowledged the command.
    bool setM2DefaultSpeed( uint16_t a_speed );
    /// @brief Reads the default speed of each motor.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< uint16_t > > getDefaultSpeeds();
    /// @brief Sets the CTRL output modes.
    /// @param a_ctrl1 CTRL1 mode.
    /// @param a_ctrl2 CTRL2 mode.
    /// @return True if the controller acknowledged the command.
    bool setCtrlModes( uint8_t a_ctrl1, uint8_t a_ctrl2 );
    /// @brief Reads the CTRL output modes.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< uint8_t > > getCtrlModes();
    /// @brief Sets the CTRL1 output value.
    /// @param a_value Output value.
    /// @return True if the controller acknowledged the command.
    bool setCtrl1( uint16_t a_value );
    /// @brief Sets the CTRL2 output value.
    /// @param a_value Output value.
    /// @return True if the controller acknowledged the command.
    bool setCtrl2( uint16_t a_value );
    /// @brief Reads the CTRL output values.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< uint16_t > > getCtrls();
    /// @brief Sets the M1 auto duty and timeout.
    /// @param a_duty Duty value.
    /// @param a_timeout Timeout.
    /// @return True if the controller acknowledged the command.
    bool setAuto1( uint16_t a_duty, uint32_t a_timeout );
    /// @brief Sets the M2 auto duty and timeout.
    /// @param a_duty Duty value.
    /// @param a_timeout Timeout.
    /// @return True if the controller acknowledged the command.
    bool setAuto2( uint16_t a_duty, uint32_t a_timeout );
    /// @brief Reads the auto duty and timeout of each motor.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< auto_setting > > getAutos();
    /// @brief Reads both speeds as signed pulses per second.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< int32_t > > readSpeeds();
    /// @brief Sets the speed error limits.
    /// @param a_limit1 M1 limit.
    /// @param a_limit2 M2 limit.
    /// @return True if the controller acknowledged the command.
    bool setSpeedErrorLimits( uint32_t a_limit1, uint32_t a_limit2 );
    /// @brief Reads the speed error limits.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< uint32_t > > getSpeedErrorLimits();
    /// @brief Reads the current speed errors.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< int32_t > > readSpeedErrors();
    /// @brief Sets the position error limits.
    /// @param a_limit1 M1 limit.
    /// @param a_limit2 M2 limit.
    /// @return True if the controller acknowledged the command.
    bool setPosErrorLimits( uint32_t a_limit1, uint32_t a_limit2 );
    /// @brief Reads the position error limits.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< uint32_t > > getPosErrorLimits();
    /// @brief Reads the current position errors.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< int32_t > > readPosErrors();
    /// @brief Sets the battery reading offsets.
    /// @param a_main Main battery offset in tenths of a volt.
    /// @param a_logic Logic battery offset in tenths of a volt.
    /// @return True if the controller acknowledged the command.
    bool setVoltageOffsets( int8_t a_main, int8_t a_logic );
    /// @brief Reads the battery reading offsets.
    /// @return The value, or empty if the read failed.
    std::optional< voltage_offsets > getVoltageOffsets();
    /// @brief Sets the current blanking of both motors. Firmware 4.1.34 rounds the value and may apply it to both.
    /// @param a_blanking1 M1 blanking as a fraction of 65536.
    /// @param a_blanking2 M2 blanking as a fraction of 65536.
    /// @return True if the controller acknowledged the command.
    bool setCurrentBlanking( uint16_t a_blanking1, uint16_t a_blanking2 );
    /// @brief Reads the current blanking of each motor as a fraction of 65536.
    /// @return The value, or empty if the read failed.
    std::optional< motor_values< uint16_t > > getCurrentBlanking();

    /// @brief Gets the library version.
    /// @return The library version number.
    static int16_t libraryVersion() { return 16; }

    /// @brief Checks for received data.
    /// @return The number of bytes waiting on the serial port.
    [[nodiscard]] int available() const;
    /// @brief Opens and configures the serial port. Does nothing if it is already open.
    /// @param a_speed Baud rate constant such as B115200, not a plain number.
    void begin( speed_t a_speed );
    /// @brief Closes the serial port.
    void end();
    /// @brief Reads one byte from the serial port, blocking until it arrives.
    /// @return The byte, or 0 if the port is closed or the read fails.
    [[nodiscard]] uint8_t read() const;
    /// @brief Reads one byte with a timeout.
    /// @param a_timeout Timeout in milliseconds.
    /// @return The byte, or -1 on timeout.
    [[nodiscard]] int16_t read( uint32_t a_timeout ) const;
    /// @brief Writes one byte to the serial port.
    /// @param a_byte Byte to write.
    /// @return The result of the write call, or 0 if the port is closed.
    virtual size_t write( uint8_t a_byte );
    /// @brief Discards unsent and unread serial data.
    virtual void flush();
    /// @brief Reads and discards everything waiting on the serial port.
    void clear() const;

protected:
    /// @brief Resets the running CRC.
    void crcClear();
    /// @brief Adds a byte to the running CRC.
    /// @param a_data Byte to add.
    void crcUpdate( uint8_t a_data );
    /// @brief Gets the running CRC.
    /// @return The running CRC.
    [[nodiscard]] uint16_t crcGet() const;
    /// @brief Sends a byte and adds it to the running CRC.
    /// @param a_byte Byte to send.
    void sendByte( uint8_t a_byte );
    /// @brief Clears the serial buffers and sends the address and a read command.
    /// @param a_cmd Command to send.
    void sendCommand( Command a_cmd );
    /// @brief Sends a complete write packet with its CRC.
    /// @param a_cmd Command to send.
    /// @param a_payload Bytes after the command.
    void sendPacket( Command a_cmd, std::span< const uint8_t > a_payload );
    /// @brief Sends a write packet, retrying until it is acknowledged.
    /// @param a_cmd Command to send.
    /// @param a_payload Bytes after the command.
    /// @return True if the controller acknowledged the command.
    bool writeN( Command a_cmd, std::span< const uint8_t > a_payload = {} );
    template< size_t Count >
    /// @brief Runs a read command with a fixed reply size.
    /// @tparam Count Reply size in bytes, without the CRC.
    /// @param a_cmd Command to send.
    /// @param a_retryBadCrc Whether a CRC mismatch is retried.
    /// @return The value, or empty if the read failed.
    std::optional< std::array< uint8_t, Count > > readArray( Command a_cmd, bool a_retryBadCrc = true );
    /// @brief Reads a 32-bit value followed by a status byte.
    /// @param a_cmd Command to send.
    /// @return The value, or empty if the read failed.
    std::optional< status_value > readStatusValue( Command a_cmd );
    /// @brief Reads a velocity PID reply.
    /// @param a_cmd Command to send.
    /// @return The value, or empty if the read failed.
    std::optional< velocity_pid > readVelocityPID( Command a_cmd );
    /// @brief Reads a position PID reply.
    /// @param a_cmd Command to send.
    /// @return The value, or empty if the read failed.
    std::optional< position_pid > readPositionPID( Command a_cmd );
    /// @brief Runs a read command and checks the CRC of its reply.
    /// @param a_cmd Command to send.
    /// @param a_data Receives the reply, and sets how many bytes are expected.
    /// @param a_retryBadCrc Whether a CRC mismatch is retried.
    /// @return True on a complete reply with a valid CRC.
    bool readBytes( Command a_cmd, std::span< uint8_t > a_data, bool a_retryBadCrc = true );
    /// @brief Sends a flash command once and waits up to 2 seconds for its acknowledge.
    /// @param a_cmd Command to send.
    /// @param a_payload Bytes after the command.
    /// @return True if the controller acknowledged the command.
    bool writeSlow( Command a_cmd, std::span< const uint8_t > a_payload = {} );

private:
    std::string m_portName;

    uint16_t m_crc{ 0 };
    uint32_t m_timeout;
    uint8_t m_address;
    int m_serial{ -1 };
};

#endif
