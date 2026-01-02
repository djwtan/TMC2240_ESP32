#ifndef REGISTERS_H
#define REGISTERS_H

#include <cstdint>

namespace S_Reg {
constexpr uint8_t TARGET_POSITION             = 0x00;
constexpr uint8_t TARGET_RPM                  = 0x01;
constexpr uint8_t MOVE                        = 0x02;
constexpr uint8_t TEMPERATURE                 = 0x03;
constexpr uint8_t DRV_STATUS                  = 0x04;
constexpr uint8_t MOTOR_STATUS                = 0x05;
constexpr uint8_t EMERGENCY_STOP              = 0x06;
constexpr uint8_t STOP_VELOCITY               = 0x07;
constexpr uint8_t ENABLE_STEPPER              = 0x08;
constexpr uint8_t OPERATION_MODE              = 0x09;
constexpr uint8_t ACEL_TIME                   = 0x0A;
constexpr uint8_t DECEL_TIME                  = 0x0B;
constexpr uint8_t CURRENT_RPM                 = 0X0C;
constexpr uint8_t CURRENT_POS                 = 0x0D;
constexpr uint8_t ACTUAL_ACCELERATION_TIME    = 0x0E;
constexpr uint8_t ACTUAL_DECCELERATION_TIME   = 0x0F;
constexpr uint8_t STOP_ON_STALL               = 0x10;
constexpr uint8_t MICROSTEPPING               = 0x11;
constexpr uint8_t RUNNING_CURRENT             = 0x12;
constexpr uint8_t HOLDING_CURRENT             = 0x13;
constexpr uint8_t DISABLE_STEPPER             = 0x14;
constexpr uint8_t STALL_VALUE                 = 0x15;
constexpr uint8_t HOMING_METHOD               = 0X16;
constexpr uint8_t HOMING_SENSOR_TRIGGER_VALUE = 0X17;
constexpr uint8_t REQUEST_HOMING              = 0X18;
constexpr uint8_t HOMED                       = 0x19;
constexpr uint8_t POSITIONING_MODE            = 0x1A;
} // namespace S_Reg

#endif