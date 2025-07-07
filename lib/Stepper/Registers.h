#ifndef REGISTERS_H
#define REGISTERS_H

    #define REG_TARGET_POSITION             0x00
    #define REG_TARGET_RPM                  0x01
    #define REG_MOVE                        0x02
    #define REG_TEMPERATURE                 0x03
    #define REG_DRV_STATUS                  0x04
    #define REG_MOTOR_STATUS                0x05
    #define REG_EMERGENCY_STOP              0x06
    #define REG_STOP_VELOCITY               0x07
    #define REG_ENABLE_STEPPER              0x08
    #define REG_OPERATION_MODE              0x09
    #define REG_ACEL_TIME                   0x0A
    #define REG_DECEL_TIME                  0x0B
    #define REG_CURRENT_RPM                 0X0C
    #define REG_CURRENT_POS                 0x0D
    #define REG_ACTUAL_ACCELERATION_TIME    0x0E
    #define REG_ACTUAL_DECCELERATION_TIME   0x0F
    #define REG_STOP_ON_STALL               0x10
    #define REG_MICROSTEPPING               0x11
    #define REG_RUNNING_CURRENT             0x12
    #define REG_HOLDING_CURRENT_PERCENTAGE  0x13
    #define REG_DISABLE_STEPPER             0x14
    #define REG_STALL_VALUE                 0x15
    #define REG_HOMING_METHOD               0X16
    #define REG_HOMING_SENSOR_TRIGGER_VALUE 0X17
    #define REG_REQUEST_HOMING              0X18
    #define REG_HOMED                       0x19
    #define REG_POSITIONING_MODE            0x1A

#endif