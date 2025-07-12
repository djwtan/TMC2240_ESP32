#ifndef TMC2240_REGISTERS_H
#define TMC2240_REGISTERS_H

namespace TMC2240_Registers {
constexpr uint8_t CHOPCONF      = 0x6C;
constexpr uint8_t IHOLD_IRUN    = 0x10;
constexpr uint8_t TPOWERDOWN    = 0x11;
constexpr uint8_t TPWMTHRS      = 0x13;
constexpr uint8_t GCONF         = 0x00;
constexpr uint8_t GSTAT         = 0x01;
constexpr uint8_t PWMCONF       = 0x70;
constexpr uint8_t SG_RESULT_IND = 0x75;
constexpr uint8_t TEMPERATURE   = 0x51;
} // namespace TMC2240_Registers

#endif