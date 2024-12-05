# TMC2240_ESP32

Stepper motor controller using the TMC2240 motor driver and the ESP32

# TODO

- ~~Relationship of decceleration time drift with target RPM~~
- ~~Expose stall thresholds~~ (_not required_)
- ~~Implement velocity mode~~
- Implement inverse time mode
- Rework return message
- Document wiring
- LED status indication
- ~~Torque - Current (page 115) - T = KTi~~ (Current value returned is the set value. Does not change dynamically)
- Document Max RPM by microstepping
- Detect driver disconnection error _(2-12-2024 - Currently detected as a "stall" event)_
- Inverse motor direction (page 81 of [datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/tmc2240_datasheet.pdf))
- Set current precision (page 86)
- Read SG & SG4 value and relate to current to calculate torque (page 116, 122)
- Release axis to use "Free Wheeling" (page 118)
- ~~Homing~~
- Limit Switch
- ~~Multi Axis Control~~
