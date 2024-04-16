# Robotik Arduino Sources

> This repository contains the sources for the controller used by team __Kraft auf Fläche__

## Development process

### Tools

- `PlatformIO IDE` for vscode

### Hardware 

> There are two supported platforms (technically), preferred is the ESP32-based one.
- Platforms:
    - __ESP32 Wemos D1 Mini (MCU)__
        - Custom kieblitz board (schematic included)
            - Quadruple tri-state buffer gate
    - __ATMega2560 (MCU)__
        - Dynamixel Shield for Arduino
- __2x Dynamixel AX12W__
- __Sensors connected with I2C Bus__
    - 2x VL53L0X ToF Sensor
    - 6-axis inertial measurement unit (IMU)


### Code quality assurance

```bash
# Static code analysis
pio check

# run unit tests (for UNO)
pio test -e unotest -v

# run unit tests (for kieblitz)
pio test -e wemos_d1_mini32

# ci commands
pio ci --board=uno --lib="." --exclude="src/test" .
pio ci --board=megaatmega2560 --lib="." --exclude="src/test" .
pio ci --board=wemos_d1_mini32 --lib="." .
```
