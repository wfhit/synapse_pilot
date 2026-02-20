# Wheel Loader Board Interface Reference

Hardware interface allocation for the 3 wheel loader boards.
Last updated: 2026-02-20

---

## CUAV X7Plus-WL (Main Coordinator)

### UART / Serial Ports

| MCU UART | ttyS       | Connector | Baud    | Usage                              |
|----------|------------|-----------|---------|------------------------------------|
| USART1   | /dev/ttyS0 | GPS1      | 38400   | Primary GPS (NMEA/UBX)             |
| USART2   | /dev/ttyS1 | TELEM1    | 921600  | VLA proxy (`VLA_CONFIG=101`)       |
| UART4    | /dev/ttyS2 | GPS2      | 38400   | Second GPS                         |
| USART6   | /dev/ttyS3 | TELEM2    | 57600   | RC input (RadioMaster TX16S, CRSF/SBUS) |
| UART7    | /dev/ttyS4 | DSU7      | 115200  | NSH debug console (SERIAL_CONSOLE) |
| UART8    | /dev/ttyS5 | RC port   | 57600   | spare                              |
| WK2132 ch0 | /dev/ttyS10 | UBR3   | 921600  | Front NXT uORB bridge (`UORB_BR_F_PORT=10`) |
| WK2132 ch1 | /dev/ttyS11 | UBR4   | 921600  | Rear NXT uORB bridge (`UORB_BR_R_PORT=11`)  |

### I2C Buses

| MCU I2C | Pins        | Board connector | Usage                        |
|---------|-------------|-----------------|------------------------------|
| I2C1    | PB8/PB9     | GPS1 port       | Primary GPS compass          |
| I2C2    | PF0/PF1     | GPS2 port       | Second GPS compass           |
| I2C3    | PH7/PH8     | I2C3 connector  | WK2132 I2C-UART bridge       |
| I2C4    | PF14/PF15   | I2C4 connector  | LED light driver             |

### GPIO

| Pin  | Usage                                      |
|------|--------------------------------------------|
| PE0  | Nav lamp state signal (STATUS_LAMP_STATE)  |
| PE1  | Nav lamp illumination (STATUS_LAMP_LIGHT)  |
| PA8  | IMU heater output                          |
| PH2  | CAN1 silent mode control                   |
| PH3  | CAN2 silent mode control                   |

### USB

| Interface | Usage                                                      |
|-----------|------------------------------------------------------------|
| USB CDC ACM | Auto-detect: `\r\r\r` → NSH, MAVLink heartbeat → MAVLink |
| `SYS_USB_AUTO=1` | Auto-detect mode                                  |

### CAN

| Bus  | Usage                    |
|------|--------------------------|
| CAN1 | DroneCAN devices         |

---

## NXT-Dual-WL-Front (Front Actuator Controller)

### UART / Serial Ports

| MCU UART | ttyS       | Connector | Baud    | Usage                                        |
|----------|------------|-----------|---------|----------------------------------------------|
| UART8    | /dev/ttyS0 | SERIAL_CONSOLE | 115200 | NSH debug console                       |
| USART1   | /dev/ttyS1 | —         | 921600  | uORB UART proxy to main board (`UORB_PX_UART=1`) |

Disabled (pins repurposed):
- UART4 (PA0/PA1) — not used
- UART5 (PB12/PB13) — repurposed as quadrature encoder 1 (A/B phases)
- UART7 (PE7) — repurposed as DRV8701 H-bridge enable GPIO
- USART2 (PD5/PD6) — repurposed as quadrature encoder 2 (A/B phases)
- USART3 — not used, disabled
- USART6 — not connected by board design

### I2C Buses

| MCU I2C | Usage                              |
|---------|------------------------------------|
| I2C1    | AS5600 magnetic encoder (tilt/bucket position) |

### GPIO

| Pin        | Usage                                      |
|------------|--------------------------------------------|
| PB12/PB13  | Quadrature encoder 1 A/B (front wheel)     |
| PD5/PD6    | Quadrature encoder 2 A/B (bucket/tilt)     |
| PE7        | DRV8701 H-bridge enable                    |
| PE13       | DRV8701 right direction                    |
| PE14       | DRV8701 left direction                     |
| PB10       | Bucket dump limit switch 1 (PWM5)          |
| PB11       | Bucket load limit switch 1 (PWM6)          |
| PB0        | Bucket load limit switch 2 (PWM7)          |
| PB1        | Bucket dump limit switch 2 (PWM8)          |
| PD14       | PB12 inverter control (SN74LVC1G86DCKR)    |
| PB5        | SPL I2C address select                     |

### PWM

| Channels | Usage                                    |
|----------|------------------------------------------|
| PWM1-2   | DRV8701 H-bridge PWM signals             |
| PWM3-4   | DRV8701 direction signals (GPIO mode)    |
| PWM5-8   | Limit switch GPIO inputs                 |

### USB

| Interface | Usage                                                      |
|-----------|------------------------------------------------------------|
| USB CDC ACM | Auto-detect: `\r\r\r` → NSH, MAVLink heartbeat → MAVLink |
| `SYS_USB_AUTO=1` | Auto-detect mode                                  |

---

## NXT-Dual-WL-Rear (Rear Actuator Controller)

### UART / Serial Ports

| MCU UART | ttyS       | Connector | Baud    | Usage                                        |
|----------|------------|-----------|---------|----------------------------------------------|
| UART5    | /dev/ttyS0 | SRV1      | 115200  | ST3215 smart servo (steering)                |
| UART8    | /dev/ttyS1 | SERIAL_CONSOLE | 115200 | NSH debug console                       |
| USART1   | /dev/ttyS2 | —         | 921600  | uORB UART proxy to main board (`UORB_PX_UART=2`) |

Disabled (pins repurposed):
- UART4 (PA0/PA1) — not used
- UART7 (PE7) — repurposed as DRV8701 H-bridge enable GPIO
- USART2 (PD5/PD6) — repurposed as quadrature encoder (A/B phases)
- USART3 — not used, disabled
- USART6 — not connected by board design

### I2C Buses

| MCU I2C | Usage                              |
|---------|------------------------------------|
| I2C1    | AS5600 magnetic encoder (boom position) |

### GPIO

| Pin        | Usage                                      |
|------------|--------------------------------------------|
| PD5/PD6    | Quadrature encoder A/B (rear wheel)        |
| PE7        | DRV8701 H-bridge enable                    |
| PE13       | DRV8701 right direction                    |
| PE14       | DRV8701 left direction                     |
| PB10       | Steering right limit switch (PWM5)         |
| PB11       | Steering left limit switch (PWM6)          |
| PB0        | Boom down limit switch (PWM7)              |
| PB1        | Boom up limit switch (PWM8)                |
| PA4        | Load lamp left                             |
| PC1        | Load lamp right                            |
| PC0        | Load lamp GND (open drain)                 |
| PB2        | Driver lamp left                           |
| PB4        | Driver lamp right                          |
| PB3        | Driver lamp GND (open drain)               |
| PB5        | SPL I2C address select                     |

### PWM

| Channels | Usage                                    |
|----------|------------------------------------------|
| PWM1-2   | DRV8701 H-bridge PWM signals             |
| PWM3-4   | DRV8701 direction signals (GPIO mode)    |
| PWM5-8   | Limit switch GPIO inputs                 |

### USB

| Interface | Usage                                                      |
|-----------|------------------------------------------------------------|
| USB CDC ACM | Auto-detect: `\r\r\r` → NSH, MAVLink heartbeat → MAVLink |
| `SYS_USB_AUTO=1` | Auto-detect mode                                  |
