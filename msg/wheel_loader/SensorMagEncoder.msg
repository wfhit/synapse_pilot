# Magnetic Rotary Encoder Sensor (AS5600 compatible)
# 12-bit contactless magnetic rotary position sensor

uint64 timestamp          # time since system start (microseconds)

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 angle             # absolute angle position in radians [0, 2*PI]
uint16 raw_angle          # raw 12-bit angle value [0, 4095]

uint8 magnet_detected     # 0 = magnet not detected, 1 = magnet detected
uint8 magnet_too_strong   # 0 = normal magnet strength, 1 = magnet too strong
uint8 magnet_too_weak     # 0 = normal magnet strength, 1 = magnet too weak

uint16 magnitude          # magnitude value (0-4095) indicating magnetic field strength
uint8 automatic_gain_control  # AGC value (0-255) for gain adjustment

uint32 error_count        # number of communication errors

uint8 ORB_QUEUE_LENGTH = 4
