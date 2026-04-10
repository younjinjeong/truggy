# Serial Protocol

Binary protocol between Jetson Nano and Arduino Uno over USB serial at 115200 baud, 8N1.

## Command Packet (Jetson → Arduino)

**8 bytes, sent at 50 Hz**

```
Byte   Field         Type     Range              Description
─────────────────────────────────────────────────────────────
 0     sync          uint8    0xAA               Start marker
 1-2   steering      int16    [-10000, 10000]    float * 10000
 3-4   throttle      int16    [-10000, 10000]    float * 10000
 5     flags         uint8    bitmask            Control flags
 6     crc8          uint8    [0, 255]           CRC-8 over bytes 1-5
 7     end           uint8    0x55               End marker
```

### Flags Byte

```
Bit 0: arm        (1 = armed, 0 = disarmed → neutral output)
Bit 1: estop      (1 = emergency stop)
Bit 2-7: reserved
```

### Steering/Throttle Encoding

```c
// Jetson side (encode)
int16_t steer_raw = (int16_t)(steering_float * 10000.0f);  // [-0.99, 0.99] → [-9900, 9900]
int16_t throt_raw = (int16_t)(throttle_float * 10000.0f);

// Arduino side (decode)
float steering = (float)steer_raw / 10000.0f;
int pwm_us = 1500 + (int)(steering * 500.0f);  // → [1000, 2000] us
```

## Telemetry Packet (Arduino → Jetson)

**24 bytes, sent at 100 Hz**

```
Byte   Field         Type     Scale     Description
───────────────────────────────────────────────────────────────
 0     sync          uint8    —         0xBB start marker
 1-2   qw            int16    * 10000   Quaternion w
 3-4   qx            int16    * 10000   Quaternion x
 5-6   qy            int16    * 10000   Quaternion y
 7-8   qz            int16    * 10000   Quaternion z
 9-10  ax            int16    * 1000    Accel X (m/s^2)
 11-12 ay            int16    * 1000    Accel Y (m/s^2)
 13-14 az            int16    * 1000    Accel Z (m/s^2)
 15-16 gx            int16    * 1000    Gyro X (rad/s)
 17-18 gy            int16    * 1000    Gyro Y (rad/s)
 19-20 gz            int16    * 1000    Gyro Z (rad/s)
 21-22 wheel_speeds  2x uint8 * 100    L/R wheel speed (rev/s) — see note
 23    crc8          uint8    —         CRC-8 over bytes 1-22
```

**Note on wheel speeds (bytes 21-22):** These are 2x int16 packed into 4 bytes:
- Bytes 21-22: left wheel (int16, speed_rps * 100)
- Actually the telemetry is 24 bytes with the following corrected layout:

```
Byte   Field         Type     Scale     Description
───────────────────────────────────────────────────────────────
 0     sync          uint8    —         0xBB
 1-2   qw            int16    * 10000   Quaternion w
 3-4   qx            int16    * 10000   Quaternion x
 5-6   qy            int16    * 10000   Quaternion y
 7-8   qz            int16    * 10000   Quaternion z
 9-10  ax            int16    * 1000    Accel X (m/s^2)
11-12  ay            int16    * 1000    Accel Y (m/s^2)
13-14  az            int16    * 1000    Accel Z (m/s^2)
15-16  gx            int16    * 1000    Gyro X (rad/s)
17-18  gy            int16    * 1000    Gyro Y (rad/s)
19-20  gz            int16    * 1000    Gyro Z (rad/s)
21-22  wheel_L       int16    * 100     Left wheel (rev/s)
  23   crc8          uint8    —         CRC-8 over bytes 1-22
```

Total: 24 bytes (1 sync + 11 x int16 + 1 CRC = 1 + 22 + 1 = 24)

## CRC-8 (Dallas/Maxim)

Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)

```c
uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}
```

**Same implementation must be used on both Arduino and Jetson.**

## Byte Ordering

All multi-byte values are **big-endian** (network byte order):
```c
// Encode int16 to bytes
buf[0] = (value >> 8) & 0xFF;  // high byte
buf[1] = value & 0xFF;         // low byte

// Decode bytes to int16
int16_t value = (int16_t)((buf[0] << 8) | buf[1]);
```

## Bandwidth

```
Telemetry: 24 bytes * 100 Hz = 2,400 bytes/s
Command:    8 bytes *  50 Hz =   400 bytes/s
Total:                         2,800 bytes/s

USB Serial at 115200 baud ≈ 11,520 bytes/s
Utilization: 24.3% — well within budget
```

## Parsing Strategy (Jetson Side)

1. Read bytes into circular buffer
2. Scan for sync byte (0xBB for telemetry)
3. If 24 bytes available after sync:
   - Read 23 payload bytes
   - Compute CRC-8 over bytes 1-22
   - If CRC matches byte 23: valid packet, consume
   - If CRC mismatch: discard sync byte, scan for next

## Watchdog (Arduino Side)

- Track timestamp of last valid command packet
- If no valid command received for 250 ms:
  - Set throttle PWM to neutral (1500 us)
  - Keep steering at last known position
  - Set run-stop relay HIGH (stop)
- Resume normal operation when valid command received

## Timing Diagram

```
Arduino:  |--IMU read--|--encode telem--|--send 24B--|--parse cmd--|--PWM write--|
          |<-------------- 10 ms (100 Hz) ---------------------------------------->|

Jetson:   |--parse telem--|--write bus--|--read bus--|--encode cmd--|--send 8B--|
          |<--------- varies, driven by serial data arrival ---------------------->|
```
