# Hardware

## System Overview

```
┌────────────────────────────────────────────────────────┐
│                   TLR 1/8 Truggy                        │
│                                                         │
│    ┌──────────────────────────┐                         │
│    │      ZED Camera          │  720p stereo, 30fps     │
│    │  ┌───┐  120mm  ┌───┐    │  10-15° downward tilt   │
│    │  │ L │         │ R │    │  USB 3.0 → Jetson        │
│    │  └───┘         └───┘    │                          │
│    └──────────┬───────────────┘                         │
│               │ shock tower mount                       │
│    ┌──────────┴───────────────┐                         │
│    │    Jetson Nano 2GB       │  JetPack 4.6            │
│    │    (main computer)       │  CUDA 10.2, sm_53       │
│    │                          │  128 CUDA cores          │
│    └──────────┬───────────────┘                         │
│               │ USB serial                              │
│    ┌──────────┴───────────────┐                         │
│    │    Arduino Uno           │  ATmega328P, 16 MHz     │
│    │    (sensor/actuator hub) │  2KB SRAM               │
│    └──────────┬───────────────┘                         │
│          ┌────┼────┬────┐                               │
│          │    │    │    │                                │
│        IMU  Enc  Servo ESC                              │
│      BNO085 x2   PWM  PWM                              │
└────────────────────────────────────────────────────────┘
```

## Jetson Nano 2GB

| Spec | Value |
|------|-------|
| SoC | Tegra X1 (Maxwell) |
| CPU | 4x ARM Cortex-A57 @ 1.43 GHz |
| GPU | 128 CUDA cores, sm_53 |
| RAM | 2 GB LPDDR4 (shared CPU/GPU) |
| Storage | microSD (64 GB+ recommended) |
| USB | 1x USB 3.0, 2x USB 2.0 |
| Power | 5V/3A barrel jack or USB-C |
| OS | JetPack 4.6 (Ubuntu 18.04, CUDA 10.2, TensorRT 8.x) |

### Memory Budget

| Component | GPU Memory | CPU Memory |
|-----------|-----------|------------|
| Kernel + System | ~300 MB | ~200 MB |
| ZED SDK (720p) | ~400 MB | ~100 MB |
| TensorRT Engine | ~50 MB | ~20 MB |
| MPPI arrays | ~2 MB | ~1 MB |
| Costmap (double-buf) | ~0.2 MB | — |
| curand state | ~1 MB | — |
| **Total** | **~753 MB** | **~321 MB** |
| **Available** | 2048 MB | 2048 MB |
| **Headroom** | ~1295 MB | ~1727 MB |

### Power Management

```bash
# Required for consistent performance
sudo nvpmodel -m 0        # MAXN mode (all cores, max clocks)
sudo jetson_clocks         # Lock GPU/CPU/EMC clocks
```

### Thermal

Sustained load will cause thermal throttling. Required:
- Heatsink with fan (active cooling)
- Monitor with `tegrastats`

## Jetson Orin Nano Super (Future Upgrade)

| Spec | Value |
|------|-------|
| GPU | 1024 CUDA cores, sm_87, Tensor Cores |
| RAM | 8 GB LPDDR5 |
| CUDA | 12.x |
| TensorRT | 10.x with FP16 |
| JetPack | 6.x |

## Arduino Uno

| Spec | Value |
|------|-------|
| MCU | ATmega328P, 16 MHz |
| SRAM | 2 KB (keep usage < 1.5 KB) |
| Flash | 32 KB |
| ADC | 10-bit, 6 channels |
| PWM | 6 pins (we use D9, D10) |
| Interrupts | 2 external (D2, D3) |
| Serial | 115200 baud via USB |

### Pin Assignment

```
Arduino Uno Pin Map:
┌─────────────────────────────────────────┐
│ Pin    │ Function              │ Notes   │
├─────────────────────────────────────────┤
│ A4     │ I2C SDA               │ BNO085  │
│ A5     │ I2C SCL               │ BNO085  │
│ D2     │ INT0 — L wheel encoder│ ISR     │
│ D3     │ INT1 — R wheel encoder│ ISR     │
│ D7     │ Run-stop relay        │ OUTPUT  │
│ D9     │ Steering servo PWM    │ Servo   │
│ D10    │ ESC/throttle PWM      │ Servo   │
│ USB    │ Serial 115200 baud    │ Jetson  │
└─────────────────────────────────────────┘
```

## ZED Stereo Camera

| Spec | Value |
|------|-------|
| Resolution | HD720 (1280x720) @ 30 fps |
| Baseline | 120 mm |
| Depth Range | 0.3 — 20 m |
| Depth Mode | PERFORMANCE (Nano) / NEURAL (Orin) |
| VIO | Built-in positional tracking (6DOF) |
| Interface | USB 3.0 |
| SDK Memory | ~400 MB GPU |

### Mounting

- Mounted on shock tower with 3D-printed bracket
- 10-15 degrees downward tilt
- Vibration damping bushings between mount and bracket
- Cable routed along chassis to prevent snag

## BNO085 IMU

| Spec | Value |
|------|-------|
| Interface | I2C @ 400 kHz, address 0x4A |
| Update Rate | 100 Hz |
| Outputs | Quaternion, linear acceleration, angular velocity |
| Fusion | On-chip sensor fusion (BNO085 handles calibration) |
| Library | Adafruit_BNO08x (SH2 protocol) |

### Data Output (per sample)
- Quaternion: qw, qx, qy, qz (rotation)
- Linear acceleration: ax, ay, az (m/s^2, gravity-compensated)
- Angular velocity: gx, gy, gz (rad/s)

## Wheel Encoders

| Spec | Value |
|------|-------|
| Type | Optical incremental (single channel A) |
| Resolution | TBD (measure ticks per revolution) |
| Interface | Digital interrupt (D2, D3) |
| Measurement | Count rising edges per time period → speed |

### Speed Calculation
```
speed_mps = (tick_count / ticks_per_rev) * wheel_circumference / dt
```

## ESC and Steering Servo

| Component | Spec |
|-----------|------|
| ESC | Brushless compatible, 1000-2000 us PWM |
| Steering Servo | Standard RC servo, 1000-2000 us PWM |
| Neutral | 1500 us |
| Range | 1000 us (full left/reverse) — 2000 us (full right/forward) |
| Mapping | `pwm_us = 1500 + value * 500` where value in [-1.0, 1.0] |

## Run-Stop Relay

- D7 on Arduino: HIGH = STOP (kill throttle), LOW = RUN
- Hardware safety cutoff — independent of software
- Can be triggered by:
  - Software command (flags byte bit 1)
  - RC transmitter kill switch (future)
  - Watchdog timeout (250 ms no command → stop)
