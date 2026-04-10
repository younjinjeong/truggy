/*
 * TruggyAD Arduino Uno Bridge Firmware
 *
 * Hardware:
 *   - BNO085 IMU via I2C (A4/A5, address 0x4A)
 *   - Left wheel encoder on D2 (INT0)
 *   - Right wheel encoder on D3 (INT1)
 *   - Steering servo on D9
 *   - ESC/throttle on D10
 *   - Run-stop relay on D7
 *   - USB Serial to Jetson at 115200 baud
 *   - Futaba receiver CH1 (steering) on D4
 *   - Futaba receiver CH2 (throttle) on D5
 *   - Futaba receiver CH3 (mode switch) on D6
 *
 * Mode control via Futaba 7PX:
 *   CH3 switch > 1700us = AUTO (Jetson controls)
 *   CH3 switch < 1300us = MANUAL (RC passthrough)
 *   No signal = FAILSAFE (neutral output)
 */

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>
#include "../common/protocol.h"

/* ── Pin assignments (Arduino Uno) ───────────────────────────────────────── */

#define PIN_ENCODER_L   2   /* INT0 — wheel encoder left */
#define PIN_ENCODER_R   3   /* INT1 — wheel encoder right */
#define PIN_RC_STEER    4   /* Futaba receiver CH1 (steering) */
#define PIN_RC_THROT    5   /* Futaba receiver CH2 (throttle) */
#define PIN_RC_MODE     6   /* Futaba receiver CH3 (auto/manual switch) */
#define PIN_RUNSTOP     7   /* Run-stop relay */
#define PIN_STEERING    9   /* Steering servo output */
#define PIN_THROTTLE    10  /* ESC output */

/* ── Constants ───────────────────────────────────────────────────────────── */

#define IMU_RATE_US     10000   /* 100 Hz */
#define PWM_NEUTRAL     1500
#define PWM_MIN         1000
#define PWM_MAX         2000
#define RC_TIMEOUT_MS   500     /* No RC pulse = failsafe */
#define RC_PULSE_MIN    900     /* Valid PWM range */
#define RC_PULSE_MAX    2100

/* ── Encoder state (ISR) ─────────────────────────────────────────────────── */

static volatile uint32_t encoder_l_count = 0;
static volatile uint32_t encoder_r_count = 0;

void isr_encoder_l() { encoder_l_count++; }
void isr_encoder_r() { encoder_r_count++; }

/* ── Globals ─────────────────────────────────────────────────────────────── */

Adafruit_BNO08x bno;
sh2_SensorValue_t imu_value;
Servo servo_steering;
Servo servo_throttle;

/* IMU data */
static float g_qw = 1.0f, g_qx = 0.0f, g_qy = 0.0f, g_qz = 0.0f;
static float g_ax = 0.0f, g_ay = 0.0f, g_az = 0.0f;
static float g_gx = 0.0f, g_gy = 0.0f, g_gz = 0.0f;

/* Wheel speed */
static float g_wheel_l_rps = 0.0f;
static uint32_t prev_enc_l = 0, prev_enc_r = 0;

/* Jetson command state */
static float cmd_steering = 0.0f;
static float cmd_throttle = 0.0f;
static uint8_t cmd_flags = 0;
static unsigned long last_cmd_ms = 0;

/* RC receiver state */
static uint16_t rc_steer_us = PWM_NEUTRAL;
static uint16_t rc_throt_us = PWM_NEUTRAL;
static uint16_t rc_mode_us  = PWM_NEUTRAL;
static unsigned long last_rc_ms = 0;
static uint8_t drive_mode = PROTO_MODE_FAILSAFE;

/* Timing */
static unsigned long next_telem_us = 0;

/* Serial receive buffer */
static uint8_t rx_buf[16];
static uint8_t rx_idx = 0;

/* ── RC PWM reading ──────────────────────────────────────────────────────── */
/* Uses pulseIn() which is blocking but acceptable at 100Hz loop rate
   since RC PWM period is 20ms and we read 3 channels sequentially.
   Each pulseIn takes ~1-2ms. Total ~5ms per cycle, fits in 10ms budget. */

static bool read_rc_channel(uint8_t pin, uint16_t* out_us) {
    unsigned long pw = pulseIn(pin, HIGH, 25000);  /* 25ms timeout */
    if (pw >= RC_PULSE_MIN && pw <= RC_PULSE_MAX) {
        *out_us = (uint16_t)pw;
        return true;
    }
    return false;
}

static void read_rc_inputs() {
    bool got_signal = false;

    if (read_rc_channel(PIN_RC_STEER, &rc_steer_us)) got_signal = true;
    if (read_rc_channel(PIN_RC_THROT, &rc_throt_us)) got_signal = true;
    if (read_rc_channel(PIN_RC_MODE,  &rc_mode_us))  got_signal = true;

    if (got_signal) {
        last_rc_ms = millis();
    }

    /* Determine drive mode from CH3 switch */
    bool rc_connected = (millis() - last_rc_ms) < RC_TIMEOUT_MS;
    if (!rc_connected) {
        drive_mode = PROTO_MODE_FAILSAFE;
    } else if (rc_mode_us > PROTO_RC_AUTO_THRESHOLD) {
        drive_mode = PROTO_MODE_AUTO;
    } else {
        drive_mode = PROTO_MODE_MANUAL;
    }
}

/* ── IMU setup ───────────────────────────────────────────────────────────── */

static bool setup_imu() {
    if (!bno.begin_I2C(0x4A)) return false;
    if (!bno.enableReport(SH2_ROTATION_VECTOR, IMU_RATE_US)) return false;
    if (!bno.enableReport(SH2_LINEAR_ACCELERATION, IMU_RATE_US)) return false;
    if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_RATE_US)) return false;
    return true;
}

static void read_imu() {
    while (bno.getSensorEvent(&imu_value)) {
        switch (imu_value.sensorId) {
            case SH2_ROTATION_VECTOR:
                g_qw = imu_value.un.rotationVector.real;
                g_qx = imu_value.un.rotationVector.i;
                g_qy = imu_value.un.rotationVector.j;
                g_qz = imu_value.un.rotationVector.k;
                break;
            case SH2_LINEAR_ACCELERATION:
                g_ax = imu_value.un.linearAcceleration.x;
                g_ay = imu_value.un.linearAcceleration.y;
                g_az = imu_value.un.linearAcceleration.z;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                g_gx = imu_value.un.gyroscope.x;
                g_gy = imu_value.un.gyroscope.y;
                g_gz = imu_value.un.gyroscope.z;
                break;
        }
    }
}

/* ── Wheel speed ─────────────────────────────────────────────────────────── */

static void compute_wheel_speeds(float dt_s) {
    noInterrupts();
    uint32_t enc_l = encoder_l_count;
    interrupts();

    uint32_t dl = enc_l - prev_enc_l;
    prev_enc_l = enc_l;

    const float ticks_per_rev = 20.0f;
    if (dt_s > 0.001f) {
        g_wheel_l_rps = (float)dl / (ticks_per_rev * dt_s);
    }
}

/* ── Telemetry ───────────────────────────────────────────────────────────── */

static void send_telemetry() {
    int8_t rc_throt_pct = (int8_t)constrain(
        ((int)rc_throt_us - PWM_NEUTRAL) * 100 / 500, -100, 100);

    uint8_t pkt[PROTO_TELEM_SIZE];
    proto_build_telemetry(pkt,
        g_qw, g_qx, g_qy, g_qz,
        g_ax, g_ay, g_az,
        g_gx, g_gy, g_gz,
        g_wheel_l_rps,
        drive_mode,
        rc_steer_us,
        rc_throt_pct);
    Serial.write(pkt, PROTO_TELEM_SIZE);
}

/* ── Command parsing ─────────────────────────────────────────────────────── */

static void parse_commands() {
    while (Serial.available()) {
        rx_buf[rx_idx++] = Serial.read();

        if (rx_idx >= sizeof(rx_buf)) {
            rx_idx = 0;
            continue;
        }

        if (rx_idx >= PROTO_CMD_SIZE) {
            uint8_t start = 255;
            for (uint8_t i = 0; i <= rx_idx - PROTO_CMD_SIZE; i++) {
                if (rx_buf[i] == PROTO_CMD_SYNC &&
                    rx_buf[i + PROTO_CMD_SIZE - 1] == PROTO_CMD_END) {
                    start = i;
                    break;
                }
            }

            if (start < 255) {
                proto_command_t parsed = proto_parse_command(&rx_buf[start]);
                if (parsed.valid) {
                    cmd_steering = parsed.steering;
                    cmd_throttle = parsed.throttle;
                    cmd_flags = parsed.flags;
                    last_cmd_ms = millis();
                }
                rx_idx = 0;
            }
        }
    }
}

/* ── Actuator output ─────────────────────────────────────────────────────── */

static void apply_actuators() {
    int steer_us, throt_us;

    switch (drive_mode) {
        case PROTO_MODE_MANUAL:
            /* RC passthrough: Futaba 7PX controls directly */
            steer_us = rc_steer_us;
            throt_us = rc_throt_us;
            digitalWrite(PIN_RUNSTOP, LOW);  /* RUN */
            break;

        case PROTO_MODE_AUTO: {
            /* Jetson controls: use serial commands */
            bool armed = (cmd_flags & 0x01) != 0;
            bool estop = (cmd_flags & 0x02) != 0;
            bool watchdog_ok = (millis() - last_cmd_ms) < PROTO_WATCHDOG_MS;

            if (!armed || estop || !watchdog_ok) {
                steer_us = PWM_NEUTRAL;
                throt_us = PWM_NEUTRAL;
                digitalWrite(PIN_RUNSTOP, HIGH);  /* STOP */
            } else {
                steer_us = PWM_NEUTRAL + (int)(cmd_steering * 500.0f);
                throt_us = PWM_NEUTRAL + (int)(cmd_throttle * 500.0f);
                digitalWrite(PIN_RUNSTOP, LOW);  /* RUN */
            }
            break;
        }

        case PROTO_MODE_FAILSAFE:
        default:
            /* No RC signal: safe neutral */
            steer_us = PWM_NEUTRAL;
            throt_us = PWM_NEUTRAL;
            digitalWrite(PIN_RUNSTOP, HIGH);  /* STOP */
            break;
    }

    steer_us = constrain(steer_us, PWM_MIN, PWM_MAX);
    throt_us = constrain(throt_us, PWM_MIN, PWM_MAX);

    servo_steering.writeMicroseconds(steer_us);
    servo_throttle.writeMicroseconds(throt_us);
}

/* ── Setup ───────────────────────────────────────────────────────────────── */

void setup() {
    Serial.begin(PROTO_SERIAL_BAUD);
    Wire.begin();
    Wire.setClock(400000);

    /* Pins */
    pinMode(PIN_ENCODER_L, INPUT_PULLUP);
    pinMode(PIN_ENCODER_R, INPUT_PULLUP);
    pinMode(PIN_RC_STEER, INPUT);
    pinMode(PIN_RC_THROT, INPUT);
    pinMode(PIN_RC_MODE, INPUT);
    pinMode(PIN_RUNSTOP, OUTPUT);
    digitalWrite(PIN_RUNSTOP, HIGH);  /* Start in STOP */

    /* Encoder interrupts */
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L), isr_encoder_l, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R), isr_encoder_r, RISING);

    /* Servos */
    servo_steering.attach(PIN_STEERING);
    servo_throttle.attach(PIN_THROTTLE);
    servo_steering.writeMicroseconds(PWM_NEUTRAL);
    servo_throttle.writeMicroseconds(PWM_NEUTRAL);

    /* IMU */
    if (!setup_imu()) {
        pinMode(LED_BUILTIN, OUTPUT);
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }

    next_telem_us = micros();
}

/* ── Main loop ───────────────────────────────────────────────────────────── */

void loop() {
    unsigned long now_us = micros();

    if (now_us >= next_telem_us) {
        float dt_s = (float)IMU_RATE_US / 1000000.0f;

        read_imu();
        read_rc_inputs();
        compute_wheel_speeds(dt_s);
        send_telemetry();
        parse_commands();
        apply_actuators();

        next_telem_us += IMU_RATE_US;
        if (now_us > next_telem_us + IMU_RATE_US * 2) {
            next_telem_us = now_us + IMU_RATE_US;
        }
    }
}
