/*
 * TruggyAD Arduino Bridge Firmware
 *
 * Hardware:
 *   - BNO085 IMU via I2C (A4/A5, address 0x4A)
 *   - Left wheel encoder on D2 (INT0)
 *   - Right wheel encoder on D3 (INT1)
 *   - Steering servo on D9
 *   - ESC/throttle on D10
 *   - Run-stop relay on D7
 *   - USB Serial to Jetson at 115200 baud
 *
 * Protocol:
 *   TX (100 Hz): 24-byte telemetry packet (sync 0xBB)
 *   RX (50 Hz):   8-byte command packet (sync 0xAA, end 0x55)
 */

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>

/* ── Pin assignments ─────────────────────────────────────────────────────── */

#define PIN_ENCODER_L   2   /* INT0 */
#define PIN_ENCODER_R   3   /* INT1 */
#define PIN_RUNSTOP     7
#define PIN_STEERING    9
#define PIN_THROTTLE    10

/* ── Constants ───────────────────────────────────────────────────────────── */

#define SERIAL_BAUD     115200
#define IMU_RATE_US     10000   /* 100 Hz */
#define WATCHDOG_MS     250     /* neutral throttle if no command */
#define PWM_NEUTRAL     1500
#define PWM_MIN         1000
#define PWM_MAX         2000

#define CMD_SYNC        0xAA
#define CMD_END         0x55
#define CMD_SIZE        8

#define TELEM_SYNC      0xBB
#define TELEM_SIZE      24

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

/* IMU data (updated at 100 Hz) */
static float g_qw = 1.0f, g_qx = 0.0f, g_qy = 0.0f, g_qz = 0.0f;
static float g_ax = 0.0f, g_ay = 0.0f, g_az = 0.0f;
static float g_gx = 0.0f, g_gy = 0.0f, g_gz = 0.0f;

/* Wheel speed (computed from encoder counts) */
static float g_wheel_l_rps = 0.0f;
static float g_wheel_r_rps = 0.0f;
static uint32_t prev_enc_l = 0, prev_enc_r = 0;

/* Command state */
static float cmd_steering = 0.0f;
static float cmd_throttle = 0.0f;
static uint8_t cmd_flags = 0;
static unsigned long last_cmd_ms = 0;

/* Timing */
static unsigned long next_telem_us = 0;

/* Serial receive buffer */
static uint8_t rx_buf[16];
static uint8_t rx_idx = 0;

/* ── CRC-8 (Dallas/Maxim, polynomial 0x31) ───────────────────────────────── */

static uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ── IMU setup ───────────────────────────────────────────────────────────── */

static bool setup_imu() {
    if (!bno.begin_I2C(0x4A)) {
        return false;
    }

    /* Enable rotation vector (quaternion) at 100 Hz */
    if (!bno.enableReport(SH2_ROTATION_VECTOR, IMU_RATE_US)) {
        return false;
    }
    /* Enable linear acceleration at 100 Hz */
    if (!bno.enableReport(SH2_LINEAR_ACCELERATION, IMU_RATE_US)) {
        return false;
    }
    /* Enable gyroscope at 100 Hz */
    if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_RATE_US)) {
        return false;
    }

    return true;
}

/* ── Read IMU ────────────────────────────────────────────────────────────── */

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

/* ── Compute wheel speeds ────────────────────────────────────────────────── */

static void compute_wheel_speeds(float dt_s) {
    noInterrupts();
    uint32_t enc_l = encoder_l_count;
    uint32_t enc_r = encoder_r_count;
    interrupts();

    uint32_t dl = enc_l - prev_enc_l;
    uint32_t dr = enc_r - prev_enc_r;
    prev_enc_l = enc_l;
    prev_enc_r = enc_r;

    /* Ticks per revolution — measure and update this constant */
    const float ticks_per_rev = 20.0f;
    if (dt_s > 0.001f) {
        g_wheel_l_rps = (float)dl / (ticks_per_rev * dt_s);
        g_wheel_r_rps = (float)dr / (ticks_per_rev * dt_s);
    }
}

/* ── Encode and send telemetry ───────────────────────────────────────────── */

static void send_telemetry() {
    uint8_t pkt[TELEM_SIZE];
    pkt[0] = TELEM_SYNC;

    /* Helper: float to big-endian int16 with scale */
    #define PACK16(buf, idx, val, scale) do { \
        int16_t v = (int16_t)((val) * (scale)); \
        (buf)[(idx)]     = (uint8_t)(v >> 8); \
        (buf)[(idx) + 1] = (uint8_t)(v & 0xFF); \
    } while(0)

    PACK16(pkt, 1,  g_qw, 10000.0f);
    PACK16(pkt, 3,  g_qx, 10000.0f);
    PACK16(pkt, 5,  g_qy, 10000.0f);
    PACK16(pkt, 7,  g_qz, 10000.0f);
    PACK16(pkt, 9,  g_ax, 1000.0f);
    PACK16(pkt, 11, g_ay, 1000.0f);
    PACK16(pkt, 13, g_az, 1000.0f);
    PACK16(pkt, 15, g_gx, 1000.0f);
    PACK16(pkt, 17, g_gy, 1000.0f);
    PACK16(pkt, 19, g_gz, 1000.0f);
    PACK16(pkt, 21, g_wheel_l_rps, 100.0f);

    #undef PACK16

    pkt[23] = crc8(pkt + 1, 22);

    Serial.write(pkt, TELEM_SIZE);
}

/* ── Parse incoming command ──────────────────────────────────────────────── */

static void parse_commands() {
    while (Serial.available()) {
        uint8_t b = Serial.read();
        rx_buf[rx_idx++] = b;

        /* Reset if buffer overflows */
        if (rx_idx >= sizeof(rx_buf)) {
            rx_idx = 0;
            continue;
        }

        /* Look for complete command packet */
        if (rx_idx >= CMD_SIZE) {
            /* Scan for sync byte */
            uint8_t start = 255;
            for (uint8_t i = 0; i <= rx_idx - CMD_SIZE; i++) {
                if (rx_buf[i] == CMD_SYNC &&
                    rx_buf[i + CMD_SIZE - 1] == CMD_END) {
                    start = i;
                    break;
                }
            }

            if (start < 255) {
                uint8_t* cmd = &rx_buf[start];
                uint8_t expected_crc = crc8(cmd + 1, 5);
                if (cmd[6] == expected_crc) {
                    /* Decode steering and throttle */
                    int16_t steer_raw = (int16_t)((cmd[1] << 8) | cmd[2]);
                    int16_t throt_raw = (int16_t)((cmd[3] << 8) | cmd[4]);
                    cmd_steering = (float)steer_raw / 10000.0f;
                    cmd_throttle = (float)throt_raw / 10000.0f;
                    cmd_flags = cmd[5];
                    last_cmd_ms = millis();
                }
                /* Consume processed bytes */
                rx_idx = 0;
            }
        }
    }
}

/* ── Apply actuator outputs ──────────────────────────────────────────────── */

static void apply_actuators() {
    bool armed = (cmd_flags & 0x01) != 0;
    bool estop = (cmd_flags & 0x02) != 0;
    bool watchdog_ok = (millis() - last_cmd_ms) < WATCHDOG_MS;

    if (!armed || estop || !watchdog_ok) {
        /* Safety: neutral throttle, keep last steering */
        servo_throttle.writeMicroseconds(PWM_NEUTRAL);
        digitalWrite(PIN_RUNSTOP, HIGH);  /* STOP */
        return;
    }

    /* Convert [-1, 1] to PWM [1000, 2000] */
    int steer_us = PWM_NEUTRAL + (int)(cmd_steering * 500.0f);
    int throt_us = PWM_NEUTRAL + (int)(cmd_throttle * 500.0f);

    steer_us = constrain(steer_us, PWM_MIN, PWM_MAX);
    throt_us = constrain(throt_us, PWM_MIN, PWM_MAX);

    servo_steering.writeMicroseconds(steer_us);
    servo_throttle.writeMicroseconds(throt_us);
    digitalWrite(PIN_RUNSTOP, LOW);  /* RUN */
}

/* ── Setup ───────────────────────────────────────────────────────────────── */

void setup() {
    Serial.begin(SERIAL_BAUD);
    Wire.begin();
    Wire.setClock(400000);  /* 400 kHz I2C */

    /* Pin modes */
    pinMode(PIN_ENCODER_L, INPUT_PULLUP);
    pinMode(PIN_ENCODER_R, INPUT_PULLUP);
    pinMode(PIN_RUNSTOP, OUTPUT);
    digitalWrite(PIN_RUNSTOP, HIGH);  /* Start in STOP mode */

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
        /* Blink LED to indicate IMU failure */
        pinMode(LED_BUILTIN, OUTPUT);
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }

    next_telem_us = micros();
}

/* ── Main loop (100 Hz) ──────────────────────────────────────────────────── */

void loop() {
    unsigned long now_us = micros();

    if (now_us >= next_telem_us) {
        float dt_s = (float)IMU_RATE_US / 1000000.0f;  /* 0.01s */

        /* Read sensors */
        read_imu();
        compute_wheel_speeds(dt_s);

        /* Send telemetry */
        send_telemetry();

        /* Parse any incoming commands */
        parse_commands();

        /* Apply actuator outputs */
        apply_actuators();

        next_telem_us += IMU_RATE_US;

        /* Prevent runaway if we fell behind */
        if (now_us > next_telem_us + IMU_RATE_US * 2) {
            next_telem_us = now_us + IMU_RATE_US;
        }
    }
}
