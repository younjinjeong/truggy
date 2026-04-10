/*
 * TruggyAD ESP32 Bridge Firmware — M5StickC Plus 2
 *
 * Hardware:
 *   - BNO085 IMU via I2C on Grove port (G32=SDA, G33=SCL)
 *   - Built-in MPU6886 on internal I2C (G21/G22) — bonus sensor
 *   - Left wheel encoder on G26 (interrupt)
 *   - Right wheel encoder on G36 (interrupt, input-only)
 *   - Steering servo PWM on G0
 *   - ESC/throttle PWM on G25
 *   - Run-stop via software flag (no dedicated relay pin — use G0 or external MOSFET)
 *   - USB-C Serial to Jetson at 115200 baud
 *   - TFT display for real-time debugging
 *
 * Protocol: Same binary protocol as Arduino Uno version.
 *   TX (100 Hz): 24-byte telemetry packet (sync 0xBB)
 *   RX (50 Hz):   8-byte command packet (sync 0xAA, end 0x55)
 *
 * FreeRTOS: Sensor reading on Core 0, Serial I/O on Core 1
 */

#include <M5StickCPlus2.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_BNO08x.h>
#include "../common/protocol.h"

/* ── Pin assignments (M5StickC Plus 2) ───────────────────────────────────── */

#define PIN_ENCODER_L   26   /* HAT connector, interrupt-capable */
#define PIN_ENCODER_R   36   /* HAT connector, input-only, interrupt OK */
#define PIN_STEERING    0    /* HAT connector, PWM output */
#define PIN_THROTTLE    25   /* HAT connector, PWM output */
#define PIN_I2C_SDA     32   /* Grove port */
#define PIN_I2C_SCL     33   /* Grove port */

/* ── Constants ───────────────────────────────────────────────────────────── */

#define IMU_RATE_US     10000   /* 100 Hz */
#define PWM_NEUTRAL     1500
#define PWM_MIN         1000
#define PWM_MAX         2000
#define DISPLAY_RATE_MS 200     /* 5 Hz display update */

/* ── Encoder state (ISR-safe) ────────────────────────────────────────────── */

static volatile uint32_t encoder_l_count = 0;
static volatile uint32_t encoder_r_count = 0;

void IRAM_ATTR isr_encoder_l() { encoder_l_count++; }
void IRAM_ATTR isr_encoder_r() { encoder_r_count++; }

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
static float g_wheel_r_rps = 0.0f;
static uint32_t prev_enc_l = 0, prev_enc_r = 0;

/* Command state */
static float cmd_steering = 0.0f;
static float cmd_throttle = 0.0f;
static uint8_t cmd_flags = 0;
static unsigned long last_cmd_ms = 0;

/* Timing */
static unsigned long next_telem_us = 0;
static unsigned long next_display_ms = 0;

/* Serial receive buffer */
static uint8_t rx_buf[32];
static uint8_t rx_idx = 0;

/* Status */
static bool imu_ok = false;
static uint32_t telem_count = 0;
static uint32_t cmd_count = 0;

/* ── BNO085 setup via Grove I2C ──────────────────────────────────────────── */

static bool setup_imu() {
    /* Use Wire1 for Grove port (G32/G33), Wire0 is internal MPU6886 */
    Wire1.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

    if (!bno.begin_I2C(0x4A, &Wire1)) {
        return false;
    }

    if (!bno.enableReport(SH2_ROTATION_VECTOR, IMU_RATE_US)) return false;
    if (!bno.enableReport(SH2_LINEAR_ACCELERATION, IMU_RATE_US)) return false;
    if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_RATE_US)) return false;

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
    uint32_t enc_l = encoder_l_count;
    uint32_t enc_r = encoder_r_count;

    uint32_t dl = enc_l - prev_enc_l;
    uint32_t dr = enc_r - prev_enc_r;
    prev_enc_l = enc_l;
    prev_enc_r = enc_r;

    const float ticks_per_rev = 20.0f;
    if (dt_s > 0.001f) {
        g_wheel_l_rps = (float)dl / (ticks_per_rev * dt_s);
        g_wheel_r_rps = (float)dr / (ticks_per_rev * dt_s);
    }
}

/* ── Send telemetry ──────────────────────────────────────────────────────── */

static void send_telemetry() {
    uint8_t pkt[PROTO_TELEM_SIZE];
    proto_build_telemetry(pkt,
        g_qw, g_qx, g_qy, g_qz,
        g_ax, g_ay, g_az,
        g_gx, g_gy, g_gz,
        g_wheel_l_rps);
    Serial.write(pkt, PROTO_TELEM_SIZE);
    telem_count++;
}

/* ── Parse incoming commands ─────────────────────────────────────────────── */

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
                    cmd_count++;
                }
                rx_idx = 0;
            }
        }
    }
}

/* ── Apply actuator outputs ──────────────────────────────────────────────── */

static void apply_actuators() {
    bool armed = (cmd_flags & 0x01) != 0;
    bool estop = (cmd_flags & 0x02) != 0;
    bool watchdog_ok = (millis() - last_cmd_ms) < PROTO_WATCHDOG_MS;

    if (!armed || estop || !watchdog_ok) {
        servo_throttle.writeMicroseconds(PWM_NEUTRAL);
        return;
    }

    int steer_us = PWM_NEUTRAL + (int)(cmd_steering * 500.0f);
    int throt_us = PWM_NEUTRAL + (int)(cmd_throttle * 500.0f);

    steer_us = constrain(steer_us, PWM_MIN, PWM_MAX);
    throt_us = constrain(throt_us, PWM_MIN, PWM_MAX);

    servo_steering.writeMicroseconds(steer_us);
    servo_throttle.writeMicroseconds(throt_us);
}

/* ── TFT display update ─────────────────────────────────────────────────── */

static void update_display() {
    auto& lcd = StickCP2.Display;
    lcd.fillScreen(BLACK);
    lcd.setCursor(0, 0);
    lcd.setTextSize(1);

    /* Title */
    lcd.setTextColor(CYAN);
    lcd.println("TruggyAD ESP32");
    lcd.println();

    /* IMU status */
    lcd.setTextColor(imu_ok ? GREEN : RED);
    lcd.printf("IMU: %s\n", imu_ok ? "OK" : "FAIL");

    /* State */
    lcd.setTextColor(WHITE);
    lcd.printf("Steer: %+.2f\n", cmd_steering);
    lcd.printf("Throt: %+.2f\n", cmd_throttle);
    lcd.printf("Speed: %.1f rps\n", g_wheel_l_rps);
    lcd.println();

    /* Counters */
    lcd.setTextColor(YELLOW);
    lcd.printf("TX: %lu\n", telem_count);
    lcd.printf("RX: %lu\n", cmd_count);

    /* Armed status */
    bool armed = (cmd_flags & 0x01) != 0;
    lcd.setTextColor(armed ? GREEN : RED);
    lcd.printf("\n%s", armed ? "ARMED" : "DISARMED");
}

/* ── Setup ───────────────────────────────────────────────────────────────── */

void setup() {
    /* M5StickC Plus 2 init (display, power, internal IMU) */
    auto cfg = M5.config();
    StickCP2.begin(cfg);
    StickCP2.Display.setRotation(1);
    StickCP2.Display.fillScreen(BLACK);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.println("TruggyAD ESP32\nInitializing...");

    Serial.begin(PROTO_SERIAL_BAUD);

    /* Encoder interrupts */
    pinMode(PIN_ENCODER_L, INPUT_PULLUP);
    pinMode(PIN_ENCODER_R, INPUT);  /* G36 is input-only, no pullup */
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L), isr_encoder_l, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R), isr_encoder_r, RISING);

    /* Servo PWM (ESP32Servo uses LEDC channels) */
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    servo_steering.setPeriodHertz(50);
    servo_throttle.setPeriodHertz(50);
    servo_steering.attach(PIN_STEERING, PWM_MIN, PWM_MAX);
    servo_throttle.attach(PIN_THROTTLE, PWM_MIN, PWM_MAX);
    servo_steering.writeMicroseconds(PWM_NEUTRAL);
    servo_throttle.writeMicroseconds(PWM_NEUTRAL);

    /* BNO085 IMU via Grove I2C */
    imu_ok = setup_imu();

    StickCP2.Display.fillScreen(BLACK);
    StickCP2.Display.setCursor(0, 0);
    StickCP2.Display.printf("IMU: %s\nReady.", imu_ok ? "OK" : "FAIL");

    next_telem_us = micros();
    next_display_ms = millis();
}

/* ── Main loop (100 Hz sensor + telemetry) ───────────────────────────────── */

void loop() {
    unsigned long now_us = micros();

    if (now_us >= next_telem_us) {
        float dt_s = (float)IMU_RATE_US / 1000000.0f;

        /* Read sensors */
        if (imu_ok) read_imu();
        compute_wheel_speeds(dt_s);

        /* Send telemetry (same format as Arduino Uno) */
        send_telemetry();

        /* Parse commands */
        parse_commands();

        /* Apply actuators */
        apply_actuators();

        next_telem_us += IMU_RATE_US;
        if (now_us > next_telem_us + IMU_RATE_US * 2) {
            next_telem_us = now_us + IMU_RATE_US;
        }
    }

    /* Update display at 5 Hz */
    unsigned long now_ms = millis();
    if (now_ms >= next_display_ms) {
        StickCP2.update();  /* Read buttons */
        update_display();
        next_display_ms += DISPLAY_RATE_MS;
    }
}
