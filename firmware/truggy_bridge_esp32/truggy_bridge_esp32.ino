/*
 * TruggyAD ESP32 Bridge Firmware — M5StickC Plus 2
 *
 * Hardware:
 *   - BNO085 IMU via I2C on Grove port (G32=SDA, G33=SCL)
 *   - Built-in MPU6886 on internal I2C (G21/G22)
 *   - Left wheel encoder on G26 (interrupt)
 *   - Right wheel encoder on G36 (interrupt, input-only)
 *   - Steering servo PWM on G0
 *   - ESC/throttle PWM on G25
 *   - USB-C Serial to Jetson at 115200 baud
 *   - TFT display for real-time debugging
 *
 * RC receiver via S.BUS on Serial2 (or PWM on spare pins):
 *   For M5StickC Plus 2 with limited GPIO, use Futaba S.BUS output
 *   connected to G33 (Grove SCL) via inverter, OR use BtnA/BtnB
 *   as manual mode override. See PIN_RC_* defines below.
 *
 * Mode control via Futaba 7PX:
 *   CH3 switch > 1700us = AUTO (Jetson controls)
 *   CH3 switch < 1300us = MANUAL (RC passthrough)
 *   BtnA on M5Stick = force MANUAL override
 */

#include <M5StickCPlus2.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_BNO08x.h>
#include "../common/protocol.h"

/* ── Pin assignments (M5StickC Plus 2) ───────────────────────────────────── */

#define PIN_ENCODER_L   26   /* HAT, interrupt-capable */
#define PIN_ENCODER_R   36   /* HAT, input-only, interrupt OK */
#define PIN_STEERING    0    /* HAT, PWM output */
#define PIN_THROTTLE    25   /* HAT, PWM output */
#define PIN_I2C_SDA     32   /* Grove port */
#define PIN_I2C_SCL     33   /* Grove port */

/* RC input: ESP32 has limited pins on M5StickC Plus 2.
 * Option 1: Use BtnA (G37) as manual override toggle (no RC PWM reading)
 * Option 2: Sacrifice one encoder and use G36 for RC mode channel
 * We use Option 1: button toggle + Jetson arm flag for mode control */
#define PIN_BTN_A       37   /* Built-in button A (input-only) */
#define PIN_BTN_B       39   /* Built-in button B (input-only) */

/* ── Constants ───────────────────────────────────────────────────────────── */

#define IMU_RATE_US     10000   /* 100 Hz */
#define PWM_NEUTRAL     1500
#define PWM_MIN         1000
#define PWM_MAX         2000
#define DISPLAY_RATE_MS 200     /* 5 Hz display update */

/* ── Encoder ISR ─────────────────────────────────────────────────────────── */

static volatile uint32_t encoder_l_count = 0;
static volatile uint32_t encoder_r_count = 0;

void IRAM_ATTR isr_encoder_l() { encoder_l_count++; }
void IRAM_ATTR isr_encoder_r() { encoder_r_count++; }

/* ── Globals ─────────────────────────────────────────────────────────────── */

Adafruit_BNO08x bno;
sh2_SensorValue_t imu_value;
Servo servo_steering;
Servo servo_throttle;

/* IMU */
static float g_qw = 1.0f, g_qx = 0.0f, g_qy = 0.0f, g_qz = 0.0f;
static float g_ax = 0.0f, g_ay = 0.0f, g_az = 0.0f;
static float g_gx = 0.0f, g_gy = 0.0f, g_gz = 0.0f;

/* Wheels */
static float g_wheel_l_rps = 0.0f;
static uint32_t prev_enc_l = 0;

/* Jetson commands */
static float cmd_steering = 0.0f;
static float cmd_throttle = 0.0f;
static uint8_t cmd_flags = 0;
static unsigned long last_cmd_ms = 0;

/* Mode control */
static uint8_t drive_mode = PROTO_MODE_MANUAL;  /* Start manual (safe) */
static bool manual_override = true;  /* BtnA toggles this */

/* Timing */
static unsigned long next_telem_us = 0;
static unsigned long next_display_ms = 0;

/* Serial */
static uint8_t rx_buf[32];
static uint8_t rx_idx = 0;

/* Stats */
static bool imu_ok = false;
static uint32_t telem_count = 0;
static uint32_t cmd_count = 0;

/* ── Mode control ────────────────────────────────────────────────────────── */

static void update_drive_mode() {
    /* BtnA press toggles manual override */
    /* M5StickC Plus 2 buttons are active-low, handled by StickCP2.update() */
    if (StickCP2.BtnA.wasPressed()) {
        manual_override = !manual_override;
    }

    /* BtnB = emergency stop (always available) */
    if (StickCP2.BtnB.isPressed()) {
        drive_mode = PROTO_MODE_FAILSAFE;
        return;
    }

    if (manual_override) {
        drive_mode = PROTO_MODE_MANUAL;
    } else {
        /* Auto mode: Jetson controls, but only if armed */
        bool armed = (cmd_flags & 0x01) != 0;
        drive_mode = armed ? PROTO_MODE_AUTO : PROTO_MODE_MANUAL;
    }
}

/* ── IMU ─────────────────────────────────────────────────────────────────── */

static bool setup_imu() {
    Wire1.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    if (!bno.begin_I2C(0x4A, &Wire1)) return false;
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

/* ── Wheels ───────────────────────────────────────────────────────────────── */

static void compute_wheel_speeds(float dt_s) {
    uint32_t enc_l = encoder_l_count;
    uint32_t dl = enc_l - prev_enc_l;
    prev_enc_l = enc_l;
    const float ticks_per_rev = 20.0f;
    if (dt_s > 0.001f) {
        g_wheel_l_rps = (float)dl / (ticks_per_rev * dt_s);
    }
}

/* ── Telemetry ───────────────────────────────────────────────────────────── */

static void send_telemetry() {
    uint8_t pkt[PROTO_TELEM_SIZE];
    proto_build_telemetry(pkt,
        g_qw, g_qx, g_qy, g_qz,
        g_ax, g_ay, g_az,
        g_gx, g_gy, g_gz,
        g_wheel_l_rps,
        drive_mode,
        PWM_NEUTRAL,  /* No RC PWM reading on ESP32 — report neutral */
        0);           /* No RC throttle */
    Serial.write(pkt, PROTO_TELEM_SIZE);
    telem_count++;
}

/* ── Commands ────────────────────────────────────────────────────────────── */

static void parse_commands() {
    while (Serial.available()) {
        rx_buf[rx_idx++] = Serial.read();
        if (rx_idx >= sizeof(rx_buf)) { rx_idx = 0; continue; }

        if (rx_idx >= PROTO_CMD_SIZE) {
            uint8_t start = 255;
            for (uint8_t i = 0; i <= rx_idx - PROTO_CMD_SIZE; i++) {
                if (rx_buf[i] == PROTO_CMD_SYNC &&
                    rx_buf[i + PROTO_CMD_SIZE - 1] == PROTO_CMD_END) {
                    start = i; break;
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

/* ── Actuators ───────────────────────────────────────────────────────────── */

static void apply_actuators() {
    int steer_us, throt_us;

    switch (drive_mode) {
        case PROTO_MODE_MANUAL:
            /* On ESP32, manual = neutral (no RC PWM reading).
             * User drives with Futaba directly to receiver-connected servos,
             * OR this board is bypassed in manual mode. */
            steer_us = PWM_NEUTRAL;
            throt_us = PWM_NEUTRAL;
            break;

        case PROTO_MODE_AUTO: {
            bool watchdog_ok = (millis() - last_cmd_ms) < PROTO_WATCHDOG_MS;
            if (!watchdog_ok) {
                steer_us = PWM_NEUTRAL;
                throt_us = PWM_NEUTRAL;
            } else {
                steer_us = PWM_NEUTRAL + (int)(cmd_steering * 500.0f);
                throt_us = PWM_NEUTRAL + (int)(cmd_throttle * 500.0f);
            }
            break;
        }

        case PROTO_MODE_FAILSAFE:
        default:
            steer_us = PWM_NEUTRAL;
            throt_us = PWM_NEUTRAL;
            break;
    }

    servo_steering.writeMicroseconds(constrain(steer_us, PWM_MIN, PWM_MAX));
    servo_throttle.writeMicroseconds(constrain(throt_us, PWM_MIN, PWM_MAX));
}

/* ── Display ─────────────────────────────────────────────────────────────── */

static void update_display() {
    auto& lcd = StickCP2.Display;
    lcd.fillScreen(BLACK);
    lcd.setCursor(0, 0);
    lcd.setTextSize(1);

    /* Mode banner */
    uint16_t mode_color;
    const char* mode_str;
    switch (drive_mode) {
        case PROTO_MODE_AUTO:     mode_color = GREEN;  mode_str = "AUTO";     break;
        case PROTO_MODE_MANUAL:   mode_color = YELLOW; mode_str = "MANUAL";   break;
        case PROTO_MODE_FAILSAFE: mode_color = RED;    mode_str = "FAILSAFE"; break;
        default:                  mode_color = WHITE;  mode_str = "???";      break;
    }
    lcd.setTextColor(mode_color);
    lcd.setTextSize(2);
    lcd.println(mode_str);
    lcd.setTextSize(1);
    lcd.println();

    /* IMU */
    lcd.setTextColor(imu_ok ? GREEN : RED);
    lcd.printf("IMU: %s\n", imu_ok ? "OK" : "FAIL");

    /* Controls */
    lcd.setTextColor(WHITE);
    lcd.printf("Steer: %+.2f\n", cmd_steering);
    lcd.printf("Throt: %+.2f\n", cmd_throttle);
    lcd.printf("Speed: %.1f rps\n", g_wheel_l_rps);

    /* Stats */
    lcd.setTextColor(CYAN);
    lcd.printf("TX:%lu RX:%lu\n", telem_count, cmd_count);

    /* Hint */
    lcd.setTextColor(DARKGREY);
    lcd.println("\nBtnA: toggle mode");
    lcd.println("BtnB: E-STOP");
}

/* ── Setup ───────────────────────────────────────────────────────────────── */

void setup() {
    auto cfg = M5.config();
    StickCP2.begin(cfg);
    StickCP2.Display.setRotation(1);
    StickCP2.Display.fillScreen(BLACK);
    StickCP2.Display.setTextColor(WHITE);
    StickCP2.Display.println("TruggyAD ESP32\nInitializing...");

    Serial.begin(PROTO_SERIAL_BAUD);

    /* Encoders */
    pinMode(PIN_ENCODER_L, INPUT_PULLUP);
    pinMode(PIN_ENCODER_R, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L), isr_encoder_l, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R), isr_encoder_r, RISING);

    /* Servos */
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    servo_steering.setPeriodHertz(50);
    servo_throttle.setPeriodHertz(50);
    servo_steering.attach(PIN_STEERING, PWM_MIN, PWM_MAX);
    servo_throttle.attach(PIN_THROTTLE, PWM_MIN, PWM_MAX);
    servo_steering.writeMicroseconds(PWM_NEUTRAL);
    servo_throttle.writeMicroseconds(PWM_NEUTRAL);

    /* IMU */
    imu_ok = setup_imu();

    StickCP2.Display.fillScreen(BLACK);
    StickCP2.Display.setCursor(0, 0);
    StickCP2.Display.printf("IMU: %s\nMode: MANUAL\nReady.",
                            imu_ok ? "OK" : "FAIL");

    next_telem_us = micros();
    next_display_ms = millis();
}

/* ── Loop ────────────────────────────────────────────────────────────────── */

void loop() {
    unsigned long now_us = micros();

    if (now_us >= next_telem_us) {
        float dt_s = (float)IMU_RATE_US / 1000000.0f;

        if (imu_ok) read_imu();
        compute_wheel_speeds(dt_s);
        parse_commands();
        update_drive_mode();
        apply_actuators();
        send_telemetry();

        next_telem_us += IMU_RATE_US;
        if (now_us > next_telem_us + IMU_RATE_US * 2) {
            next_telem_us = now_us + IMU_RATE_US;
        }
    }

    /* Display at 5 Hz */
    unsigned long now_ms = millis();
    if (now_ms >= next_display_ms) {
        StickCP2.update();
        update_display();
        next_display_ms += DISPLAY_RATE_MS;
    }
}
