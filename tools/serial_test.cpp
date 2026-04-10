#include "common/serial.h"
#include "common/timing.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>

using namespace truggy;

static volatile bool g_running = true;
static void sig_handler(int) { g_running = false; }

static void print_usage(const char* prog) {
    fprintf(stderr, "Usage: %s [port] [baud] [--send-neutral]\n", prog);
    fprintf(stderr, "  port            Serial port (default: /dev/ttyACM0)\n");
    fprintf(stderr, "  baud            Baud rate (default: 115200)\n");
    fprintf(stderr, "  --send-neutral  Send neutral commands at 50 Hz\n");
}

int main(int argc, char** argv) {
    const char* port = "/dev/ttyACM0";
    int baud = 115200;
    bool send_commands = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--send-neutral") == 0) {
            send_commands = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (argv[i][0] == '/') {
            port = argv[i];
        } else if (atoi(argv[i]) > 0) {
            baud = atoi(argv[i]);
        }
    }

    signal(SIGINT, sig_handler);

    fprintf(stderr, "Opening %s at %d baud...\n", port, baud);

    serial_port_t sp;
    if (serial_open(sp, port, baud) < 0) {
        fprintf(stderr, "Failed to open serial port\n");
        return 1;
    }

    fprintf(stderr, "Listening for telemetry (24-byte packets, sync 0xBB)...\n");
    if (send_commands)
        fprintf(stderr, "Sending neutral commands at 50 Hz\n");
    fprintf(stderr, "Press Ctrl+C to stop\n\n");

    uint8_t rx_buf[256];
    int rx_len = 0;
    uint32_t count = 0;
    uint32_t crc_fail = 0;
    uint64_t cmd_interval = 20000;  /* 50 Hz */
    uint64_t next_cmd = now_us();

    while (g_running) {
        /* Read available data */
        if (serial_poll(sp, 10)) {
            int n = serial_read(sp, rx_buf + rx_len,
                               sizeof(rx_buf) - rx_len);
            if (n > 0) rx_len += n;
        }

        uint64_t ts = now_us();

        /* Scan for telemetry packets */
        while (rx_len >= 24) {
            /* Find sync byte 0xBB */
            int sync = -1;
            for (int i = 0; i <= rx_len - 24; i++) {
                if (rx_buf[i] == 0xBB) { sync = i; break; }
            }
            if (sync < 0) {
                rx_len = 0;
                break;
            }

            /* Discard before sync */
            if (sync > 0) {
                memmove(rx_buf, rx_buf + sync, rx_len - sync);
                rx_len -= sync;
            }

            if (rx_len < 24) break;

            /* Check CRC */
            uint8_t expected = crc8(rx_buf + 1, 22);
            if (rx_buf[23] == expected) {
                count++;

                /* Decode */
                float qw = (float)((int16_t)((rx_buf[1] << 8) | rx_buf[2])) / 10000.0f;
                float qx = (float)((int16_t)((rx_buf[3] << 8) | rx_buf[4])) / 10000.0f;
                float qy = (float)((int16_t)((rx_buf[5] << 8) | rx_buf[6])) / 10000.0f;
                float qz = (float)((int16_t)((rx_buf[7] << 8) | rx_buf[8])) / 10000.0f;
                float ax = (float)((int16_t)((rx_buf[9] << 8) | rx_buf[10])) / 1000.0f;
                float ay = (float)((int16_t)((rx_buf[11] << 8) | rx_buf[12])) / 1000.0f;
                float az = (float)((int16_t)((rx_buf[13] << 8) | rx_buf[14])) / 1000.0f;
                float gx = (float)((int16_t)((rx_buf[15] << 8) | rx_buf[16])) / 1000.0f;
                float gy = (float)((int16_t)((rx_buf[17] << 8) | rx_buf[18])) / 1000.0f;
                float gz = (float)((int16_t)((rx_buf[19] << 8) | rx_buf[20])) / 1000.0f;
                float wl = (float)((int16_t)((rx_buf[21] << 8) | rx_buf[22])) / 100.0f;

                /* Print every 10th packet (10 Hz display) */
                if (count % 10 == 0) {
                    fprintf(stderr, "\r[%5u] q=(%.3f,%.3f,%.3f,%.3f) "
                            "a=(%.2f,%.2f,%.2f) g=(%.2f,%.2f,%.2f) wl=%.1f  ",
                            count, qw, qx, qy, qz,
                            ax, ay, az, gx, gy, gz, wl);
                }
            } else {
                crc_fail++;
                if (crc_fail % 10 == 1) {
                    fprintf(stderr, "\r[CRC FAIL #%u] expected=0x%02X got=0x%02X  ",
                            crc_fail, expected, rx_buf[23]);
                }
            }

            /* Consume packet */
            memmove(rx_buf, rx_buf + 24, rx_len - 24);
            rx_len -= 24;
        }

        /* Send neutral commands if requested */
        if (send_commands && ts >= next_cmd) {
            uint8_t cmd[8];
            cmd[0] = 0xAA;
            cmd[1] = 0; cmd[2] = 0;  /* steering = 0 */
            cmd[3] = 0; cmd[4] = 0;  /* throttle = 0 */
            cmd[5] = 0x01;            /* armed */
            cmd[6] = crc8(cmd + 1, 5);
            cmd[7] = 0x55;
            serial_write(sp, cmd, 8);
            next_cmd += cmd_interval;
        }
    }

    fprintf(stderr, "\n\nTotal: %u packets, %u CRC failures (%.1f%% success)\n",
            count + crc_fail, crc_fail,
            count > 0 ? 100.0f * count / (count + crc_fail) : 0.0f);

    serial_close(sp);
    return 0;
}
