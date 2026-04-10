#include "common/serial.h"
#include "common/timing.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace truggy;

int main(int argc, char** argv) {
    const char* port = argc > 1 ? argv[1] : "/dev/ttyACM0";
    int baud = argc > 2 ? atoi(argv[2]) : 115200;

    fprintf(stderr, "Opening %s at %d baud...\n", port, baud);

    serial_port_t sp;
    if (serial_open(sp, port, baud) < 0) {
        fprintf(stderr, "Failed to open serial port\n");
        return 1;
    }

    fprintf(stderr, "Listening for telemetry (24-byte packets, sync 0xBB)...\n");
    fprintf(stderr, "Press Ctrl+C to stop\n\n");

    uint8_t buf[256];
    int count = 0;

    while (true) {
        if (!serial_poll(sp, 100)) continue;

        int n = serial_read(sp, buf, sizeof(buf));
        if (n <= 0) continue;

        // Scan for telemetry sync byte
        for (int i = 0; i < n; i++) {
            if (buf[i] == 0xBB && (n - i) >= 24) {
                uint8_t* pkt = &buf[i];
                uint8_t expected_crc = crc8(pkt + 1, 22);
                if (pkt[23] == expected_crc) {
                    count++;
                    // Decode quaternion (first field for sanity check)
                    int16_t qw_raw = (int16_t)((pkt[1] << 8) | pkt[2]);
                    float qw = qw_raw / 10000.0f;
                    fprintf(stderr, "\r[%d] qw=%.4f CRC=OK  ", count, qw);
                } else {
                    fprintf(stderr, "\r[CRC FAIL] expected=0x%02X got=0x%02X  ",
                            expected_crc, pkt[23]);
                }
                i += 23;  // skip to end of packet
            }
        }
    }

    serial_close(sp);
    return 0;
}
