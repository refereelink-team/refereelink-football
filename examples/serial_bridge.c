/* Host example: preserve complete verified manufacturer frames, one selected channel. */
#include "rf_telemetry.h"
#include <stdio.h>
#include <string.h>

static void forward(void *context, rf_channel channel, const uint8_t *frame,
                    size_t length, const rf_sample *sample) {
    (void)channel;
    (void)sample;
    FILE *output = context;
    if (fwrite(frame, 1u, length, output) != length) {
        fprintf(stderr, "output write failure\n");
    }
}
int main(int argc, char **argv) {
    if (argc != 2 || (strcmp(argv[1], "uwb") != 0 && strcmp(argv[1], "imu") != 0)) {
        fprintf(stderr, "usage: serial_bridge uwb|imu < UART_BYTES > FORWARDED_BYTES\n");
        return 2;
    }
    rf_gateway gateway;
    if (!rf_gateway_init(&gateway, rf_default_config(), forward, stdout)) return 3;
    rf_channel channel = strcmp(argv[1], "uwb") == 0 ? RF_UWB : RF_IMU;
    uint8_t bytes[128];
    size_t size;
    while ((size = fread(bytes, 1u, sizeof(bytes), stdin)) > 0u) {
        rf_gateway_receive(&gateway, channel, bytes, size);
        rf_gateway_process(&gateway, 256u);
    }
    fprintf(stderr, "accepted=%u rejected=%u overruns=%u\n",
            (unsigned)gateway.parser[channel].accepted_frames,
            (unsigned)gateway.parser[channel].rejected_frames,
            (unsigned)rf_gateway_overruns(&gateway, channel));
    return ferror(stdin) || ferror(stdout) ? 1 : 0;
}
