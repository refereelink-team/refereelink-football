#include "rf_telemetry.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const uint8_t vendor_imu[] = {
    0x7e,0x23,0x17,0x04,0x51,0,0x99,0xff,0xf7,7,0xf5,0xff,
    0,0,0,0,0x1b,1,0x40,0xfa,0x5d,0xfd,0x47
};
static const uint8_t uwb[] = {'m','r',2,5,1,2,0xf4,1,0xf4,1,0x90,1,0xf4,1,13,10};
typedef struct { unsigned count; rf_sample last; uint8_t bytes[68]; size_t length; } output;
static void sink(void *context, rf_channel channel, const uint8_t *frame,
                 size_t length, const rf_sample *sample) {
    output *out = context;
    assert(channel == sample->channel);
    out->count++;
    out->last = *sample;
    out->length = length;
    memcpy(out->bytes, frame, length);
}
int main(void) {
    rf_parser parser;
    output out = {0};
    rf_parser_init(&parser, RF_IMU, rf_default_config());
    for (size_t i = 0; i < sizeof(vendor_imu); ++i)
        rf_parser_feed(&parser, &vendor_imu[i], 1u, sink, &out);
    assert(out.count == 1 && parser.accepted_frames == 1);
    assert(out.last.value.imu.raw_xyz[0] == 81);
    assert(out.last.value.imu.raw_xyz[1] == -103);
    assert(out.last.value.imu.raw_xyz[2] == 2039);
    assert(fabsf(out.last.value.imu.acceleration_m_s2[2] - 9.763889f) < .0001f);
    assert(out.length == 23 && memcmp(out.bytes, vendor_imu, 23) == 0);
    uint8_t bad[23]; memcpy(bad, vendor_imu, 23); bad[8] ^= 1u;
    rf_parser_feed(&parser, bad, 23u, sink, &out);
    rf_parser_feed(&parser, vendor_imu, 23u, sink, &out);
    assert(out.count == 2 && parser.rejected_frames == 1);
    assert(parser.last_rejection == RF_REJECT_CHECKSUM);
    uint8_t invalid_length[] = {0x7e,0x23,0xff,4};
    rf_parser_feed(&parser, invalid_length, 4u, sink, &out);
    rf_parser_feed(&parser, vendor_imu, 23u, sink, &out);
    assert(out.count == 3 && parser.rejected_frames == 2);

    out.count = 0;
    rf_parser_init(&parser, RF_UWB, rf_default_config());
    rf_parser_feed(&parser, (const uint8_t *)"init pass\r\n", 11u, sink, &out);
    rf_parser_feed(&parser, uwb, 5u, sink, &out);
    assert(out.count == 0);
    rf_parser_feed(&parser, uwb + 5, sizeof(uwb) - 5u, sink, &out);
    assert(out.count == 1 && out.last.value.uwb.sequence == 513);
    assert(fabsf(out.last.value.uwb.ranges_m[0] - 5.f) < .00001f);
    uint8_t bad_uwb[16]; memcpy(bad_uwb, uwb, 16); bad_uwb[14] = 'X';
    rf_parser_feed(&parser, bad_uwb, 16u, sink, &out);
    memcpy(bad_uwb, uwb, 16); bad_uwb[12] ^= 1u;
    rf_parser_feed(&parser, bad_uwb, 16u, sink, &out);
    memcpy(bad_uwb, uwb, 16); bad_uwb[6] = 0xff; bad_uwb[7] = 0xff;
    bad_uwb[12] = 0xff; bad_uwb[13] = 0xff;
    rf_parser_feed(&parser, bad_uwb, 16u, sink, &out);
    memcpy(bad_uwb, uwb, 16); bad_uwb[3] = 6;
    rf_parser_feed(&parser, bad_uwb, 16u, sink, &out);
    rf_parser_feed(&parser, uwb, 16u, sink, &out);
    assert(out.count == 2 && parser.rejected_frames == 4);

    rf_gateway gateway;
    out.count = 0;
    assert(rf_gateway_init(&gateway, rf_default_config(), sink, &out));
    assert(rf_gateway_receive(&gateway, RF_UWB, uwb, 16u) == 16u);
    assert(rf_gateway_receive(&gateway, RF_IMU, vendor_imu, 23u) == 23u);
    assert(rf_gateway_process(&gateway, 5u) == 10u && out.count == 0);
    assert(rf_gateway_process(&gateway, 64u) == 29u && out.count == 2);
    uint8_t noise[800]; memset(noise, 'x', sizeof(noise));
    assert(rf_gateway_receive(&gateway, RF_UWB, noise, 800u) == 511u);
    assert(rf_gateway_overruns(&gateway, RF_UWB) == 289u);
    assert(rf_gateway_process(&gateway, 512u) == 0u);
    assert(gateway.discarded_overflow_bytes[RF_UWB] == 511u);
    assert(rf_gateway_receive(&gateway, RF_UWB, uwb, 16u) == 16u);
    assert(rf_gateway_process(&gateway, 512u) == 16u && out.count == 3);
    assert(gateway.parser[RF_UWB].used < RF_FRAME_CAPACITY);
    rf_config bad_config = rf_default_config();
    bad_config.maximum_range_m = NAN;
    assert(!rf_gateway_init(&gateway, bad_config, sink, &out));
    /* Deterministic noise drives byte-wise bounds and re-synchronization. */
    uint32_t rng = 42u;
    for (size_t channel = 0; channel < 2u; ++channel) {
        rf_parser_init(&parser, (rf_channel)channel, rf_default_config());
        for (size_t i = 0; i < 100000u; ++i) {
            rng = rng * 1664525u + 1013904223u;
            uint8_t byte = (uint8_t)(rng >> 24);
            rf_parser_feed(&parser, &byte, 1u, NULL, NULL);
            assert(parser.used <= RF_FRAME_CAPACITY);
        }
    }
    puts("portable parser, byte fragmentation, checksum/range rejection, two-channel bridge, bounded ring: PASS");
    return 0;
}
