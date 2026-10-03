#include "rf_telemetry.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const uint8_t vendor_imu[] = {
    0x7e,0x23,0x17,0x04,0x51,0,0x99,0xff,0xf7,7,0xf5,0xff,
    0,0,0,0,0x1b,1,0x40,0xfa,0x5d,0xfd,0x47
};
/* Synthetic fixed float fixtures from the documented layout, not device captures.
   Bytes were supplied independently of the decoder and checksum helper. */
static const uint8_t quaternion[] = {
    0x7e,0x23,0x15,0x16,0x00,0x00,0x80,0x3f,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8b
};
static const uint8_t euler[] = {
    0x7e,0x23,0x11,0x26,0x00,0x00,0x00,0x3f,0x00,0x00,0x80,0xbf,
    0x00,0x00,0xc0,0x3f,0x55
};
static const uint8_t barometer[] = {
    0x7e,0x23,0x15,0x32,0x00,0x00,0xa0,0x3f,0x00,0x00,0xc8,0x41,
    0x80,0xe6,0xc5,0x47,0x00,0xda,0xc5,0x47,0x28
};
static const uint8_t version[] = {0x7e,0x23,0x08,0x01,0x01,0x02,0x03,0xb0};
static const uint8_t response[] = {0x7e,0x23,0x07,0x81,0x70,0x01,0x9a};
typedef struct { const uint8_t *bytes; size_t length; rf_imu_kind kind; } imu_fixture;
static const imu_fixture imu_profiles[] = {
    {vendor_imu, sizeof(vendor_imu), RF_IMU_RAW},
    {quaternion, sizeof(quaternion), RF_IMU_QUATERNION},
    {euler, sizeof(euler), RF_IMU_EULER},
    {barometer, sizeof(barometer), RF_IMU_BAROMETER}
};
static const uint8_t uwb[] = {'m','r',2,5,1,2,0xf4,1,0xf4,1,0x90,1,0xf4,1,13,10};
typedef struct { unsigned count; rf_sample last; uint8_t bytes[68]; size_t length; } output;
static void assert_inactive_imu_fields_zero(const rf_imu_sample *imu) {
    if (imu->kind != RF_IMU_RAW) {
        for (size_t i = 0; i < 3u; ++i) {
            assert(imu->raw_xyz[i] == 0 && imu->acceleration_m_s2[i] == 0.0f);
            assert(imu->gyro_raw_xyz[i] == 0 && imu->angular_velocity_rad_s[i] == 0.0f);
            assert(imu->magnetometer_raw_xyz[i] == 0);
        }
    }
    if (imu->kind != RF_IMU_QUATERNION)
        for (size_t i = 0; i < 4u; ++i) assert(imu->quaternion_wxyz[i] == 0.0f);
    if (imu->kind != RF_IMU_EULER)
        for (size_t i = 0; i < 3u; ++i) assert(imu->euler_rad[i] == 0.0f);
    if (imu->kind != RF_IMU_BAROMETER) {
        assert(imu->barometer.relative_height_m == 0.0f);
        assert(imu->barometer.temperature_c == 0.0f);
        assert(imu->barometer.pressure_pa == 0.0f);
        assert(imu->barometer.reference_pressure_pa == 0.0f);
    }
}
static void sink(void *context, rf_channel channel, const uint8_t *frame,
                 size_t length, const rf_sample *sample) {
    output *out = context;
    assert(channel == sample->channel);
    if (channel == RF_IMU) assert_inactive_imu_fields_zero(&sample->value.imu);
    out->count++;
    out->last = *sample;
    out->length = length;
    memcpy(out->bytes, frame, length);
}
static void checksum(uint8_t *frame, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length - 1u; ++i) sum = (uint8_t)(sum + frame[i]);
    frame[length - 1u] = sum;
}
static void feed_fragmented(rf_parser *parser, const uint8_t *bytes, size_t length,
                            unsigned fragmented, rf_frame_sink callback, void *context) {
    if (!fragmented) {
        rf_parser_feed(parser, bytes, length, callback, context);
        return;
    }
    for (size_t i = 0; i < length; ++i)
        rf_parser_feed(parser, bytes + i, 1u, callback, context);
}
static void assert_golden_sample(const rf_sample *sample, const imu_fixture *fixture) {
    assert(sample->channel == RF_IMU);
    const rf_imu_sample *imu = &sample->value.imu;
    assert(imu->kind == fixture->kind);
    assert_inactive_imu_fields_zero(imu);
    switch (imu->kind) {
        case RF_IMU_RAW:
            assert(imu->raw_xyz[0] == 81 && imu->raw_xyz[1] == -103 && imu->raw_xyz[2] == 2039);
            assert(fabsf(imu->acceleration_m_s2[2] - 9.763889f) < .0001f);
            assert(imu->gyro_raw_xyz[0] == -11 && imu->gyro_raw_xyz[1] == 0 && imu->gyro_raw_xyz[2] == 0);
            assert(fabsf(imu->angular_velocity_rad_s[0] -
                         (-11.0f / 32767.0f) * 2000.0f * (3.14159265358979323846f / 180.0f)) < .000001f);
            assert(imu->magnetometer_raw_xyz[0] == 283);
            assert(imu->magnetometer_raw_xyz[1] == -1472);
            assert(imu->magnetometer_raw_xyz[2] == -675);
            break;
        case RF_IMU_QUATERNION:
            assert(imu->quaternion_wxyz[0] == 1.0f);
            for (size_t i = 1; i < 4u; ++i) assert(imu->quaternion_wxyz[i] == 0.0f);
            break;
        case RF_IMU_EULER:
            assert(imu->euler_rad[0] == .5f && imu->euler_rad[1] == -1.0f && imu->euler_rad[2] == 1.5f);
            break;
        case RF_IMU_BAROMETER:
            assert(imu->barometer.relative_height_m == 1.25f);
            assert(imu->barometer.temperature_c == 25.0f);
            assert(imu->barometer.pressure_pa == 101325.0f);
            assert(imu->barometer.reference_pressure_pa == 101300.0f);
            break;
        default: assert(0);
    }
}
static void test_imu_golden_profiles(void) {
    for (size_t i = 0; i < sizeof(imu_profiles) / sizeof(imu_profiles[0]); ++i) {
        const imu_fixture *fixture = &imu_profiles[i];
        for (unsigned fragmented = 0; fragmented < 2u; ++fragmented) {
            rf_parser parser;
            output out = {0};
            rf_parser_init(&parser, RF_IMU, rf_default_config());
            feed_fragmented(&parser, fixture->bytes, fixture->length, fragmented, sink, &out);
            assert(out.count == 1u && parser.accepted_frames == 1u);
            assert(parser.rejected_frames == 0u && parser.used == 0u);
            assert(out.length == fixture->length && memcmp(out.bytes, fixture->bytes, fixture->length) == 0);
            assert_golden_sample(&out.last, fixture);
        }
    }
}
static void mixed_sink(void *context, rf_channel channel, const uint8_t *frame,
                       size_t length, const rf_sample *sample) {
    size_t *count = context;
    assert(*count < sizeof(imu_profiles) / sizeof(imu_profiles[0]));
    const imu_fixture *fixture = &imu_profiles[*count];
    assert(channel == RF_IMU && length == fixture->length);
    assert(memcmp(frame, fixture->bytes, length) == 0);
    assert_golden_sample(sample, fixture);
    (*count)++;
}
static void test_mixed_imu_profiles_skip_control_frames(void) {
    uint8_t stream[128];
    size_t used = 0;
    const uint8_t *frames[] = {vendor_imu, version, quaternion, response, euler, barometer};
    for (size_t i = 0; i < sizeof(frames) / sizeof(frames[0]); ++i) {
        size_t length = frames[i][2];
        assert(used + length <= sizeof(stream));
        memcpy(stream + used, frames[i], length);
        used += length;
    }
    for (unsigned fragmented = 0; fragmented < 2u; ++fragmented) {
        rf_parser parser;
        size_t count = 0;
        rf_parser_init(&parser, RF_IMU, rf_default_config());
        if (!fragmented) {
            rf_parser_feed(&parser, stream, used, mixed_sink, &count);
        } else {
            for (size_t offset = 0; offset < used;) {
                size_t chunk = 1u + offset % 7u;
                if (chunk > used - offset) chunk = used - offset;
                rf_parser_feed(&parser, stream + offset, chunk, mixed_sink, &count);
                offset += chunk;
            }
        }
        assert(count == 4u && parser.accepted_frames == 4u);
        assert(parser.rejected_frames == 0u && parser.used == 0u);
    }
}
static void test_imu_profile_checksum_recovery(void) {
    for (size_t i = 0; i < sizeof(imu_profiles) / sizeof(imu_profiles[0]); ++i) {
        const imu_fixture *fixture = &imu_profiles[i];
        uint8_t bad[RF_FRAME_CAPACITY];
        memcpy(bad, fixture->bytes, fixture->length);
        bad[fixture->length - 1u] ^= 1u;
        for (unsigned fragmented = 0; fragmented < 2u; ++fragmented) {
            rf_parser parser;
            output out = {0};
            rf_parser_init(&parser, RF_IMU, rf_default_config());
            feed_fragmented(&parser, bad, fixture->length, fragmented, sink, &out);
            assert(out.count == 0u && parser.rejected_frames == 1u);
            assert(parser.last_rejection == RF_REJECT_CHECKSUM);
            feed_fragmented(&parser, fixture->bytes, fixture->length, fragmented, sink, &out);
            assert(out.count == 1u && parser.accepted_frames == 1u);
            assert(parser.rejected_frames == 1u);
            assert_golden_sample(&out.last, fixture);
        }
    }
}
static void write_le32(uint8_t *bytes, uint32_t bits) {
    for (size_t i = 0; i < 4u; ++i) bytes[i] = (uint8_t)(bits >> (8u * i));
}
static void test_nonfinite_imu_values_reject_and_recover(void) {
    const uint32_t nonfinite[] = {UINT32_C(0x7fc00000), UINT32_C(0x7f800001),
                                 UINT32_C(0x7f800000), UINT32_C(0xff800000)};
    for (size_t profile = 1; profile < sizeof(imu_profiles) / sizeof(imu_profiles[0]); ++profile) {
        const imu_fixture *fixture = &imu_profiles[profile];
        size_t fields = (fixture->length - 5u) / 4u;
        for (size_t field = 0; field < fields; ++field) {
            for (size_t value = 0; value < sizeof(nonfinite) / sizeof(nonfinite[0]); ++value) {
                uint8_t bad[RF_FRAME_CAPACITY];
                memcpy(bad, fixture->bytes, fixture->length);
                write_le32(bad + 4u + 4u * field, nonfinite[value]);
                checksum(bad, fixture->length);
                for (unsigned fragmented = 0; fragmented < 2u; ++fragmented) {
                    rf_parser parser;
                    output out = {0};
                    rf_parser_init(&parser, RF_IMU, rf_default_config());
                    feed_fragmented(&parser, bad, fixture->length, fragmented, sink, &out);
                    assert(out.count == 0u && parser.rejected_frames == 1u);
                    assert(parser.last_rejection == RF_REJECT_VALUE);
                    feed_fragmented(&parser, fixture->bytes, fixture->length, fragmented, sink, &out);
                    assert(out.count == 1u && parser.accepted_frames == 1u);
                    assert(parser.rejected_frames == 1u);
                    assert_golden_sample(&out.last, fixture);
                }
            }
        }
    }
}
static void test_finite_imu_values_are_preserved(void) {
    /* These finite values deliberately avoid assumptions about norm or physical limits. */
    const uint32_t finite[] = {UINT32_C(0x00000000), UINT32_C(0xc0200000),
                              UINT32_C(0x80000000), UINT32_C(0x7f7fffff)};
    for (size_t profile = 1; profile < sizeof(imu_profiles) / sizeof(imu_profiles[0]); ++profile) {
        const imu_fixture *fixture = &imu_profiles[profile];
        uint8_t frame[RF_FRAME_CAPACITY];
        memcpy(frame, fixture->bytes, fixture->length);
        size_t fields = (fixture->length - 5u) / 4u;
        for (size_t i = 0; i < fields; ++i) write_le32(frame + 4u + 4u * i, finite[i]);
        checksum(frame, fixture->length);
        rf_parser parser;
        output out = {0};
        rf_parser_init(&parser, RF_IMU, rf_default_config());
        rf_parser_feed(&parser, frame, fixture->length, sink, &out);
        assert(out.count == 1u && parser.rejected_frames == 0u);
        assert(out.last.value.imu.kind == fixture->kind);
        assert(memcmp(out.bytes, frame, fixture->length) == 0);
        float decoded[4] = {0};
        if (fixture->kind == RF_IMU_QUATERNION) {
            memcpy(decoded, out.last.value.imu.quaternion_wxyz, sizeof(decoded));
        } else if (fixture->kind == RF_IMU_EULER) {
            memcpy(decoded, out.last.value.imu.euler_rad, 3u * sizeof(float));
        } else {
            decoded[0] = out.last.value.imu.barometer.relative_height_m;
            decoded[1] = out.last.value.imu.barometer.temperature_c;
            decoded[2] = out.last.value.imu.barometer.pressure_pa;
            decoded[3] = out.last.value.imu.barometer.reference_pressure_pa;
        }
        assert(decoded[0] == 0.0f && !signbit(decoded[0]));
        assert(decoded[1] == -2.5f);
        assert(decoded[2] == 0.0f && signbit(decoded[2]));
        if (fields == 4u) assert(decoded[3] == FLT_MAX);
    }
}
static void test_wrong_imu_known_lengths_recover(void) {
    const uint8_t *known[] = {vendor_imu, quaternion, euler, barometer, version, response};
    for (size_t profile = 0; profile < sizeof(known) / sizeof(known[0]); ++profile) {
        for (size_t length = 5u; length <= RF_FRAME_CAPACITY; ++length) {
            if (length == known[profile][2]) continue;
            uint8_t stream[4u + sizeof(vendor_imu)] = {0x7e, 0x23, (uint8_t)length, known[profile][3]};
            memcpy(stream + 4u, vendor_imu, sizeof(vendor_imu));
            for (unsigned fragmented = 0; fragmented < 2u; ++fragmented) {
                rf_parser parser;
                output out = {0};
                rf_parser_init(&parser, RF_IMU, rf_default_config());
                if (fragmented) {
                    rf_parser_feed(&parser, stream, 3u, sink, &out);
                    assert(parser.rejected_frames == 0u && out.count == 0u);
                    rf_parser_feed(&parser, stream + 3u, 1u, sink, &out);
                    assert(parser.rejected_frames == 1u);
                    assert(parser.last_rejection == RF_REJECT_LENGTH);
                    for (size_t i = 4u; i < sizeof(stream); ++i)
                        rf_parser_feed(&parser, stream + i, 1u, sink, &out);
                } else {
                    rf_parser_feed(&parser, stream, sizeof(stream), sink, &out);
                }
                /* A complete valid frame must recover without any trailing padding. */
                assert(out.count == 1u && parser.accepted_frames == 1u);
                assert(parser.rejected_frames == 1u);
                assert(parser.last_rejection == RF_REJECT_LENGTH);
                assert(out.length == sizeof(vendor_imu));
                assert(memcmp(out.bytes, vendor_imu, sizeof(vendor_imu)) == 0);
            }
        }
    }
}
static void test_unknown_imu_function_consumes_embedded_raw_frame(void) {
    /* Synthetic unsupported function; its payload is a complete raw frame. */
    uint8_t unknown[4u + sizeof(vendor_imu) + 1u] = {0x7e, 0x23, 0, 0xa5};
    unknown[2] = (uint8_t)sizeof(unknown);
    memcpy(unknown + 4u, vendor_imu, sizeof(vendor_imu));
    checksum(unknown, sizeof(unknown));
    for (unsigned fragmented = 0; fragmented < 2u; ++fragmented) {
        rf_parser parser;
        output out = {0};
        rf_parser_init(&parser, RF_IMU, rf_default_config());
        if (fragmented) {
            for (size_t i = 0; i < sizeof(unknown); ++i)
                rf_parser_feed(&parser, unknown + i, 1u, sink, &out);
        } else {
            rf_parser_feed(&parser, unknown, sizeof(unknown), sink, &out);
        }
        assert(out.count == 0u && parser.accepted_frames == 0u);
        assert(parser.rejected_frames == 0u && parser.used == 0u);
        rf_parser_feed(&parser, vendor_imu, sizeof(vendor_imu), sink, &out);
        assert(out.count == 1u && parser.accepted_frames == 1u);
        assert(memcmp(out.bytes, vendor_imu, sizeof(vendor_imu)) == 0);
    }
}
static void test_imu_signed_acceleration_scaling(void) {
    uint8_t frame[sizeof(vendor_imu)];
    memcpy(frame, vendor_imu, sizeof(frame));
    frame[4] = 0x00; frame[5] = 0x80;
    frame[6] = 0xff; frame[7] = 0x7f;
    frame[8] = 0x01; frame[9] = 0x00;
    frame[10] = 0x00; frame[11] = 0x80;
    frame[12] = 0xff; frame[13] = 0x7f;
    frame[14] = 0xff; frame[15] = 0xff;
    frame[16] = 0x00; frame[17] = 0x80;
    frame[18] = 0xff; frame[19] = 0xff;
    frame[20] = 0xff; frame[21] = 0x7f;
    checksum(frame, sizeof(frame));
    rf_parser parser;
    output out = {0};
    rf_parser_init(&parser, RF_IMU, rf_default_config());
    rf_parser_feed(&parser, frame, sizeof(frame), sink, &out);
    assert(out.count == 1u);
    assert(out.last.value.imu.kind == RF_IMU_RAW);
    assert(out.last.value.imu.raw_xyz[0] == -32768);
    assert(out.last.value.imu.raw_xyz[1] == 32767);
    assert(out.last.value.imu.raw_xyz[2] == 1);
    assert(fabsf(out.last.value.imu.acceleration_m_s2[0] -
                 (-32768.0f / 32767.0f) * 16.0f * RF_STANDARD_GRAVITY) < .0001f);
    assert(fabsf(out.last.value.imu.acceleration_m_s2[1] -
                 16.0f * RF_STANDARD_GRAVITY) < .0001f);
    assert(fabsf(out.last.value.imu.acceleration_m_s2[2] -
                 (16.0f / 32767.0f) * RF_STANDARD_GRAVITY) < .0000001f);
    const int16_t gyro[] = {-32768, 32767, -1};
    const int16_t magnetometer[] = {-32768, -1, 32767};
    for (size_t i = 0; i < 3u; ++i) {
        assert(out.last.value.imu.gyro_raw_xyz[i] == gyro[i]);
        assert(out.last.value.imu.magnetometer_raw_xyz[i] == magnetometer[i]);
        float expected = ((float)gyro[i] / 32767.0f) * 2000.0f * (3.14159265358979323846f / 180.0f);
        assert(fabsf(out.last.value.imu.angular_velocity_rad_s[i] - expected) < .00001f);
    }
}
int main(void) {
    test_imu_golden_profiles();
    test_mixed_imu_profiles_skip_control_frames();
    test_imu_profile_checksum_recovery();
    test_nonfinite_imu_values_reject_and_recover();
    test_finite_imu_values_are_preserved();
    test_wrong_imu_known_lengths_recover();
    test_unknown_imu_function_consumes_embedded_raw_frame();
    test_imu_signed_acceleration_scaling();
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
    puts("portable parser, typed IMU profiles, finite values, byte fragmentation, rejection/recovery, two-channel bridge, bounded ring: PASS");
    return 0;
}
