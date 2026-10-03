/* Original implementation, Copyright (c) 2026 RefereeLink contributors. MIT. */
#include "rf_telemetry.h"
#include <float.h>
#include <limits.h>
#include <math.h>
#include <string.h>

_Static_assert((RF_RING_CAPACITY & (RF_RING_CAPACITY - 1u)) == 0u,
               "ring size must be power of two");
_Static_assert(CHAR_BIT == 8 && sizeof(float) == sizeof(uint32_t) &&
               FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MIN_EXP == -125 && FLT_MAX_EXP == 128,
               "IMU float decoding requires IEEE-754 binary32");

static uint16_t le16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static int16_t signed_le16(const uint8_t *p) {
    uint16_t u = le16(p);
    int32_t signed_value = u >= 32768u ? (int32_t)u - 65536 : (int32_t)u;
    return (int16_t)signed_value;
}
static int imu_float_layout_supported(void) {
    /* Binary32 dimensions alone do not establish native integer/float byte order. */
    const float one = 1.0f, negative_zero = -0.0f;
    uint32_t one_bits, negative_zero_bits;
    memcpy(&one_bits, &one, sizeof(one_bits));
    memcpy(&negative_zero_bits, &negative_zero, sizeof(negative_zero_bits));
    return one_bits == UINT32_C(0x3f800000) &&
           negative_zero_bits == UINT32_C(0x80000000);
}
static int finite_le_floats(const uint8_t *p, float *values, size_t count) {
    if (!imu_float_layout_supported()) return 0;
    for (size_t i = 0; i < count; ++i) {
        const uint8_t *bytes = p + 4u * i;
        uint32_t bits = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                        ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
        /* Reject NaN and either infinity before creating a float object. */
        if ((bits & UINT32_C(0x7f800000)) == UINT32_C(0x7f800000)) return 0;
        memcpy(values + i, &bits, sizeof(bits));
    }
    return 1;
}
static size_t imu_expected_length(uint8_t function) {
    switch (function) {
        case RF_IMU_RAW: return 23u;
        case RF_IMU_QUATERNION: return 21u;
        case RF_IMU_EULER: return 17u;
        case RF_IMU_BAROMETER: return 21u;
        case 0x01u: return 8u; /* Firmware version response, ignored. */
        case 0x81u: return 7u; /* Command status response, ignored. */
        default: return 0u;
    }
}
rf_config rf_default_config(void) {
    rf_config c = {5u, 0.2f, 30.0f};
    return c;
}
void rf_parser_init(rf_parser *p, rf_channel channel, rf_config config) {
    memset(p, 0, sizeof(*p));
    p->channel = channel;
    p->config = config;
}
static void discard(rf_parser *p, size_t amount) {
    p->used -= amount;
    memmove(p->bytes, p->bytes + amount, p->used);
}
static void reject(rf_parser *p, rf_rejection reason) {
    p->rejected_frames++;
    p->last_rejection = reason;
}
static void parse_available(rf_parser *p, rf_frame_sink sink, void *context) {
    const uint8_t h0 = p->channel == RF_UWB ? (uint8_t)'m' : 0x7eu;
    const uint8_t h1 = p->channel == RF_UWB ? (uint8_t)'r' : 0x23u;
    while (p->used >= 2u) {
        if (p->bytes[0] != h0 || p->bytes[1] != h1) {
            discard(p, 1u);
            continue;
        }
        if (p->used < 4u) return;
        size_t length = p->channel == RF_UWB ? 16u : p->bytes[2];
        size_t expected = p->channel == RF_IMU ? imu_expected_length(p->bytes[3]) : 0u;
        /* Known IMU profiles have fixed lengths, even when fragmented. */
        if (length < 5u || length > RF_FRAME_CAPACITY ||
                (expected && length != expected)) {
            reject(p, RF_REJECT_LENGTH);
            discard(p, 1u);
            continue;
        }
        if (p->used < length) return;
        rf_sample sample;
        memset(&sample, 0, sizeof(sample));
        sample.channel = p->channel;
        rf_rejection reason = RF_REJECT_NONE;
        if (p->channel == RF_UWB) {
            if (p->bytes[2] != 2u || !((p->bytes[14] == 13u && p->bytes[15] == 10u) ||
                    (p->bytes[14] == 10u && p->bytes[15] == 13u))) {
                reject(p, RF_REJECT_FRAMING);
                discard(p, 1u);
                continue;
            }
            sample.value.uwb.tag_id = p->bytes[3];
            sample.value.uwb.sequence = le16(p->bytes + 4);
            if (p->config.tag_id && sample.value.uwb.tag_id != p->config.tag_id)
                reason = RF_REJECT_TAG;
            if (le16(p->bytes + 6) != le16(p->bytes + 12))
                reason = RF_REJECT_THREE_ANCHOR;
            for (size_t i = 0; i < 3u; ++i) {
                float distance = (float)le16(p->bytes + 6u + 2u * i) * 0.01f;
                sample.value.uwb.ranges_m[i] = distance;
                if (distance < p->config.minimum_range_m || distance > p->config.maximum_range_m)
                    reason = RF_REJECT_RANGE;
            }
        } else {
            uint8_t sum = 0;
            for (size_t i = 0; i < length - 1u; ++i) sum = (uint8_t)(sum + p->bytes[i]);
            if (sum != p->bytes[length - 1u]) {
                reject(p, RF_REJECT_CHECKSUM);
                discard(p, 1u);
                continue;
            }
            rf_imu_sample *imu = &sample.value.imu;
            switch (p->bytes[3]) {
                case RF_IMU_RAW:
                    imu->kind = RF_IMU_RAW;
                    for (size_t i = 0; i < 3u; ++i) {
                        int16_t raw = signed_le16(p->bytes + 4u + 2u * i);
                        int16_t gyro = signed_le16(p->bytes + 10u + 2u * i);
                        imu->raw_xyz[i] = raw;
                        imu->acceleration_m_s2[i] =
                            (float)raw * (16.0f / 32767.0f) * RF_STANDARD_GRAVITY;
                        imu->gyro_raw_xyz[i] = gyro;
                        imu->angular_velocity_rad_s[i] = (float)gyro *
                            (2000.0f / 32767.0f) * (3.14159265358979323846f / 180.0f);
                        imu->magnetometer_raw_xyz[i] = signed_le16(p->bytes + 16u + 2u * i);
                    }
                    break;
                case RF_IMU_QUATERNION:
                    imu->kind = RF_IMU_QUATERNION;
                    if (!finite_le_floats(p->bytes + 4u, imu->quaternion_wxyz, 4u))
                        reason = RF_REJECT_VALUE;
                    break;
                case RF_IMU_EULER:
                    imu->kind = RF_IMU_EULER;
                    if (!finite_le_floats(p->bytes + 4u, imu->euler_rad, 3u))
                        reason = RF_REJECT_VALUE;
                    break;
                case RF_IMU_BAROMETER: {
                    float values[4];
                    imu->kind = RF_IMU_BAROMETER;
                    if (!finite_le_floats(p->bytes + 4u, values, 4u)) {
                        reason = RF_REJECT_VALUE;
                    } else {
                        imu->barometer.relative_height_m = values[0];
                        imu->barometer.temperature_c = values[1];
                        imu->barometer.pressure_pa = values[2];
                        imu->barometer.reference_pressure_pa = values[3];
                    }
                    break;
                }
                default: discard(p, length); continue;
            }
        }
        if (reason != RF_REJECT_NONE) reject(p, reason);
        else {
            p->accepted_frames++;
            if (sink) sink(context, p->channel, p->bytes, length, &sample);
        }
        discard(p, length);
    }
}
void rf_parser_feed(rf_parser *p, const uint8_t *data, size_t length,
                    rf_frame_sink sink, void *context) {
    if (!p || (!data && length) || (p->channel != RF_UWB && p->channel != RF_IMU)) return;
    for (size_t i = 0; i < length; ++i) {
        if (p->used == RF_FRAME_CAPACITY) {
            reject(p, RF_REJECT_LENGTH);
            discard(p, 1u);
        }
        p->bytes[p->used++] = data[i];
        parse_available(p, sink, context);
    }
}
int rf_gateway_init(rf_gateway *g, rf_config config, rf_frame_sink sink, void *context) {
    if (!g || !imu_float_layout_supported() ||
            !isfinite(config.minimum_range_m) || !isfinite(config.maximum_range_m) ||
            config.minimum_range_m < 0.2f || config.maximum_range_m > 30.0f ||
            config.maximum_range_m < config.minimum_range_m)
        return 0;
    memset(g, 0, sizeof(*g));
    g->sink = sink;
    g->sink_context = context;
    for (size_t i = 0; i < 2u; ++i) {
        atomic_init(&g->input[i].head, 0u);
        atomic_init(&g->input[i].tail, 0u);
        atomic_init(&g->input[i].overruns, 0u);
        if (!atomic_is_lock_free(&g->input[i].head) || !atomic_is_lock_free(&g->input[i].tail) ||
                !atomic_is_lock_free(&g->input[i].overruns)) return 0;
        rf_parser_init(&g->parser[i], (rf_channel)i, config);
    }
    return 1;
}
size_t rf_gateway_receive(rf_gateway *g, rf_channel channel, const uint8_t *bytes, size_t length) {
    if (!g || (!bytes && length) || (channel != RF_UWB && channel != RF_IMU)) return 0u;
    rf_ring *ring = &g->input[channel];
    size_t accepted = 0;
    for (size_t i = 0; i < length; ++i) {
        uint_least16_t head = atomic_load_explicit(&ring->head, memory_order_relaxed);
        uint_least16_t next = (uint_least16_t)((head + 1u) & (RF_RING_CAPACITY - 1u));
        if (next == atomic_load_explicit(&ring->tail, memory_order_acquire)) {
            uint_least32_t old = atomic_load_explicit(&ring->overruns, memory_order_relaxed);
            atomic_store_explicit(&ring->overruns, old + 1u, memory_order_release);
            continue;
        }
        ring->bytes[head] = bytes[i];
        atomic_store_explicit(&ring->head, next, memory_order_release);
        accepted++;
    }
    return accepted;
}
size_t rf_gateway_process(rf_gateway *g, size_t budget_per_channel) {
    if (!g) return 0u;
    size_t count = 0;
    for (size_t channel = 0; channel < 2u; ++channel) {
        rf_ring *ring = &g->input[channel];
        for (size_t i = 0; i < budget_per_channel; ++i) {
            uint_least32_t overruns = atomic_load_explicit(&ring->overruns, memory_order_acquire);
            if (overruns != g->observed_overruns[channel]) {
                uint_least16_t head = atomic_load_explicit(&ring->head, memory_order_acquire);
                uint_least16_t old_tail = atomic_load_explicit(&ring->tail, memory_order_relaxed);
                g->discarded_overflow_bytes[channel] +=
                    (uint32_t)((head - old_tail) & (RF_RING_CAPACITY - 1u));
                atomic_store_explicit(&ring->tail, head, memory_order_release);
                g->observed_overruns[channel] = overruns;
                g->parser[channel].used = 0;
                break;
            }
            uint_least16_t tail = atomic_load_explicit(&ring->tail, memory_order_relaxed);
            if (tail == atomic_load_explicit(&ring->head, memory_order_acquire)) break;
            uint8_t byte = ring->bytes[tail];
            atomic_store_explicit(&ring->tail,
                (uint_least16_t)((tail + 1u) & (RF_RING_CAPACITY - 1u)), memory_order_release);
            rf_parser_feed(&g->parser[channel], &byte, 1u, g->sink, g->sink_context);
            count++;
        }
    }
    return count;
}
uint32_t rf_gateway_overruns(const rf_gateway *g, rf_channel channel) {
    if (!g || (channel != RF_UWB && channel != RF_IMU)) return 0u;
    return atomic_load_explicit(&g->input[channel].overruns, memory_order_acquire);
}
