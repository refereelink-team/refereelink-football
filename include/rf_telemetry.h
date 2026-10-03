/* Original implementation, Copyright (c) 2026 RefereeLink contributors. MIT. */
#ifndef RF_TELEMETRY_H
#define RF_TELEMETRY_H
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#define RF_FRAME_CAPACITY 68u
#define RF_RING_CAPACITY 512u
#define RF_STANDARD_GRAVITY 9.80665f

typedef enum { RF_UWB = 0, RF_IMU = 1 } rf_channel;
typedef enum {
    RF_REJECT_NONE = 0, RF_REJECT_FRAMING, RF_REJECT_LENGTH,
    RF_REJECT_CHECKSUM, RF_REJECT_RANGE, RF_REJECT_THREE_ANCHOR,
    RF_REJECT_TAG, RF_REJECT_VALUE
} rf_rejection;

typedef enum {
    RF_IMU_RAW = 0x04, RF_IMU_QUATERNION = 0x16,
    RF_IMU_EULER = 0x26, RF_IMU_BAROMETER = 0x32
} rf_imu_kind;

typedef struct {
    uint8_t tag_id;
    uint16_t sequence;
    float ranges_m[3];
} rf_uwb_sample;
typedef struct {
    float relative_height_m; /* Relative to the sensor position at startup. */
    float temperature_c;
    float pressure_pa;
    float reference_pressure_pa; /* Reference pressure at startup. */
} rf_imu_barometer;
typedef struct {
    /* Only the fields belonging to kind are valid; other fields are zero. */
    /* RF_IMU_RAW: sensor X/Y/Z; acceleration includes gravity. */
    int16_t raw_xyz[3];
    float acceleration_m_s2[3];
    rf_imu_kind kind;
    int16_t gyro_raw_xyz[3];
    float angular_velocity_rad_s[3];
    int16_t magnetometer_raw_xyz[3]; /* Counts; no physical magnetic unit inferred. */
    float quaternion_wxyz[4]; /* RF_IMU_QUATERNION: w, x, y, z. */
    float euler_rad[3]; /* RF_IMU_EULER: roll, pitch, yaw in radians. */
    rf_imu_barometer barometer; /* RF_IMU_BAROMETER; 10-axis sensor only. */
} rf_imu_sample;
typedef struct {
    rf_channel channel;
    union { rf_uwb_sample uwb; rf_imu_sample imu; } value;
} rf_sample;
typedef struct {
    uint8_t tag_id;             /* Zero accepts any tag, otherwise selected tag. */
    float minimum_range_m;
    float maximum_range_m;
} rf_config;

/* Called in the main-loop consumer, never in the UART interrupt. */
typedef void (*rf_frame_sink)(void *context, rf_channel channel,
                             const uint8_t *frame, size_t length,
                             const rf_sample *sample);
typedef struct {
    uint8_t bytes[RF_FRAME_CAPACITY];
    size_t used;
    rf_channel channel;
    rf_config config;
    uint32_t rejected_frames;
    uint32_t accepted_frames;
    rf_rejection last_rejection;
} rf_parser;

void rf_parser_init(rf_parser *parser, rf_channel channel, rf_config config);
void rf_parser_feed(rf_parser *parser, const uint8_t *data, size_t length,
                    rf_frame_sink sink, void *context);

/* One producer ISR and one main-loop consumer per ring. */
typedef struct {
    uint8_t bytes[RF_RING_CAPACITY];
    atomic_uint_least16_t head;
    atomic_uint_least16_t tail;
    atomic_uint_least32_t overruns;
} rf_ring;
typedef struct {
    rf_ring input[2];
    rf_parser parser[2];
    uint32_t observed_overruns[2];
    uint32_t discarded_overflow_bytes[2];
    rf_frame_sink sink;
    void *sink_context;
} rf_gateway;

rf_config rf_default_config(void);
/* Returns zero for invalid configuration, unsupported float layout or non-lock-free atomics. */
int rf_gateway_init(rf_gateway *gateway, rf_config config,
                    rf_frame_sink sink, void *context);
/* ISR-safe SPSC enqueue; returns accepted byte count; drops newest when full. */
size_t rf_gateway_receive(rf_gateway *gateway, rf_channel channel,
                          const uint8_t *bytes, size_t length);
/* Process at most budget bytes per channel, bounding time spent in one call. */
size_t rf_gateway_process(rf_gateway *gateway, size_t budget_per_channel);
uint32_t rf_gateway_overruns(const rf_gateway *gateway, rf_channel channel);
#endif
