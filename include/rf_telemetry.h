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
    RF_REJECT_TAG
} rf_rejection;

typedef struct {
    uint8_t tag_id;
    uint16_t sequence;
    float ranges_m[3];
} rf_uwb_sample;
typedef struct {
    int16_t raw_xyz[3];
    float acceleration_m_s2[3]; /* Sensor X/Y/Z, includes gravity. */
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
/* Returns zero if the selected C11 atomics are not lock-free on this target. */
int rf_gateway_init(rf_gateway *gateway, rf_config config,
                    rf_frame_sink sink, void *context);
/* ISR-safe SPSC enqueue; returns accepted byte count; drops newest when full. */
size_t rf_gateway_receive(rf_gateway *gateway, rf_channel channel,
                          const uint8_t *bytes, size_t length);
/* Process at most budget bytes per channel, bounding time spent in one call. */
size_t rf_gateway_process(rf_gateway *gateway, size_t budget_per_channel);
uint32_t rf_gateway_overruns(const rf_gateway *gateway, rf_channel channel);
#endif
