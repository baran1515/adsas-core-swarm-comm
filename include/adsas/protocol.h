/**
 * @file protocol.h
 * @brief A.D.S.A.S. Core - low-overhead binary inter-UAV protocol.
 *
 * Wire format (all multi-byte fields little-endian):
 *
 *   +-------+---------+-----+-----+-----+-----+------+-----------+--------+
 *   | magic | ver|typ | src | dst | seq | len | t_ms | payload   | CRC16  |
 *   |  1 B  |   1 B   | 1 B | 1 B | 2 B | 2 B | 4 B  | len bytes | 2 B    |
 *   +-------+---------+-----+-----+-----+-----+------+-----------+--------+
 *
 * Header = 12 bytes, trailer = 2 bytes. A heartbeat frame is 12+17+2 = 31 B.
 * CRC16-CCITT (poly 0x1021, init 0xFFFF) covers header + payload.
 *
 * The packed structs below document the exact wire layout and are size-checked
 * at compile time. Serialisation is done with explicit little-endian helpers so
 * the code is portable to big-endian targets as well.
 */
#ifndef ADSAS_PROTOCOL_H
#define ADSAS_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADSAS_MAGIC        0xADu
#define ADSAS_VERSION      1u
#define ADSAS_BROADCAST    0xFFu
#define ADSAS_MAX_NODES    16u
#define ADSAS_MAX_PAYLOAD  64u
#define ADSAS_HEADER_SIZE  12u
#define ADSAS_CRC_SIZE     2u
#define ADSAS_FRAME_MAX    (ADSAS_HEADER_SIZE + ADSAS_MAX_PAYLOAD + ADSAS_CRC_SIZE)

typedef enum {
    ADSAS_MSG_HEARTBEAT = 1, /**< periodic state + position, every node      */
    ADSAS_MSG_NODE_DOWN = 2, /**< coordinator announces a failed node         */
    ADSAS_MSG_TASK_SYNC = 3  /**< coordinator publishes plan hash for checks  */
} adsas_msg_type_t;

typedef enum {
    ADSAS_STATE_IDLE      = 0,
    ADSAS_STATE_MISSION   = 1,
    ADSAS_STATE_RTL       = 2,
    ADSAS_STATE_DEGRADED  = 3
} adsas_node_state_t;

typedef enum {
    ADSAS_DOWN_TIMEOUT = 1, /**< heartbeat timeout (jamming / link loss)     */
    ADSAS_DOWN_LINK    = 2  /**< TCP connection closed                        */
} adsas_down_reason_t;

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic;
    uint8_t  ver_type;   /**< high nibble: version, low nibble: msg type   */
    uint8_t  src;
    uint8_t  dst;
    uint16_t seq;
    uint16_t len;
    uint32_t t_ms;
} adsas_header_t;

typedef struct {
    uint8_t  state;      /**< adsas_node_state_t                           */
    uint8_t  battery;    /**< percent                                      */
    int32_t  x_dm;       /**< local East, decimetres                       */
    int32_t  y_dm;       /**< local North, decimetres                      */
    int16_t  alt_dm;     /**< altitude AGL, decimetres                     */
    uint16_t epoch;      /**< plan epoch the node is executing             */
    uint16_t cells_done; /**< progress index inside its own route          */
    uint8_t  flags;      /**< bit0: GPS degraded, bit1: ext-pos source     */
} adsas_heartbeat_t;

typedef struct {
    uint8_t  node_id;
    uint8_t  reason;     /**< adsas_down_reason_t                          */
    uint16_t new_epoch;
    uint16_t cells_done; /**< last progress reported by the failed node    */
    uint32_t last_seen_ms;
} adsas_node_down_t;

typedef struct {
    uint16_t epoch;
    uint16_t alive_mask;
    uint32_t plan_hash;
} adsas_task_sync_t;
#pragma pack(pop)

_Static_assert(sizeof(adsas_header_t) == ADSAS_HEADER_SIZE, "header must be 12 bytes");
_Static_assert(sizeof(adsas_heartbeat_t) == 17, "heartbeat must be 17 bytes");
_Static_assert(sizeof(adsas_node_down_t) == 10, "node_down must be 10 bytes");
_Static_assert(sizeof(adsas_task_sync_t) == 8, "task_sync must be 8 bytes");

/** Decoded frame (header + raw payload bytes). */
typedef struct {
    adsas_header_t hdr;
    uint8_t        payload[ADSAS_MAX_PAYLOAD];
} adsas_frame_t;

static inline uint8_t adsas_msg_type(const adsas_header_t *h) { return (uint8_t)(h->ver_type & 0x0Fu); }
static inline uint8_t adsas_msg_ver(const adsas_header_t *h)  { return (uint8_t)(h->ver_type >> 4); }

/* ---- CRC ---------------------------------------------------------------- */
uint16_t adsas_crc16(const uint8_t *data, size_t len);
uint16_t adsas_crc16_update(uint16_t crc, uint8_t byte);

/* ---- payload (de)serialisation ------------------------------------------ */
size_t adsas_heartbeat_write(uint8_t *out, const adsas_heartbeat_t *hb);
int    adsas_heartbeat_read(const uint8_t *in, size_t len, adsas_heartbeat_t *hb);
size_t adsas_node_down_write(uint8_t *out, const adsas_node_down_t *nd);
int    adsas_node_down_read(const uint8_t *in, size_t len, adsas_node_down_t *nd);
size_t adsas_task_sync_write(uint8_t *out, const adsas_task_sync_t *ts);
int    adsas_task_sync_read(const uint8_t *in, size_t len, adsas_task_sync_t *ts);

/* ---- framing ------------------------------------------------------------ */
/**
 * Build a complete frame into @p out.
 * @return number of bytes written, or 0 if @p cap is too small / len too big.
 */
size_t adsas_frame_build(uint8_t *out, size_t cap, uint8_t type, uint8_t src, uint8_t dst,
                         uint16_t seq, uint32_t t_ms, const uint8_t *payload, uint16_t len);

/** Byte-oriented stream parser; resynchronises on the magic byte. */
typedef struct {
    uint8_t  buf[ADSAS_FRAME_MAX];
    size_t   n;
    size_t   need;        /**< total frame length once header is known     */
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t bytes_dropped;
} adsas_parser_t;

void adsas_parser_init(adsas_parser_t *p);
/**
 * Feed one byte.
 * @return 1 when a valid frame was completed and stored in @p out, else 0.
 */
int  adsas_parser_push(adsas_parser_t *p, uint8_t byte, adsas_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ADSAS_PROTOCOL_H */
