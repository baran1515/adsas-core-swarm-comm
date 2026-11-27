/**
 * @file protocol.c
 * @brief Serialisation, CRC and stream parsing for the A.D.S.A.S. protocol.
 */
#include "adsas/protocol.h"

#include <string.h>

/* ---- little-endian helpers ---------------------------------------------- */
static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---- CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection) -------------- */
uint16_t adsas_crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (int i = 0; i < 8; ++i)
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    return crc;
}

uint16_t adsas_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i)
        crc = adsas_crc16_update(crc, data[i]);
    return crc;
}

/* ---- payloads ----------------------------------------------------------- */
size_t adsas_heartbeat_write(uint8_t *o, const adsas_heartbeat_t *hb)
{
    o[0] = hb->state;
    o[1] = hb->battery;
    put_u32(o + 2, (uint32_t)hb->x_dm);
    put_u32(o + 6, (uint32_t)hb->y_dm);
    put_u16(o + 10, (uint16_t)hb->alt_dm);
    put_u16(o + 12, hb->epoch);
    put_u16(o + 14, hb->cells_done);
    o[16] = hb->flags;
    return sizeof(adsas_heartbeat_t);
}

int adsas_heartbeat_read(const uint8_t *in, size_t len, adsas_heartbeat_t *hb)
{
    if (len != sizeof(adsas_heartbeat_t)) return -1;
    hb->state      = in[0];
    hb->battery    = in[1];
    hb->x_dm       = (int32_t)get_u32(in + 2);
    hb->y_dm       = (int32_t)get_u32(in + 6);
    hb->alt_dm     = (int16_t)get_u16(in + 10);
    hb->epoch      = get_u16(in + 12);
    hb->cells_done = get_u16(in + 14);
    hb->flags      = in[16];
    return 0;
}

size_t adsas_node_down_write(uint8_t *o, const adsas_node_down_t *nd)
{
    o[0] = nd->node_id;
    o[1] = nd->reason;
    put_u16(o + 2, nd->new_epoch);
    put_u16(o + 4, nd->cells_done);
    put_u32(o + 6, nd->last_seen_ms);
    return sizeof(adsas_node_down_t);
}

int adsas_node_down_read(const uint8_t *in, size_t len, adsas_node_down_t *nd)
{
    if (len != sizeof(adsas_node_down_t)) return -1;
    nd->node_id      = in[0];
    nd->reason       = in[1];
    nd->new_epoch    = get_u16(in + 2);
    nd->cells_done   = get_u16(in + 4);
    nd->last_seen_ms = get_u32(in + 6);
    return 0;
}

size_t adsas_task_sync_write(uint8_t *o, const adsas_task_sync_t *ts)
{
    put_u16(o, ts->epoch);
    put_u16(o + 2, ts->alive_mask);
    put_u32(o + 4, ts->plan_hash);
    return sizeof(adsas_task_sync_t);
}

int adsas_task_sync_read(const uint8_t *in, size_t len, adsas_task_sync_t *ts)
{
    if (len != sizeof(adsas_task_sync_t)) return -1;
    ts->epoch      = get_u16(in);
    ts->alive_mask = get_u16(in + 2);
    ts->plan_hash  = get_u32(in + 4);
    return 0;
}

/* ---- framing ------------------------------------------------------------ */
size_t adsas_frame_build(uint8_t *out, size_t cap, uint8_t type, uint8_t src, uint8_t dst,
                         uint16_t seq, uint32_t t_ms, const uint8_t *payload, uint16_t len)
{
    const size_t total = ADSAS_HEADER_SIZE + (size_t)len + ADSAS_CRC_SIZE;
    if (len > ADSAS_MAX_PAYLOAD || cap < total) return 0;

    out[0] = ADSAS_MAGIC;
    out[1] = (uint8_t)((ADSAS_VERSION << 4) | (type & 0x0Fu));
    out[2] = src;
    out[3] = dst;
    put_u16(out + 4, seq);
    put_u16(out + 6, len);
    put_u32(out + 8, t_ms);
    if (len) memcpy(out + ADSAS_HEADER_SIZE, payload, len);

    put_u16(out + ADSAS_HEADER_SIZE + len, adsas_crc16(out, ADSAS_HEADER_SIZE + len));
    return total;
}

void adsas_parser_init(adsas_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

/* Drop the first byte and shift buffer to the next magic byte (resync). */
static void parser_resync(adsas_parser_t *p)
{
    size_t i = 1;
    while (i < p->n && p->buf[i] != ADSAS_MAGIC) ++i;
    p->bytes_dropped += (uint32_t)i;
    memmove(p->buf, p->buf + i, p->n - i);
    p->n -= i;
    p->need = 0;
}

/* Try to complete a frame from buffered bytes. Returns 1 on success. */
static int parser_try(adsas_parser_t *p, adsas_frame_t *out)
{
    for (;;) {
        if (p->n == 0) return 0;
        if (p->buf[0] != ADSAS_MAGIC) { parser_resync(p); continue; }

        if (p->n < ADSAS_HEADER_SIZE) return 0;
        if (p->need == 0) {
            const uint16_t len = get_u16(p->buf + 6);
            if ((p->buf[1] >> 4) != ADSAS_VERSION || len > ADSAS_MAX_PAYLOAD) {
                parser_resync(p);
                continue;
            }
            p->need = ADSAS_HEADER_SIZE + len + ADSAS_CRC_SIZE;
        }
        if (p->n < p->need) return 0;

        const size_t body = p->need - ADSAS_CRC_SIZE;
        if (adsas_crc16(p->buf, body) != get_u16(p->buf + body)) {
            p->crc_errors++;
            parser_resync(p);
            continue;
        }

        out->hdr.magic    = p->buf[0];
        out->hdr.ver_type = p->buf[1];
        out->hdr.src      = p->buf[2];
        out->hdr.dst      = p->buf[3];
        out->hdr.seq      = get_u16(p->buf + 4);
        out->hdr.len      = get_u16(p->buf + 6);
        out->hdr.t_ms     = get_u32(p->buf + 8);
        memcpy(out->payload, p->buf + ADSAS_HEADER_SIZE, out->hdr.len);

        const size_t used = p->need;
        memmove(p->buf, p->buf + used, p->n - used);
        p->n -= used;
        p->need = 0;
        p->frames_ok++;
        return 1;
    }
}

int adsas_parser_push(adsas_parser_t *p, uint8_t byte, adsas_frame_t *out)
{
    if (p->n >= sizeof(p->buf)) parser_resync(p); /* cannot happen with valid lengths */
    p->buf[p->n++] = byte;
    return parser_try(p, out);
}
