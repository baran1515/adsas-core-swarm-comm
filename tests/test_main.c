/**
 * @file test_main.c
 * @brief Unit tests for protocol and task allocation (no external framework).
 */
#include <stdio.h>
#include <string.h>

#include "adsas/protocol.h"
#include "adsas/task_alloc.h"

static int g_fail = 0, g_run = 0;

#define CHECK(cond) do { \
    g_run++; \
    if (!(cond)) { g_fail++; printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

static void test_crc(void)
{
    const uint8_t v[] = "123456789";
    CHECK(adsas_crc16(v, 9) == 0x29B1); /* CRC-16/CCITT-FALSE check value */
}

static void test_heartbeat_roundtrip(void)
{
    adsas_heartbeat_t a = { ADSAS_STATE_MISSION, 87, -123456, 7654321, -42, 3, 17, 0x02 }, b;
    uint8_t buf[32];
    CHECK(adsas_heartbeat_write(buf, &a) == 17);
    CHECK(adsas_heartbeat_read(buf, 17, &b) == 0);
    CHECK(memcmp(&a, &b, sizeof(a)) == 0);
    CHECK(adsas_heartbeat_read(buf, 16, &b) != 0); /* wrong length rejected */
}

static size_t make_hb_frame(uint8_t *out, uint16_t seq)
{
    adsas_heartbeat_t hb = { 1, 100, 10, 20, 30, 0, 0, 0 };
    uint8_t pl[17];
    adsas_heartbeat_write(pl, &hb);
    return adsas_frame_build(out, ADSAS_FRAME_MAX, ADSAS_MSG_HEARTBEAT, 2, ADSAS_BROADCAST, seq, 1000, pl, 17);
}

static void test_frame_and_parser(void)
{
    uint8_t fr[ADSAS_FRAME_MAX];
    const size_t n = make_hb_frame(fr, 7);
    CHECK(n == 31); /* 12 header + 17 payload + 2 crc */

    adsas_parser_t p;
    adsas_frame_t f;
    adsas_parser_init(&p);
    int got = 0;
    for (size_t i = 0; i < n; ++i) got += adsas_parser_push(&p, fr[i], &f);
    CHECK(got == 1);
    CHECK(f.hdr.src == 2 && f.hdr.seq == 7 && f.hdr.len == 17);
    CHECK(adsas_msg_type(&f.hdr) == ADSAS_MSG_HEARTBEAT);
}

static void test_parser_resync(void)
{
    uint8_t stream[200];
    size_t k = 0;
    const uint8_t garbage[] = { 0x00, 0xAD, 0x99, 0x13, 0xAD };
    memcpy(stream + k, garbage, sizeof(garbage)); k += sizeof(garbage);

    uint8_t fr[ADSAS_FRAME_MAX];
    size_t n = make_hb_frame(fr, 1);
    memcpy(stream + k, fr, n); k += n;              /* good frame #1   */

    n = make_hb_frame(fr, 2);
    fr[15] ^= 0xFF;                                 /* corrupt payload */
    memcpy(stream + k, fr, n); k += n;

    n = make_hb_frame(fr, 3);
    memcpy(stream + k, fr, n); k += n;              /* good frame #3   */

    adsas_parser_t p;
    adsas_frame_t f;
    adsas_parser_init(&p);
    int got = 0;
    uint16_t seqs[4] = { 0 };
    for (size_t i = 0; i < k; ++i)
        if (adsas_parser_push(&p, stream[i], &f)) seqs[got++] = f.hdr.seq;

    CHECK(got == 2);
    CHECK(seqs[0] == 1 && seqs[1] == 3);
    CHECK(p.crc_errors >= 1);
}

/* every cell 0..N-1 must appear exactly once over all routes */
static int coverage_ok(const adsas_plan_t *pl, uint16_t n_cells)
{
    int seen[ADSAS_MAX_CELLS] = { 0 };
    for (unsigned id = 0; id < ADSAS_MAX_NODES; ++id)
        for (uint16_t i = 0; i < pl->count[id]; ++i) {
            if (pl->route[id][i] >= n_cells) return 0;
            seen[pl->route[id][i]]++;
        }
    for (uint16_t c = 0; c < n_cells; ++c) if (seen[c] != 1) return 0;
    return 1;
}

static void test_initial_plan(void)
{
    const adsas_area_t a = { 8, 8, 200, 0, 0 };
    adsas_plan_t pl;
    const uint16_t mask = (1u << 1) | (1u << 2) | (1u << 3) | (1u << 4);
    CHECK(adsas_plan_initial(&pl, &a, mask) == 0);
    CHECK(coverage_ok(&pl, 64));
    for (unsigned id = 1; id <= 4; ++id) CHECK(pl.count[id] == 16);
    CHECK(adsas_plan_initial(&pl, &a, 0) != 0);
}

static void test_cell_center_serpentine(void)
{
    const adsas_area_t a = { 4, 3, 100, 0, 0 };
    adsas_point_t p0 = adsas_cell_center(&a, 0), p3 = adsas_cell_center(&a, 3), p4 = adsas_cell_center(&a, 4);
    CHECK(p0.x_dm == 50 && p0.y_dm == 50);
    CHECK(p3.x_dm == 350 && p3.y_dm == 50);
    CHECK(p4.x_dm == 350 && p4.y_dm == 150); /* row 1 runs right-to-left */
}

static void test_reassign(void)
{
    const adsas_area_t a = { 8, 8, 200, 0, 0 };
    const uint16_t mask = (1u << 1) | (1u << 2) | (1u << 3) | (1u << 4);
    adsas_plan_t pl, copy;
    adsas_plan_initial(&pl, &a, mask);
    copy = pl;

    const int moved = adsas_plan_reassign(&pl, &a, 4, 5); /* node 4 lost after 5 cells */
    CHECK(moved == 11);
    CHECK(pl.epoch == 1);
    CHECK(!(pl.alive_mask & (1u << 4)));
    CHECK(pl.count[4] == 5);                 /* history kept           */
    CHECK(coverage_ok(&pl, 64));             /* nothing lost/duplicated */

    unsigned total = 0;
    for (unsigned id = 1; id <= 3; ++id) {
        CHECK(pl.count[id] >= 16);
        total += pl.count[id];
        /* original prefix of every survivor is untouched */
        CHECK(memcmp(pl.route[id], copy.route[id], 16 * sizeof(uint16_t)) == 0);
    }
    CHECK(total == 59);

    /* determinism: same inputs => same hash on every node */
    adsas_plan_reassign(&copy, &a, 4, 5);
    CHECK(adsas_plan_hash(&copy) == adsas_plan_hash(&pl));

    /* second failure still keeps full coverage */
    CHECK(adsas_plan_reassign(&pl, &a, 1, 16) >= 0);
    CHECK(coverage_ok(&pl, 64));
    CHECK(pl.epoch == 2);

    /* invalid: node already removed */
    CHECK(adsas_plan_reassign(&pl, &a, 4, 0) == -1);
}

static void test_coordinator(void)
{
    CHECK(adsas_coordinator(0) == -1);
    CHECK(adsas_coordinator((1u << 3) | (1u << 2)) == 2);
}

int main(void)
{
    test_crc();
    test_heartbeat_roundtrip();
    test_frame_and_parser();
    test_parser_resync();
    test_initial_plan();
    test_cell_center_serpentine();
    test_reassign();
    test_coordinator();
    printf("%d checks, %d failed\n", g_run, g_fail);
    return g_fail ? 1 : 0;
}
