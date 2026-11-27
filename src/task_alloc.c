/**
 * @file task_alloc.c
 * @brief Deterministic coverage allocation / re-allocation.
 */
#include "adsas/task_alloc.h"

#include <math.h>
#include <string.h>

uint16_t adsas_area_cells(const adsas_area_t *a)
{
    return (uint16_t)(a->cols * a->rows);
}

adsas_point_t adsas_cell_center(const adsas_area_t *a, uint16_t idx)
{
    const uint16_t row = (uint16_t)(idx / a->cols);
    uint16_t col = (uint16_t)(idx % a->cols);
    if (row & 1u) col = (uint16_t)(a->cols - 1u - col); /* serpentine rows */

    adsas_point_t p;
    p.x_dm = a->origin_x_dm + (int32_t)col * a->cell_dm + a->cell_dm / 2;
    p.y_dm = a->origin_y_dm + (int32_t)row * a->cell_dm + a->cell_dm / 2;
    return p;
}

int adsas_coordinator(uint16_t alive_mask)
{
    for (int i = 0; i < (int)ADSAS_MAX_NODES; ++i)
        if (alive_mask & (1u << i)) return i;
    return -1;
}

static unsigned popcount16(uint16_t v)
{
    unsigned c = 0;
    while (v) { v &= (uint16_t)(v - 1u); ++c; }
    return c;
}

int adsas_plan_initial(adsas_plan_t *plan, const adsas_area_t *a, uint16_t alive_mask)
{
    const unsigned k = popcount16(alive_mask);
    const uint16_t n = adsas_area_cells(a);
    if (k == 0 || n == 0 || n > ADSAS_MAX_CELLS) return -1;

    memset(plan, 0, sizeof(*plan));
    plan->alive_mask = alive_mask;

    unsigned j = 0;
    for (unsigned id = 0; id < ADSAS_MAX_NODES; ++id) {
        if (!(alive_mask & (1u << id))) continue;
        const uint16_t start = (uint16_t)((j * n) / k);
        const uint16_t end   = (uint16_t)(((j + 1) * n) / k);
        for (uint16_t c = start; c < end; ++c)
            plan->route[id][plan->count[id]++] = c;
        ++j;
    }
    return 0;
}

static double dist_dm(adsas_point_t p, adsas_point_t q)
{
    const double dx = (double)p.x_dm - (double)q.x_dm;
    const double dy = (double)p.y_dm - (double)q.y_dm;
    return sqrt(dx * dx + dy * dy);
}

int adsas_plan_reassign(adsas_plan_t *plan, const adsas_area_t *a, uint8_t failed_id,
                        uint16_t failed_done)
{
    if (failed_id >= ADSAS_MAX_NODES || !(plan->alive_mask & (1u << failed_id))) return -1;

    const uint16_t remaining_mask = (uint16_t)(plan->alive_mask & ~(1u << failed_id));
    if (remaining_mask == 0) return -1;

    uint16_t orphans[ADSAS_MAX_CELLS];
    uint16_t n_orphans = 0;
    const uint16_t total = plan->count[failed_id];
    for (uint16_t i = (failed_done < total ? failed_done : total); i < total; ++i)
        orphans[n_orphans++] = plan->route[failed_id][i];

    /* The failed node keeps only what it actually finished (history). */
    plan->count[failed_id] = (failed_done < total) ? failed_done : total;
    plan->alive_mask = remaining_mask;
    plan->epoch++;

    for (uint16_t o = 0; o < n_orphans; ++o) {
        const adsas_point_t cell = adsas_cell_center(a, orphans[o]);
        int best = -1;
        double best_cost = 0.0;

        for (unsigned id = 0; id < ADSAS_MAX_NODES; ++id) {
            if (!(remaining_mask & (1u << id))) continue;
            if (plan->count[id] >= ADSAS_MAX_CELLS) continue;

            const adsas_point_t tail = plan->count[id]
                ? adsas_cell_center(a, plan->route[id][plan->count[id] - 1u])
                : (adsas_point_t){ a->origin_x_dm, a->origin_y_dm };

            const double cost = dist_dm(tail, cell)
                              + (double)ADSAS_LOAD_WEIGHT * plan->count[id] * a->cell_dm;

            /* strict '<' + ascending id => deterministic tie-break */
            if (best < 0 || cost < best_cost) { best = (int)id; best_cost = cost; }
        }
        if (best < 0) return -1;
        plan->route[best][plan->count[best]++] = orphans[o];
    }
    return (int)n_orphans;
}

uint32_t adsas_plan_hash(const adsas_plan_t *plan)
{
    uint32_t h = 2166136261u;
#define FNV_BYTE(b) do { h ^= (uint8_t)(b); h *= 16777619u; } while (0)
    FNV_BYTE(plan->epoch); FNV_BYTE(plan->epoch >> 8);
    FNV_BYTE(plan->alive_mask); FNV_BYTE(plan->alive_mask >> 8);
    for (unsigned id = 0; id < ADSAS_MAX_NODES; ++id) {
        FNV_BYTE(plan->count[id]); FNV_BYTE(plan->count[id] >> 8);
        for (uint16_t i = 0; i < plan->count[id]; ++i) {
            FNV_BYTE(plan->route[id][i]);
            FNV_BYTE(plan->route[id][i] >> 8);
        }
    }
#undef FNV_BYTE
    return h;
}
