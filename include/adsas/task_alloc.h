/**
 * @file task_alloc.h
 * @brief Area-coverage task allocation and re-allocation after node loss.
 *
 * The mission area is a rectangular grid of square cells. Cells are numbered
 * in boustrophedon ("lawn-mower") order so consecutive indices are always
 * neighbours. Every node holds the same plan and runs the same deterministic
 * functions, so given the same inputs all nodes reach the same plan without a
 * central ground station. A plan hash is exchanged to detect divergence.
 */
#ifndef ADSAS_TASK_ALLOC_H
#define ADSAS_TASK_ALLOC_H

#include <stdint.h>

#include "adsas/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADSAS_MAX_CELLS 256u

typedef struct {
    int32_t x_dm;
    int32_t y_dm;
} adsas_point_t;

typedef struct {
    uint8_t cols;
    uint8_t rows;
    int32_t cell_dm;      /**< edge length of one cell, decimetres     */
    int32_t origin_x_dm;  /**< south-west corner of the area           */
    int32_t origin_y_dm;
} adsas_area_t;

typedef struct {
    uint16_t epoch;
    uint16_t alive_mask;                           /**< bit i => node i alive */
    uint16_t count[ADSAS_MAX_NODES];               /**< route length per node */
    uint16_t route[ADSAS_MAX_NODES][ADSAS_MAX_CELLS]; /**< ordered cell list   */
} adsas_plan_t;

/** Tunable weight of the load-balancing term (dimensionless, per cell). */
#define ADSAS_LOAD_WEIGHT 1

uint16_t adsas_area_cells(const adsas_area_t *a);

/** Centre point of cell @p idx (boustrophedon index). */
adsas_point_t adsas_cell_center(const adsas_area_t *a, uint16_t idx);

/**
 * Initial plan: the boustrophedon sequence is cut into contiguous, nearly
 * equal segments, one per alive node (ascending node id).
 * @return 0 on success, -1 if no node is alive or area too large.
 */
int adsas_plan_initial(adsas_plan_t *plan, const adsas_area_t *a, uint16_t alive_mask);

/**
 * Re-allocate the unfinished cells of @p failed_id to the remaining nodes.
 *
 * For every orphan cell (in route order) the node with minimum cost is chosen:
 *
 *     cost(n) = |tail(n) - cell| + ADSAS_LOAD_WEIGHT * count(n) * cell_dm
 *
 * where tail(n) is the last waypoint of node n's route. Only plan data is used
 * (no live positions), which keeps the result identical on every node.
 *
 * @param failed_done cells of the failed node already completed (from its
 *                    last heartbeat, distributed by the coordinator).
 * @return number of cells moved, or -1 on invalid input.
 */
int adsas_plan_reassign(adsas_plan_t *plan, const adsas_area_t *a, uint8_t failed_id,
                        uint16_t failed_done);

/** FNV-1a hash over the plan contents (epoch, mask, routes). */
uint32_t adsas_plan_hash(const adsas_plan_t *plan);

/** Lowest alive node id = coordinator, or -1 if none. */
int adsas_coordinator(uint16_t alive_mask);

#ifdef __cplusplus
}
#endif

#endif /* ADSAS_TASK_ALLOC_H */
