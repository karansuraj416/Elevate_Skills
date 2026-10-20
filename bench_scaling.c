/* bench_scaling.c — memory and execution-time figures for one BMS_NUM_CELLS.
 * Built once per cell count by `make bench` (4, 8, 12, 16, 32).            */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <time.h>
#include "bms_engine.h"

#define ITERS 2000000u

static double now_ns(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e9 + ts.tv_nsec;
}

int main(void)
{
    static bms_engine_t e; static bms_sample_t s;
    bms_init(&e);
    uint32_t rng = 1;
    double t0 = now_ns();
    for (uint32_t k = 0; k < ITERS; ++k) {
        for (unsigned i = 0; i < BMS_NUM_CELLS; ++i) {
            rng = rng * 1664525u + 1013904223u;
            s.cell_mv[i] = (uint16_t)(3650 + (rng >> 26));   /* 3650..3713 */
        }
        s.pack_current_ma = 1000; s.timestamp_ms = k + 1;
        bms_update(&e, &s);
    }
    double gen_and_update = (now_ns() - t0) / ITERS;

    /* subtract the cost of generating random inputs */
    t0 = now_ns();
    volatile uint16_t sink = 0;
    for (uint32_t k = 0; k < ITERS; ++k)
        for (unsigned i = 0; i < BMS_NUM_CELLS; ++i) {
            rng = rng * 1664525u + 1013904223u;
            s.cell_mv[i] = (uint16_t)(3650 + (rng >> 26));
            sink ^= s.cell_mv[i];
        }
    double gen = (now_ns() - t0) / ITERS;

    printf("%u,%zu,%zu,%zu,%zu,%.1f\n", (unsigned)BMS_NUM_CELLS,
           sizeof(bms_engine_t), sizeof(bms_status_t), sizeof(bms_sample_t),
           sizeof(((bms_engine_t *)0)->hist_imb_mv) * 2u,
           gen_and_update - gen);
    return (int)(sink & 0u);
}
