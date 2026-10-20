/* bms_engine.h — public interface of the modular BMS analysis engine.
 *
 * Design rules
 *  - No dynamic memory. One bms_engine_t instance holds all state; its size
 *    is a compile-time function of BMS_NUM_CELLS (see BMS_ENGINE_SIZEOF).
 *  - One write path (bms_update) and many read paths. Every later task
 *    (balancing, logging, CAN/UART telemetry, UI, protection) reads the same
 *    const bms_status_t snapshot, so no module repeats min/max/trend maths.
 *  - Portable C99: builds on a PC, an ESP32, an STM32 or an Arduino.
 */
#ifndef BMS_ENGINE_H
#define BMS_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include "bms_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMS_MASK_BYTES   ((BMS_NUM_CELLS + 7u) / 8u)

/* ---------------------------------------------------------------- inputs */
typedef struct {
    uint16_t cell_mv[BMS_NUM_CELLS];  /* measured cell voltages, mV          */
    int32_t  pack_current_ma;         /* + = discharge, - = charge           */
    uint32_t timestamp_ms;            /* monotonic time of the sample        */
} bms_sample_t;

/* --------------------------------------------------------------- outputs */
typedef enum {
    BMS_TREND_UNKNOWN = 0,            /* not enough history yet              */
    BMS_TREND_DECREASING,             /* cells converging                    */
    BMS_TREND_STABLE,
    BMS_TREND_INCREASING              /* cells diverging                     */
} bms_trend_t;

typedef enum {
    BMS_IMB_OK = 0,
    BMS_IMB_WARNING,
    BMS_IMB_CRITICAL
} bms_imb_level_t;

typedef enum {
    BMS_FAULT_NONE        = 0,
    BMS_FAULT_CELL_UV     = 1u << 0,  /* any cell under BMS_CELL_V_MIN_MV    */
    BMS_FAULT_CELL_OV     = 1u << 1,  /* any cell over  BMS_CELL_V_MAX_MV    */
    BMS_FAULT_BAD_SAMPLE  = 1u << 2,  /* time went backwards / zero reading  */
    BMS_FAULT_IMB_CRIT    = 1u << 3
} bms_fault_t;

typedef struct {
    /* raw copy of the last accepted sample */
    uint16_t cell_mv[BMS_NUM_CELLS];
    int32_t  pack_current_ma;
    uint32_t timestamp_ms;

    /* extremes & spread */
    uint8_t  weakest_idx;             /* lowest-voltage cell (0-based)       */
    uint8_t  strongest_idx;           /* highest-voltage cell                */
    uint16_t min_mv, max_mv;
    uint16_t avg_mv;
    uint32_t pack_mv;
    uint16_t imbalance_mv;            /* max_mv - min_mv                     */

    /* trend */
    bms_trend_t trend;
    float    imbalance_slope_mv_s;    /* least-squares slope over window     */
    uint8_t  trend_samples;           /* how many samples in the window      */

    /* operating point */
    float    soc_pct;                 /* 0..100                              */
    float    c_rate;                  /* |I| / capacity                      */

    /* adaptive thresholds for THIS operating point */
    float    warn_threshold_mv;
    float    crit_threshold_mv;
    bms_imb_level_t level;

    /* advice for the balancer module (bit i => bleed cell i) */
    uint8_t  balance_mask[BMS_MASK_BYTES];

    uint32_t faults;                  /* OR of bms_fault_t                   */
    uint32_t update_count;
} bms_status_t;

/* Optional observer: called after every successful update. */
typedef void (*bms_observer_fn)(const bms_status_t *st, void *ctx);

#define BMS_MAX_OBSERVERS 4u

/* ---------------------------------------------------- engine (opaque-ish)
 * The struct is visible only so it can be allocated statically.
 * Application code must not touch its fields directly.                   */
typedef struct {
    bms_status_t status;

    /* trend ring buffer: fixed size, independent of BMS_NUM_CELLS */
    float    hist_imb_mv[BMS_TREND_WINDOW];
    float    hist_t_s[BMS_TREND_WINDOW];
    uint8_t  hist_head;
    uint8_t  hist_count;

    /* SoC estimator state */
    float    soc_pct;
    bool     soc_initialised;

    bms_observer_fn obs_fn[BMS_MAX_OBSERVERS];
    void           *obs_ctx[BMS_MAX_OBSERVERS];
    uint8_t         obs_count;
} bms_engine_t;

#define BMS_ENGINE_SIZEOF  (sizeof(bms_engine_t))

/* ------------------------------------------------------------------ API */
void bms_init(bms_engine_t *e);

/* Feed one sample. Returns false (and sets BMS_FAULT_BAD_SAMPLE) if the
 * sample is rejected; the previous status is then left intact.           */
bool bms_update(bms_engine_t *e, const bms_sample_t *s);

/* Read-only view used by every consumer module. */
const bms_status_t *bms_get_status(const bms_engine_t *e);

/* Convenience accessors (thin wrappers over the status snapshot). */
uint8_t     bms_weakest_cell(const bms_engine_t *e);
uint8_t     bms_strongest_cell(const bms_engine_t *e);
uint16_t    bms_imbalance_mv(const bms_engine_t *e);
bms_trend_t bms_trend(const bms_engine_t *e);
bool        bms_cell_should_balance(const bms_engine_t *e, uint8_t cell);

/* Pure function, exposed so tests/other tasks can query the policy. */
void bms_adaptive_thresholds(float soc_pct, float c_rate,
                             float *warn_mv, float *crit_mv);

/* Force SoC (e.g. after a full charge or from an external fuel gauge). */
void bms_set_soc(bms_engine_t *e, float soc_pct);

bool bms_register_observer(bms_engine_t *e, bms_observer_fn fn, void *ctx);

const char *bms_trend_str(bms_trend_t t);
const char *bms_level_str(bms_imb_level_t l);

#ifdef __cplusplus
}
#endif
#endif /* BMS_ENGINE_H */
