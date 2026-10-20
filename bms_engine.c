/* bms_engine.c — implementation of the modular BMS analysis engine. */
#include "bms_engine.h"
#include <string.h>

/* ============================================================ lookup tables
 * Both tables are indexed by SoC in 10 % steps (11 points, 0..100 %).      */
#define LUT_POINTS 11u

/* Open-circuit voltage of a typical NMC Li-ion cell, mV. */
static const uint16_t k_ocv_mv[LUT_POINTS] = {
    3000, 3450, 3550, 3620, 3670, 3710, 3780, 3870, 3960, 4070, 4200
};

/* Base WARNING imbalance threshold, mV, as a function of SoC.
 * Why it is not flat: in the 30-70 % region the OCV curve is flat
 * (~5-7 mV per % SoC), so even 20 mV of spread hides several % of real
 * charge mismatch -> keep the limit tight. Near empty/full the curve is
 * steep (15-45 mV per %), so larger mV spread is normal and harmless ->
 * relax the limit to avoid nuisance alarms.                               */
static const uint8_t k_warn_mv[LUT_POINTS] = {
      60,   50,   35,   25,   20,   20,   20,   25,   30,   40,   50
};

static float lut_interp_u8(const uint8_t *t, float soc)
{
    if (soc <= 0.0f)   return (float)t[0];
    if (soc >= 100.0f) return (float)t[LUT_POINTS - 1u];
    float    pos = soc / 10.0f;
    uint32_t i   = (uint32_t)pos;
    float    f   = pos - (float)i;
    return (float)t[i] + f * ((float)t[i + 1u] - (float)t[i]);
}

/* Inverse OCV lookup: cell mV -> SoC %. Used once to seed the estimator. */
static float soc_from_ocv(float mv)
{
    if (mv <= k_ocv_mv[0]) return 0.0f;
    if (mv >= k_ocv_mv[LUT_POINTS - 1u]) return 100.0f;
    for (uint32_t i = 0; i + 1u < LUT_POINTS; ++i) {
        if (mv <= k_ocv_mv[i + 1u]) {
            float span = (float)(k_ocv_mv[i + 1u] - k_ocv_mv[i]);
            return 10.0f * ((float)i + (mv - (float)k_ocv_mv[i]) / span);
        }
    }
    return 100.0f;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ======================================================== adaptive policy */
void bms_adaptive_thresholds(float soc_pct, float c_rate,
                             float *warn_mv, float *crit_mv)
{
    float base  = lut_interp_u8(k_warn_mv, clampf(soc_pct, 0.0f, 100.0f));

    /* Under load, cells with different internal resistance (R_i) sag by
     * different amounts: dV = I * dR_i. That spread is not a charge
     * imbalance, so widen the window in proportion to the C-rate.        */
    float extra = BMS_IMB_RATE_GAIN_MV_PER_C * (c_rate < 0.0f ? -c_rate : c_rate);
    if (extra > BMS_IMB_RATE_MAX_EXTRA_MV) extra = BMS_IMB_RATE_MAX_EXTRA_MV;

    float warn = base + extra;
    if (warn_mv) *warn_mv = warn;
    if (crit_mv) *crit_mv = warn * BMS_IMB_CRITICAL_FACTOR;
}

/* ============================================================ sub-stages */

/* Stage 1 — single O(N) pass: min, max, sum, limit faults. */
static void scan_cells(bms_status_t *st)
{
    uint16_t mn = st->cell_mv[0], mx = st->cell_mv[0];
    uint8_t  imn = 0, imx = 0;
    uint32_t sum = 0;
    uint32_t faults = 0;

    for (uint8_t i = 0; i < BMS_NUM_CELLS; ++i) {
        uint16_t v = st->cell_mv[i];
        sum += v;
        if (v < mn) { mn = v; imn = i; }
        if (v > mx) { mx = v; imx = i; }
        if (v < BMS_CELL_V_MIN_MV) faults |= BMS_FAULT_CELL_UV;
        if (v > BMS_CELL_V_MAX_MV) faults |= BMS_FAULT_CELL_OV;
    }
    st->min_mv = mn;        st->weakest_idx   = imn;
    st->max_mv = mx;        st->strongest_idx = imx;
    st->pack_mv = sum;
    st->avg_mv  = (uint16_t)(sum / BMS_NUM_CELLS);
    st->imbalance_mv = (uint16_t)(mx - mn);
    st->faults = faults;
}

/* Stage 2 — SoC by coulomb counting, seeded from OCV on first sample. */
static void update_soc(bms_engine_t *e, const bms_sample_t *s, uint32_t dt_ms)
{
    bms_status_t *st = &e->status;
    if (!e->soc_initialised) {
        e->soc_pct = soc_from_ocv((float)st->avg_mv);
        e->soc_initialised = true;
    } else if (dt_ms > 0u) {
        /* mAh removed = mA * h */
        float dmah = (float)s->pack_current_ma * ((float)dt_ms / 3600000.0f);
        e->soc_pct -= 100.0f * dmah / (float)BMS_CELL_CAPACITY_MAH;
        e->soc_pct  = clampf(e->soc_pct, 0.0f, 100.0f);
    }
    st->soc_pct = e->soc_pct;
    st->c_rate  = (float)(s->pack_current_ma < 0 ? -s->pack_current_ma
                                                 :  s->pack_current_ma)
                  / (float)BMS_CELL_CAPACITY_MAH;
}

/* Stage 3 — trend: least-squares slope of imbalance vs time over a fixed
 * ring buffer. Cost is O(W) and does NOT grow with the number of cells.   */
static void update_trend(bms_engine_t *e)
{
    bms_status_t *st = &e->status;
    e->hist_imb_mv[e->hist_head] = (float)st->imbalance_mv;
    e->hist_t_s[e->hist_head]    = (float)st->timestamp_ms / 1000.0f;
    e->hist_head = (uint8_t)((e->hist_head + 1u) & (BMS_TREND_WINDOW - 1u));
    if (e->hist_count < BMS_TREND_WINDOW) e->hist_count++;

    st->trend_samples = e->hist_count;
    if (e->hist_count < 4u) {
        st->trend = BMS_TREND_UNKNOWN;
        st->imbalance_slope_mv_s = 0.0f;
        return;
    }

    /* Use time relative to the newest sample to keep float precision. */
    uint8_t newest = (uint8_t)((e->hist_head + BMS_TREND_WINDOW - 1u)
                               & (BMS_TREND_WINDOW - 1u));
    float t0 = e->hist_t_s[newest];
    float n = (float)e->hist_count, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (uint8_t k = 0; k < e->hist_count; ++k) {
        uint8_t i = (uint8_t)((newest + BMS_TREND_WINDOW - k)
                              & (BMS_TREND_WINDOW - 1u));
        float x = e->hist_t_s[i] - t0, y = e->hist_imb_mv[i];
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    float den = n * sxx - sx * sx;
    float slope = (den > 1e-9f || den < -1e-9f) ? (n * sxy - sx * sy) / den : 0.0f;
    st->imbalance_slope_mv_s = slope;

    if      (slope >  BMS_TREND_DEADBAND_MV_S) st->trend = BMS_TREND_INCREASING;
    else if (slope < -BMS_TREND_DEADBAND_MV_S) st->trend = BMS_TREND_DECREASING;
    else                                       st->trend = BMS_TREND_STABLE;
}

/* Stage 4 — classify against adaptive thresholds, with 10 % hysteresis so
 * the level does not chatter when imbalance sits on a boundary.            */
static void classify(bms_status_t *st)
{
    bms_adaptive_thresholds(st->soc_pct, st->c_rate,
                            &st->warn_threshold_mv, &st->crit_threshold_mv);
    float imb = (float)st->imbalance_mv;
    bms_imb_level_t prev = st->level, lvl;

    if      (imb >= st->crit_threshold_mv) lvl = BMS_IMB_CRITICAL;
    else if (imb >= st->warn_threshold_mv) lvl = BMS_IMB_WARNING;
    else                                   lvl = BMS_IMB_OK;

    /* hysteresis: only step DOWN once 10 % below the boundary */
    if (lvl < prev) {
        float bound = (prev == BMS_IMB_CRITICAL) ? st->crit_threshold_mv
                                                 : st->warn_threshold_mv;
        if (imb > 0.9f * bound) lvl = prev;
    }
    st->level = lvl;
    if (lvl == BMS_IMB_CRITICAL) st->faults |= BMS_FAULT_IMB_CRIT;
}

/* Stage 5 — passive-balancing advice: bleed every cell that sits more than
 * half a warning window above the weakest cell. Only when the pack has
 * charge to spare and is at rest or under light load: under heavy load the
 * spread is dominated by I*R differences, and bleeding on that basis would
 * drain the wrong cells.                             */
static void balance_advice(bms_status_t *st)
{
    memset(st->balance_mask, 0, sizeof st->balance_mask);
    if (st->level == BMS_IMB_OK)                  return;
    if (st->soc_pct < BMS_BALANCE_MIN_SOC_PCT)    return;
    if (st->c_rate > BMS_BALANCE_MAX_C_RATE)      return;
    if (st->faults & BMS_FAULT_CELL_UV)           return;

    float limit = (float)st->min_mv + 0.5f * st->warn_threshold_mv;
    for (uint8_t i = 0; i < BMS_NUM_CELLS; ++i)
        if ((float)st->cell_mv[i] > limit)
            st->balance_mask[i >> 3] |= (uint8_t)(1u << (i & 7u));
}

/* =================================================================== API */
void bms_init(bms_engine_t *e)
{
    memset(e, 0, sizeof *e);
    e->status.trend = BMS_TREND_UNKNOWN;
}

bool bms_update(bms_engine_t *e, const bms_sample_t *s)
{
    bms_status_t *st = &e->status;

    /* reject non-monotonic time or a dead channel (0 mV) */
    bool bad = (st->update_count > 0u && s->timestamp_ms <= st->timestamp_ms);
    for (uint8_t i = 0; i < BMS_NUM_CELLS && !bad; ++i)
        if (s->cell_mv[i] == 0u) bad = true;
    if (bad) { st->faults |= BMS_FAULT_BAD_SAMPLE; return false; }

    uint32_t dt_ms = st->update_count ? s->timestamp_ms - st->timestamp_ms : 0u;

    memcpy(st->cell_mv, s->cell_mv, sizeof st->cell_mv);
    st->pack_current_ma = s->pack_current_ma;
    st->timestamp_ms    = s->timestamp_ms;

    scan_cells(st);
    update_soc(e, s, dt_ms);
    update_trend(e);
    classify(st);
    balance_advice(st);
    st->update_count++;

    for (uint8_t k = 0; k < e->obs_count; ++k)
        e->obs_fn[k](st, e->obs_ctx[k]);
    return true;
}

const bms_status_t *bms_get_status(const bms_engine_t *e) { return &e->status; }
uint8_t     bms_weakest_cell  (const bms_engine_t *e) { return e->status.weakest_idx; }
uint8_t     bms_strongest_cell(const bms_engine_t *e) { return e->status.strongest_idx; }
uint16_t    bms_imbalance_mv  (const bms_engine_t *e) { return e->status.imbalance_mv; }
bms_trend_t bms_trend         (const bms_engine_t *e) { return e->status.trend; }

bool bms_cell_should_balance(const bms_engine_t *e, uint8_t cell)
{
    if (cell >= BMS_NUM_CELLS) return false;
    return (e->status.balance_mask[cell >> 3] >> (cell & 7u)) & 1u;
}

void bms_set_soc(bms_engine_t *e, float soc_pct)
{
    e->soc_pct = clampf(soc_pct, 0.0f, 100.0f);
    e->soc_initialised = true;
    e->status.soc_pct = e->soc_pct;
}

bool bms_register_observer(bms_engine_t *e, bms_observer_fn fn, void *ctx)
{
    if (!fn || e->obs_count >= BMS_MAX_OBSERVERS) return false;
    e->obs_fn[e->obs_count]  = fn;
    e->obs_ctx[e->obs_count] = ctx;
    e->obs_count++;
    return true;
}

const char *bms_trend_str(bms_trend_t t)
{
    switch (t) {
    case BMS_TREND_DECREASING: return "DECREASING";
    case BMS_TREND_STABLE:     return "STABLE";
    case BMS_TREND_INCREASING: return "INCREASING";
    default:                   return "UNKNOWN";
    }
}

const char *bms_level_str(bms_imb_level_t l)
{
    switch (l) {
    case BMS_IMB_WARNING:  return "WARNING";
    case BMS_IMB_CRITICAL: return "CRITICAL";
    default:               return "OK";
    }
}
