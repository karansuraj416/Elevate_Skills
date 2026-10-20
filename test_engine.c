/* test_engine.c — unit tests for the engine. Runs for any BMS_NUM_CELLS. */
#include <stdio.h>
#include <math.h>
#include "bms_engine.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static void fill(bms_sample_t *s, uint16_t mv, int32_t ma, uint32_t t)
{
    for (unsigned i = 0; i < BMS_NUM_CELLS; ++i) s->cell_mv[i] = mv;
    s->pack_current_ma = ma; s->timestamp_ms = t;
}

static void test_extremes(void)
{
    bms_engine_t e; bms_init(&e); bms_sample_t s;
    fill(&s, 3700, 0, 1000);
    s.cell_mv[1] = 3650;                         /* weakest  */
    s.cell_mv[BMS_NUM_CELLS - 1] = 3740;         /* strongest */
    CHECK(bms_update(&e, &s));
    CHECK(bms_weakest_cell(&e) == 1);
    CHECK(bms_strongest_cell(&e) == BMS_NUM_CELLS - 1);
    CHECK(bms_imbalance_mv(&e) == 90);
    CHECK(e.status.pack_mv == 3700u * BMS_NUM_CELLS - 50u + 40u);
}

static void test_trend(void)
{
    bms_engine_t e; bms_init(&e); bms_sample_t s;
    for (uint32_t k = 0; k < 10; ++k) {          /* spread grows 2 mV/s */
        fill(&s, 3700, 0, 1000 + k * 1000);
        s.cell_mv[0] = (uint16_t)(3700 - 2 * k);
        bms_update(&e, &s);
    }
    CHECK(bms_trend(&e) == BMS_TREND_INCREASING);
    CHECK(fabsf(e.status.imbalance_slope_mv_s - 2.0f) < 0.01f);

    for (uint32_t k = 0; k < 16; ++k) {          /* spread shrinks */
        fill(&s, 3700, 0, 20000 + k * 1000);
        s.cell_mv[0] = (uint16_t)(3660 + 2 * k);
        bms_update(&e, &s);
    }
    CHECK(bms_trend(&e) == BMS_TREND_DECREASING);

    for (uint32_t k = 0; k < 16; ++k) {          /* constant */
        fill(&s, 3700, 0, 40000 + k * 1000);
        s.cell_mv[0] = 3680;
        bms_update(&e, &s);
    }
    CHECK(bms_trend(&e) == BMS_TREND_STABLE);
}

static void test_adaptive(void)
{
    float w50, w5, w95, w50_load, c;
    bms_adaptive_thresholds(50, 0, &w50, &c);
    bms_adaptive_thresholds(5, 0, &w5, NULL);
    bms_adaptive_thresholds(95, 0, &w95, NULL);
    bms_adaptive_thresholds(50, 2.0f, &w50_load, NULL);
    CHECK(w50 < w5 && w50 < w95);                /* tight in flat region  */
    CHECK(w50_load > w50);                       /* relaxed under load    */
    CHECK(fabsf(c - 2.0f * w50) < 1e-3f);
    float wcap; bms_adaptive_thresholds(50, 100.0f, &wcap, NULL);
    CHECK(wcap <= w50 + BMS_IMB_RATE_MAX_EXTRA_MV + 1e-3f);

    /* same 30 mV spread: WARNING at 50 % rest, OK at 50 % under 2C */
    bms_engine_t e; bms_init(&e); bms_sample_t s;
    bms_set_soc(&e, 50);
    fill(&s, 3710, 0, 1000); s.cell_mv[0] = 3680;
    bms_update(&e, &s);
    CHECK(e.status.level == BMS_IMB_WARNING);
    bms_init(&e); bms_set_soc(&e, 50);
    fill(&s, 3710, 2 * BMS_CELL_CAPACITY_MAH, 1000); s.cell_mv[0] = 3680;
    bms_update(&e, &s);
    CHECK(e.status.level == BMS_IMB_OK);
}

static void test_balance_and_faults(void)
{
    bms_engine_t e; bms_init(&e); bms_sample_t s;
    bms_set_soc(&e, 60);
    fill(&s, 3780, 0, 1000); s.cell_mv[0] = 3740;   /* 40 mV low cell */
    bms_update(&e, &s);
    CHECK(!bms_cell_should_balance(&e, 0));
    CHECK(bms_cell_should_balance(&e, 1));
    CHECK(!bms_cell_should_balance(&e, BMS_NUM_CELLS)); /* out of range */

    s.timestamp_ms = 500;                             /* time went back */
    CHECK(!bms_update(&e, &s));
    CHECK(e.status.faults & BMS_FAULT_BAD_SAMPLE);

    fill(&s, 3700, 0, 2000); s.cell_mv[0] = 2900;
    bms_update(&e, &s);
    CHECK(e.status.faults & BMS_FAULT_CELL_UV);
}

static int obs_hits = 0;
static void obs(const bms_status_t *st, void *ctx) { (void)st; (*(int *)ctx)++; }

static void test_observer(void)
{
    bms_engine_t e; bms_init(&e); bms_sample_t s;
    CHECK(bms_register_observer(&e, obs, &obs_hits));
    fill(&s, 3700, 0, 1000); bms_update(&e, &s);
    fill(&s, 3700, 0, 2000); bms_update(&e, &s);
    CHECK(obs_hits == 2);
}

int main(void)
{
    printf("engine tests, BMS_NUM_CELLS=%u\n", (unsigned)BMS_NUM_CELLS);
    test_extremes(); test_trend(); test_adaptive();
    test_balance_and_faults(); test_observer();
    printf(fails ? "%d FAILURE(S)\n" : "all tests passed\n", fails);
    return fails ? 1 : 0;
}
