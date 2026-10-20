/* pack_model.c — cell-level simulation used to drive the BMS engine. */
#include "pack_model.h"
#include <string.h>

static const float k_ocv[11] = {
    3000, 3450, 3550, 3620, 3670, 3710, 3780, 3870, 3960, 4070, 4200
};

float sim_ocv_mv(float soc)
{
    if (soc <= 0.0f) return k_ocv[0];
    if (soc >= 1.0f) return k_ocv[10];
    float pos = soc * 10.0f;
    int   i   = (int)pos;
    float f   = pos - (float)i;
    return k_ocv[i] + f * (k_ocv[i + 1] - k_ocv[i]);
}

/* small deterministic LCG -> runs are reproducible */
static float rnd01(sim_pack_t *p)
{
    p->rng = p->rng * 1664525u + 1013904223u;
    return (float)(p->rng >> 8) / 16777216.0f;
}
static float rnd_pm(sim_pack_t *p) { return 2.0f * rnd01(p) - 1.0f; }

void sim_pack_init(sim_pack_t *p, uint32_t seed, float start_soc)
{
    memset(p, 0, sizeof *p);
    p->rng = seed ? seed : 12345u;
    p->noise_mv = 1.5f;

    for (uint32_t i = 0; i < BMS_NUM_CELLS; ++i) {
        sim_cell_t *c = &p->cell[i];
        /* manufacturing spread */
        c->capacity_mah = (float)BMS_CELL_CAPACITY_MAH * (1.0f + 0.02f * rnd_pm(p));
        c->r_int_mohm   = 25.0f + 4.0f * rnd_pm(p);
        c->self_dis_ma  = 0.05f + 0.03f * rnd01(p);
        c->soc          = start_soc + 0.01f * rnd_pm(p);
        c->bleed_ma     = 100.0f;      /* ~42 ohm bleed resistor at 4.2 V */
    }
    /* Inject one aged "weak" cell so the scenario is interesting:
     * 8 % less capacity, higher resistance, starts a little low.         */
    sim_cell_t *w = &p->cell[BMS_NUM_CELLS / 2u];
    w->capacity_mah *= 0.92f;
    w->r_int_mohm   *= 1.6f;
    w->soc          -= 0.03f;
    /* and one cell with a leaky separator */
    p->cell[BMS_NUM_CELLS - 1u].self_dis_ma = 1.5f;
}

void sim_pack_step(sim_pack_t *p, float current_ma, float dt_s)
{
    float h = dt_s / 3600.0f;
    for (uint32_t i = 0; i < BMS_NUM_CELLS; ++i) {
        sim_cell_t *c = &p->cell[i];
        float i_cell = current_ma + c->self_dis_ma + (c->bleeding ? c->bleed_ma : 0.0f);
        c->soc -= i_cell * h / c->capacity_mah;
        if (c->soc < 0.0f) c->soc = 0.0f;
        if (c->soc > 1.0f) c->soc = 1.0f;
    }
}

void sim_pack_read(sim_pack_t *p, float current_ma, uint16_t out_mv[BMS_NUM_CELLS])
{
    for (uint32_t i = 0; i < BMS_NUM_CELLS; ++i) {
        sim_cell_t *c = &p->cell[i];
        float v = sim_ocv_mv(c->soc)
                - current_ma * c->r_int_mohm / 1000.0f    /* I*R sag      */
                + p->noise_mv * rnd_pm(p);                 /* ADC noise    */
        if (v < 1.0f) v = 1.0f;
        out_mv[i] = (uint16_t)(v + 0.5f);
    }
}

void sim_pack_set_bleed(sim_pack_t *p, uint8_t cell, uint8_t on)
{
    if (cell < BMS_NUM_CELLS) p->cell[cell].bleeding = on ? 1u : 0u;
}
