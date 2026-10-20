/* pack_model.h — physics-lite battery pack simulator ("the plant").
 *
 * Each series cell has its own capacity, internal resistance, starting SoC
 * and self-discharge, so imbalance appears and evolves naturally. The model
 * is deliberately separate from the engine: the engine only ever sees the
 * mV readings and pack current, exactly like it would on real hardware.  */
#ifndef PACK_MODEL_H
#define PACK_MODEL_H

#include <stdint.h>
#include "bms_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float soc;            /* 0..1                                   */
    float capacity_mah;
    float r_int_mohm;
    float self_dis_ma;    /* leakage current                         */
    float bleed_ma;       /* balancing resistor current when ON      */
    uint8_t bleeding;
} sim_cell_t;

typedef struct {
    sim_cell_t cell[BMS_NUM_CELLS];
    uint32_t   rng;
    float      noise_mv;  /* +/- ADC noise amplitude                 */
} sim_pack_t;

void  sim_pack_init(sim_pack_t *p, uint32_t seed, float start_soc);
/* Advance by dt_s with pack current (mA, + = discharge). */
void  sim_pack_step(sim_pack_t *p, float current_ma, float dt_s);
/* Terminal voltage readings (OCV - I*R + noise), mV. */
void  sim_pack_read(sim_pack_t *p, float current_ma, uint16_t out_mv[BMS_NUM_CELLS]);
void  sim_pack_set_bleed(sim_pack_t *p, uint8_t cell, uint8_t on);
float sim_ocv_mv(float soc01);

#ifdef __cplusplus
}
#endif

#endif
