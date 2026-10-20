/* bms_config.h — the ONLY file you edit to rescale the engine.
 *
 * Change BMS_NUM_CELLS (or pass -DBMS_NUM_CELLS=16 to the compiler) and every
 * array, loop bound and memory figure in the engine follows automatically.
 */
#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

#ifndef BMS_NUM_CELLS
#define BMS_NUM_CELLS            4u      /* 4S default; 8 and 16 tested     */
#endif

#if (BMS_NUM_CELLS < 2u) || (BMS_NUM_CELLS > 255u)
#error "BMS_NUM_CELLS must be between 2 and 255 (cell index stored in uint8_t)"
#endif

/* ---- Trend analysis ---------------------------------------------------- */
#define BMS_TREND_WINDOW         16u     /* samples kept in ring buffer (2^n) */
#define BMS_TREND_DEADBAND_MV_S  0.05f   /* |slope| below this = STABLE       */

#if (BMS_TREND_WINDOW & (BMS_TREND_WINDOW - 1u)) != 0u
#error "BMS_TREND_WINDOW must be a power of two (cheap index wrap)"
#endif

/* ---- Cell chemistry limits (Li-ion NMC, mV) ----------------------------- */
#define BMS_CELL_V_MIN_MV        3000u
#define BMS_CELL_V_MAX_MV        4200u
#define BMS_CELL_V_NOMINAL_MV    3700u
#define BMS_CELL_CAPACITY_MAH    2500u   /* per cell, series pack            */

/* ---- Adaptive imbalance thresholds (mV) --------------------------------
 * Base thresholds come from a SoC-indexed lookup table (see bms_engine.c);
 * these constants bound the discharge-rate correction.                    */
#define BMS_IMB_RATE_GAIN_MV_PER_C   10.0f  /* extra allowance per 1C load   */
#define BMS_IMB_RATE_MAX_EXTRA_MV    30.0f  /* cap on rate correction        */
#define BMS_IMB_CRITICAL_FACTOR      2.0f   /* critical = warn x factor      */

/* ---- Balancing ---------------------------------------------------------- */
#define BMS_BALANCE_MIN_SOC_PCT      20.0f  /* don't bleed nearly-empty pack */
#define BMS_BALANCE_MAX_C_RATE       0.5f   /* above this, spread is mostly I*R */

#endif /* BMS_CONFIG_H */
