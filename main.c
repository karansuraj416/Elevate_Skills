/* main.c — PC simulation runner.
 *
 *   ./bms_sim                 run the drive-cycle scenario with balancing
 *   ./bms_sim --no-balance    same scenario, balancer disabled (for comparison)
 *   ./bms_sim --csv out.csv   choose the CSV log file (default bms_log.csv)
 *   ./bms_sim --quiet         suppress event lines
 *
 * Scenario (1 Hz samples, ~2.7 h):
 *   rest -> 1C discharge -> rest -> 2C/0.2C pulses -> rest
 *        -> 0.5C CC charge with CV taper -> top-of-charge rest (balancing)
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bms_engine.h"
#include "pack_model.h"
#include "consumers.h"

#define DT_S        1u
#define T_END_S     9600u
#define C1_MA       ((int32_t)BMS_CELL_CAPACITY_MAH)

static const char *phase_name(uint32_t t)
{
    if (t <  300) return "rest";
    if (t < 1800) return "1C discharge";
    if (t < 2100) return "rest";
    if (t < 2700) return "pulse 2C/0.2C";
    if (t < 3000) return "rest";
    if (t < 8400) return "CC/CV charge";
    return "top rest";
}

/* Load profile. Charging uses the ENGINE's max cell to taper (CV stage) —
 * another consumer reusing the shared analysis.                           */
static int32_t profile_ma(uint32_t t, const bms_status_t *st)
{
    if (t <  300) return 0;
    if (t < 1800) return C1_MA;
    if (t < 2100) return 0;
    if (t < 2700) return ((t / 60u) & 1u) ? 2 * C1_MA : C1_MA / 5;
    if (t < 3000) return 0;
    if (t < 8400) {
        int32_t cc = -C1_MA / 2;
        if (st->max_mv < 4150u) return cc;
        /* CV stage: target current shrinks as the top cell nears 4.20 V.
         * A first-order filter stops ADC noise making the charger chatter. */
        static float i_cv = 0.0f;
        if (i_cv == 0.0f) i_cv = (float)cc;
        float target = (float)cc * (float)(4200 - (int32_t)st->max_mv) / 50.0f;
        if (target > -50.0f) target = -50.0f;
        i_cv += 0.02f * (target - i_cv);
        return (int32_t)i_cv;
    }
    return 0;
}

int main(int argc, char **argv)
{
    const char *csv = "bms_log.csv";
    int balance = 1, quiet = 0;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--no-balance")) balance = 0;
        else if (!strcmp(argv[i], "--quiet")) quiet = 1;
        else if (!strcmp(argv[i], "--csv") && i + 1 < argc) csv = argv[++i];
    }

    static sim_pack_t   pack;
    static bms_engine_t bms;
    sim_pack_init(&pack, 2026u, 0.85f);
    bms_init(&bms);

    FILE *fp = fopen(csv, "w");
    if (!fp) { perror(csv); return 1; }
    logger_write_header(fp);

    balancer_ctx_t bal = { &pack, balance, 0 };
    logger_ctx_t   log = { fp, 5 };                 /* log every 5 s */
    alarm_ctx_t    alm = { BMS_IMB_OK, BMS_TREND_UNKNOWN, quiet };
    bms_register_observer(&bms, balancer_observer, &bal);
    bms_register_observer(&bms, logger_observer,   &log);
    bms_register_observer(&bms, alarm_observer,    &alm);

    printf("Modular BMS engine — %uS pack simulation, balancing %s\n",
           (unsigned)BMS_NUM_CELLS, balance ? "ON" : "OFF");
    printf("engine RAM: %zu bytes (status %zu B)\n\n",
           BMS_ENGINE_SIZEOF, sizeof(bms_status_t));

    const char *last_phase = "";
    uint16_t peak_imb = 0;
    bms_sample_t s;
    for (uint32_t t = 0; t <= T_END_S; t += DT_S) {
        int32_t i_ma = profile_ma(t, bms_get_status(&bms));
        sim_pack_read(&pack, (float)i_ma, s.cell_mv);
        s.pack_current_ma = i_ma;
        s.timestamp_ms    = t * 1000u + 1u;
        bms_update(&bms, &s);
        sim_pack_step(&pack, (float)i_ma, (float)DT_S);

        const bms_status_t *st = bms_get_status(&bms);
        if (st->imbalance_mv > peak_imb) peak_imb = st->imbalance_mv;
        if (!quiet && strcmp(last_phase, phase_name(t)) != 0) {
            last_phase = phase_name(t);
            printf("---- t=%5us  phase: %s\n", t, last_phase);
        }
        if (!quiet && (t == 1500 || t == 5000 || t == T_END_S)) print_dashboard(st);
    }
    fclose(fp);

    const bms_status_t *st = bms_get_status(&bms);
    float soc_lo = 1, soc_hi = 0;
    for (unsigned i = 0; i < BMS_NUM_CELLS; ++i) {
        if (pack.cell[i].soc < soc_lo) soc_lo = pack.cell[i].soc;
        if (pack.cell[i].soc > soc_hi) soc_hi = pack.cell[i].soc;
    }
    printf("\n==== SUMMARY (%s) ====\n", balance ? "balancing ON" : "balancing OFF");
    printf("final imbalance      : %u mV (peak %u mV during run)\n", st->imbalance_mv, peak_imb);
    printf("final true SoC spread: %.2f %%  (cell SoC %.1f..%.1f %%)\n",
           100.0f * (soc_hi - soc_lo), 100.0f * soc_lo, 100.0f * soc_hi);
    printf("final level / trend  : %s / %s\n", bms_level_str(st->level), bms_trend_str(st->trend));
    printf("bleed cell-seconds   : %u\n", bal.bleed_seconds);
    printf("CSV log              : %s\n", csv);
    return 0;
}
