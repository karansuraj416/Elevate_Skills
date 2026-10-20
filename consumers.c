/* consumers.c — modules that REUSE the engine's analysis. */
#include "consumers.h"

void balancer_observer(const bms_status_t *st, void *vctx)
{
    balancer_ctx_t *b = (balancer_ctx_t *)vctx;
    for (uint8_t i = 0; i < BMS_NUM_CELLS; ++i) {
        int on = b->enabled && ((st->balance_mask[i >> 3] >> (i & 7u)) & 1u);
        sim_pack_set_bleed(b->pack, i, (uint8_t)on);
        if (on) b->bleed_seconds++;
    }
}

void logger_write_header(FILE *fp)
{
    fprintf(fp, "t_s,current_ma,soc_pct,c_rate");
    for (unsigned i = 0; i < BMS_NUM_CELLS; ++i) fprintf(fp, ",cell%u_mv", i + 1u);
    fprintf(fp, ",min_mv,max_mv,weakest,strongest,imbalance_mv,slope_mv_s,"
                "trend,warn_mv,crit_mv,level,balance_count\n");
}

void logger_observer(const bms_status_t *st, void *vctx)
{
    logger_ctx_t *l = (logger_ctx_t *)vctx;
    if (!l->fp || (st->update_count % l->every_n) != 0u) return;

    unsigned nbal = 0;
    for (uint8_t i = 0; i < BMS_NUM_CELLS; ++i)
        nbal += (st->balance_mask[i >> 3] >> (i & 7u)) & 1u;

    fprintf(l->fp, "%.1f,%ld,%.2f,%.3f", st->timestamp_ms / 1000.0,
            (long)st->pack_current_ma, st->soc_pct, st->c_rate);
    for (uint8_t i = 0; i < BMS_NUM_CELLS; ++i) fprintf(l->fp, ",%u", st->cell_mv[i]);
    fprintf(l->fp, ",%u,%u,%u,%u,%u,%.4f,%s,%.1f,%.1f,%s,%u\n",
            st->min_mv, st->max_mv, st->weakest_idx + 1u, st->strongest_idx + 1u,
            st->imbalance_mv, st->imbalance_slope_mv_s, bms_trend_str(st->trend),
            st->warn_threshold_mv, st->crit_threshold_mv,
            bms_level_str(st->level), nbal);
}

void alarm_observer(const bms_status_t *st, void *vctx)
{
    alarm_ctx_t *a = (alarm_ctx_t *)vctx;
    if (!a->quiet && st->level != a->last_level)
        printf("[t=%6.0fs] LEVEL %-8s -> %-8s  imb=%3u mV  thr=%.0f/%.0f mV  "
               "SoC=%5.1f%%  %.2fC  weakest=C%u\n",
               st->timestamp_ms / 1000.0, bms_level_str(a->last_level),
               bms_level_str(st->level), st->imbalance_mv,
               st->warn_threshold_mv, st->crit_threshold_mv,
               st->soc_pct, st->c_rate, st->weakest_idx + 1u);
    a->last_level = st->level;
    a->last_trend = st->trend;
}

void print_dashboard(const bms_status_t *st)
{
    printf("\n+------------------------------------------------------------+\n");
    printf("| BMS ENGINE  %2uS   t=%7.0f s   I=%6ld mA   SoC=%5.1f %%   |\n",
           (unsigned)BMS_NUM_CELLS, st->timestamp_ms / 1000.0,
           (long)st->pack_current_ma, st->soc_pct);
    printf("+------------------------------------------------------------+\n");
    for (uint8_t i = 0; i < BMS_NUM_CELLS; ++i) {
        int bar = (int)((st->cell_mv[i] - 3000) / 30);
        if (bar < 0) bar = 0;
        if (bar > 40) bar = 40;
        printf("| C%-2u %4u mV ", i + 1u, st->cell_mv[i]);
        for (int k = 0; k < 40; ++k) putchar(k < bar ? '#' : '.');
        printf(" %s%s\n", i == st->weakest_idx ? "<W" : (i == st->strongest_idx ? "<S" : "  "),
               ((st->balance_mask[i >> 3] >> (i & 7u)) & 1u) ? " BAL" : "");
    }
    printf("| pack %lu mV | avg %u | imbalance %u mV | %s (%.3f mV/s)\n",
           (unsigned long)st->pack_mv, st->avg_mv, st->imbalance_mv,
           bms_trend_str(st->trend), st->imbalance_slope_mv_s);
    printf("| adaptive thr: warn %.1f mV / crit %.1f mV  -> %s  faults=0x%02lx\n",
           st->warn_threshold_mv, st->crit_threshold_mv, bms_level_str(st->level),
           (unsigned long)st->faults);
}
