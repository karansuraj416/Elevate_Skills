/* consumers.h — example "subsequent task" modules.
 *
 * None of these compute min/max/imbalance/trend themselves. They only read
 * the const bms_status_t published by the engine. Adding a new task
 * (CAN telemetry, SD logging, LCD page, cloud upload) follows the same
 * pattern: write an observer, register it, done.                         */
#ifndef CONSUMERS_H
#define CONSUMERS_H

#include <stdio.h>
#include "bms_engine.h"
#include "pack_model.h"

/* Task A — passive balancer: turns the engine's balance mask into
 * bleed-resistor commands on the (simulated) hardware.                   */
typedef struct { sim_pack_t *pack; int enabled; uint32_t bleed_seconds; } balancer_ctx_t;
void balancer_observer(const bms_status_t *st, void *ctx);

/* Task B — CSV data logger. */
typedef struct { FILE *fp; uint32_t every_n; } logger_ctx_t;
void logger_write_header(FILE *fp);
void logger_observer(const bms_status_t *st, void *ctx);

/* Task C — alarm / event monitor: prints only on state changes. */
typedef struct { bms_imb_level_t last_level; bms_trend_t last_trend; int quiet; } alarm_ctx_t;
void alarm_observer(const bms_status_t *st, void *ctx);

/* Human-readable one-screen dashboard. */
void print_dashboard(const bms_status_t *st);

#endif
