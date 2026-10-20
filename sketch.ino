/* ESP32 front-end for the modular BMS engine — runs in the Wokwi simulator
 * (https://wokwi.com, project type "ESP32") or on a real ESP32 DevKit.
 *
 * Two input modes, chosen with the slide switch on GPIO4:
 *   MANUAL : 4 potentiometers = cell voltages (3.0-4.2 V),
 *            5th pot = pack current (-2.5 A charge .. +5 A discharge).
 *            Turn one pot down and watch it become the weakest cell.
 *   AUTO   : the built-in pack model (pack_model.c) drives the engine,
 *            accelerated 10x, with balancing fed back to the model.
 *
 * Outputs: 4 yellow LEDs = balance bleed command per cell,
 *          red LED = imbalance WARNING (blink) / CRITICAL (solid),
 *          Serial Monitor (115200) = live dashboard.
 *
 * The engine files are byte-for-byte the same as in ../src — that is the
 * point of the modular design.                                            */
#include "bms_engine.h"
#include "pack_model.h"

static const uint8_t CELL_ADC[4]  = {34, 35, 32, 33};
static const uint8_t CURRENT_ADC  = 36;   /* VP */
static const uint8_t BAL_LED[4]   = {25, 26, 27, 14};
static const uint8_t ALARM_LED    = 2;
static const uint8_t MODE_SW      = 4;    /* LOW = AUTO, HIGH = MANUAL */

static bms_engine_t bms;
static sim_pack_t   pack;
static uint32_t     sim_ms = 1;
static int32_t      auto_current = 0;

/* --- reuse example: LED task is just another observer ------------------ */
static void led_observer(const bms_status_t *st, void *ctx)
{
    (void)ctx;
    for (uint8_t i = 0; i < 4 && i < BMS_NUM_CELLS; ++i)
        digitalWrite(BAL_LED[i], bms_cell_should_balance(&bms, i) ? HIGH : LOW);
    bool blink = (millis() / 250) & 1;
    digitalWrite(ALARM_LED, st->level == BMS_IMB_CRITICAL ? HIGH :
                            st->level == BMS_IMB_WARNING  ? blink : LOW);
}

static void sim_balance_observer(const bms_status_t *st, void *ctx)
{
    (void)ctx;
    for (uint8_t i = 0; i < BMS_NUM_CELLS; ++i)
        sim_pack_set_bleed(&pack, i, (st->balance_mask[i >> 3] >> (i & 7)) & 1);
}

static uint16_t adc_to_cell_mv(int raw) { return 3000 + (uint32_t)raw * 1200 / 4095; }
static int32_t  adc_to_ma(int raw)      { return -2500 + (int32_t)raw * 7500 / 4095; }

/* simple repeating drive cycle for AUTO mode (seconds of sim time) */
static int32_t auto_profile(uint32_t t_s, const bms_status_t *st)
{
    uint32_t p = t_s % 7200;
    if (p < 2400) return 2500;                          /* 1C discharge    */
    if (p < 2700) return 0;
    if (p < 6900) return st->max_mv < 4150 ? -1250 : -200;  /* charge      */
    return 0;
}

void setup()
{
    Serial.begin(115200);
    for (uint8_t i = 0; i < 4; ++i) pinMode(BAL_LED[i], OUTPUT);
    pinMode(ALARM_LED, OUTPUT);
    pinMode(MODE_SW, INPUT_PULLUP);
    analogReadResolution(12);

    bms_init(&bms);
    sim_pack_init(&pack, 2026, 0.80f);
    bms_register_observer(&bms, led_observer, NULL);
    bms_register_observer(&bms, sim_balance_observer, NULL);

    Serial.printf("\nModular BMS engine on ESP32 — %uS, engine RAM %u bytes\n",
                  (unsigned)BMS_NUM_CELLS, (unsigned)sizeof(bms_engine_t));
}

void loop()
{
    static uint32_t last_print = 0;
    bool manual = digitalRead(MODE_SW) == HIGH;
    bms_sample_t s;

    if (manual && BMS_NUM_CELLS == 4) {
        for (uint8_t i = 0; i < BMS_NUM_CELLS; ++i)
            s.cell_mv[i] = adc_to_cell_mv(analogRead(CELL_ADC[i]));
        s.pack_current_ma = adc_to_ma(analogRead(CURRENT_ADC));
        sim_ms += 200;                                  /* real time      */
    } else {
        sim_ms += 10000;                                /* 10 s per step  */
        auto_current = auto_profile(sim_ms / 1000, bms_get_status(&bms));
        sim_pack_step(&pack, (float)auto_current, 10.0f);
        sim_pack_read(&pack, (float)auto_current, s.cell_mv);
        s.pack_current_ma = auto_current;
    }
    s.timestamp_ms = sim_ms;

    uint32_t t0 = micros();
    bms_update(&bms, &s);
    uint32_t us = micros() - t0;

    if (millis() - last_print > 1000) {
        last_print = millis();
        const bms_status_t *st = bms_get_status(&bms);
        Serial.printf("\n[%s] t=%lus I=%ldmA SoC=%.1f%% (%.2fC)  update=%luus\n",
                      manual ? "MANUAL" : "AUTO", (unsigned long)(sim_ms / 1000),
                      (long)st->pack_current_ma, st->soc_pct, st->c_rate,
                      (unsigned long)us);
        for (uint8_t i = 0; i < BMS_NUM_CELLS; ++i)
            Serial.printf("  C%u %4umV%s%s%s\n", i + 1, st->cell_mv[i],
                          i == st->weakest_idx ? "  <-- weakest" : "",
                          i == st->strongest_idx ? "  <-- strongest" : "",
                          bms_cell_should_balance(&bms, i) ? "  [BLEED]" : "");
        Serial.printf("  imbalance %umV  trend %s (%.3f mV/s)\n", st->imbalance_mv,
                      bms_trend_str(st->trend), st->imbalance_slope_mv_s);
        Serial.printf("  adaptive warn/crit %.0f/%.0f mV -> %s\n",
                      st->warn_threshold_mv, st->crit_threshold_mv,
                      bms_level_str(st->level));
    }
    delay(200);
}
