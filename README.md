# Modular Battery Management Engine

Scalable BMS analysis engine in portable C99. One constant (`BMS_NUM_CELLS`) sets the pack size; the engine finds the weakest/strongest cell, measures imbalance, tracks its trend, and judges it against SoC- and C-rate-adaptive thresholds. Every other task reads one shared status snapshot.

Full design write-up: **[docs/REPORT.md](docs/REPORT.md)**

```
bms_project/
├── src/            engine (the reusable module)
│   ├── bms_config.h      <- change BMS_NUM_CELLS here
│   ├── bms_engine.h      public interface
│   └── bms_engine.c
├── sim/            battery pack model (the "plant")
├── app/            PC runner + example consumer tasks (balancer, logger, alarm)
├── bench/          unit tests + scaling benchmark
├── wokwi_esp32/    ESP32 sketch + wiring for the Wokwi simulator
├── tools/          plot_log.py (matplotlib)
├── logs/           sample CSV logs and plots (4S and 16S)
└── docs/           REPORT.md, scaling.png
```

## Simulator 1 — Wokwi (browser, free, recommended for demo)

Wokwi simulates an ESP32 with potentiometers, LEDs and a serial monitor in the browser — no hardware or install needed.

1. Go to **https://wokwi.com** → *New Project* → **ESP32**.
2. Replace `sketch.ino` and `diagram.json` with the files from `wokwi_esp32/`.
3. Use the **+** (new file) button to add `bms_config.h`, `bms_engine.h`, `bms_engine.c`, `pack_model.h`, `pack_model.c` from the same folder.
4. Press ▶ Start.

Controls:
- **Slide switch** — left/right toggles MANUAL (pots) vs AUTO (built-in pack model, 10× speed).
- **Pots 1–4** — cell voltages 3.0–4.2 V. Turn one down: it becomes the weakest cell, the red LED blinks (WARNING) or goes solid (CRITICAL), and the yellow LEDs of the higher cells light (balance bleed).
- **Pot 5** — pack current −2.5 A … +5 A. Increase the load and watch the adaptive thresholds widen in the serial monitor.

The same sketch runs unchanged on a real ESP32 DevKit in Arduino IDE (keep all files in the sketch folder).

## Simulator 2 — PC (gcc, any OS)

```bash
make test                 # unit tests for 4, 8 and 16 cells
make                      # build 4S simulator
make run                  # scenario with and without balancing -> logs/*.csv
make CELLS=16 && make run CELLS=16
make bench                # memory + timing for 4/8/12/16/32 cells
make plots                # PNGs (pip install matplotlib)
```

Windows: install MSYS2 or use WSL; or compile directly:
`gcc -std=c99 -O2 -Isrc -Isim -Iapp src/bms_engine.c sim/pack_model.c app/main.c app/consumers.c -o bms_sim -lm`

## Using the engine in another task

```c
#include "bms_engine.h"
static bms_engine_t bms;
bms_init(&bms);

bms_sample_t s = { .pack_current_ma = 1200, .timestamp_ms = now_ms() };
read_cells(s.cell_mv);          /* your ADC / AFE driver */
bms_update(&bms, &s);

const bms_status_t *st = bms_get_status(&bms);
printf("weakest C%u, imbalance %u mV, trend %s, %s\n",
       st->weakest_idx + 1, st->imbalance_mv,
       bms_trend_str(st->trend), bms_level_str(st->level));
```
