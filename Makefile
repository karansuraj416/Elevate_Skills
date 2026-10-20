# Modular BMS engine — build, test, simulate, benchmark.
#   make                 -> bms_sim (4 cells)
#   make CELLS=16        -> rebuild everything for a 16S pack
#   make test            -> unit tests for 4, 8, 16 cells
#   make run             -> run simulation with and without balancing
#   make bench           -> scaling table (bench_results.csv)
#   make plots           -> PNG charts from the logs (needs python3 + matplotlib)

CELLS  ?= 4
CC     ?= gcc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Wpedantic
INC    := -Isrc -Isim -Iapp
DEF    := -DBMS_NUM_CELLS=$(CELLS)

ENGINE := src/bms_engine.c
SIM    := sim/pack_model.c
APP    := app/main.c app/consumers.c

all: bms_sim

bms_sim: $(ENGINE) $(SIM) $(APP) src/*.h sim/*.h app/*.h
	$(CC) $(CFLAGS) $(INC) $(DEF) $(ENGINE) $(SIM) $(APP) -o $@ -lm

test:
	@for n in 4 8 16; do \
	  $(CC) $(CFLAGS) $(INC) -DBMS_NUM_CELLS=$$n $(ENGINE) bench/test_engine.c -o test_$$n -lm && ./test_$$n || exit 1; \
	done

run: bms_sim
	./bms_sim --csv logs/log_balance_on_$(CELLS)s.csv
	./bms_sim --no-balance --quiet --csv logs/log_balance_off_$(CELLS)s.csv

bench:
	@echo "cells,engine_bytes,status_bytes,sample_bytes,trend_buffer_bytes,update_ns" > bench_results.csv
	@for n in 4 8 12 16 32; do \
	  $(CC) $(CFLAGS) $(INC) -DBMS_NUM_CELLS=$$n $(ENGINE) bench/bench_scaling.c -o bench_$$n -lm && ./bench_$$n >> bench_results.csv; \
	done
	@cat bench_results.csv

plots:
	python3 tools/plot_log.py logs

clean:
	rm -f bms_sim test_* bench_[0-9]* bench_results.csv

.PHONY: all test run bench plots clean
