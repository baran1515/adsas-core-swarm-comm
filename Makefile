# A.D.S.A.S. Core - build file (Linux / WSL)
CC      ?= gcc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Werror -Iinclude
LDLIBS  := -lm
BUILD   := build

LIB_SRC := src/protocol.c src/task_alloc.c

.PHONY: all test demo clean

all: $(BUILD)/adsas_node $(BUILD)/test_adsas

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/adsas_node: $(LIB_SRC) src/node.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/test_adsas: $(LIB_SRC) tests/test_main.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

test: $(BUILD)/test_adsas
	./$(BUILD)/test_adsas

demo: $(BUILD)/adsas_node
	bash scripts/run_demo.sh

clean:
	rm -rf $(BUILD) logs
