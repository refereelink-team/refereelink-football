CC ?= cc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror -pedantic
CPPFLAGS += -Iinclude
PYTHON ?= python3

.PHONY: all test clean
all: build/serial_bridge
build:
	mkdir -p build
build/serial_bridge: src/rf_telemetry.c examples/serial_bridge.c include/rf_telemetry.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/rf_telemetry.c examples/serial_bridge.c -o $@
build/test_telemetry: src/rf_telemetry.c tests/test_telemetry.c include/rf_telemetry.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/rf_telemetry.c tests/test_telemetry.c -lm -o $@
build/test_hal_adapter: src/rf_telemetry.c src/rf_hal_adapter.c tests/test_hal_adapter.c include/rf_hal_adapter.h include/rf_telemetry.h | build
	$(CC) $(CPPFLAGS) -Itests -DRF_HAL_HEADER='"fake_hal.h"' $(CFLAGS) src/rf_telemetry.c src/rf_hal_adapter.c tests/test_hal_adapter.c -o $@
build/hal_application.o: examples/hal_application.c include/rf_hal_adapter.h include/rf_telemetry.h | build
	$(CC) $(CPPFLAGS) -Itests -DRF_HAL_HEADER='"fake_hal.h"' $(CFLAGS) -c examples/hal_application.c -o $@
test: build/test_telemetry build/test_hal_adapter build/serial_bridge build/hal_application.o
	./build/test_telemetry
	./build/test_hal_adapter
	$(PYTHON) tests/test_bridge.py
clean:
	rm -rf build
