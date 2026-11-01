# This environment name is historical; platformio.ini selects the F401RE.
PIO_ENV ?= nucleo_f411re

ifeq ($(OS),Windows_NT)
PYTHON ?= python
else
PYTHON ?= python3
endif
PIO ?=
pio = "$(PYTHON)" tools/run_pio.py --pio "$(PIO)" --

build_dir := $(if $(strip $(PLATFORMIO_BUILD_DIR)),$(PLATFORMIO_BUILD_DIR),.pio/build)/$(PIO_ENV)

.PHONY: all build clean

all: build

build:
	$(pio) run --environment "$(PIO_ENV)"
	$(pio) pkg exec --package toolchain-gccarmnoneeabi -- arm-none-eabi-objcopy -O ihex "$(build_dir)/firmware.elf" "$(build_dir)/firmware.hex"

clean:
	$(pio) run --environment "$(PIO_ENV)" --target clean
