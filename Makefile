PIO_ENV ?= nucleo_f411re

empty :=
space := $(empty) $(empty)
windows_pio := $(subst \,/,$(USERPROFILE))/.platformio/penv/Scripts/pio.exe
unix_pio := $(HOME)/.platformio/penv/bin/pio

ifeq ($(origin PIO),undefined)
ifneq ($(strip $(USERPROFILE)),)
ifneq ($(wildcard $(subst $(space),\$(space),$(windows_pio))),)
PIO := $(windows_pio)
endif
endif
ifeq ($(origin PIO),undefined)
ifneq ($(strip $(HOME)),)
ifneq ($(wildcard $(subst $(space),\$(space),$(unix_pio))),)
PIO := $(unix_pio)
endif
endif
endif
endif
PIO ?= pio

build_dir := $(if $(strip $(PLATFORMIO_BUILD_DIR)),$(PLATFORMIO_BUILD_DIR),.pio/build)/$(PIO_ENV)

.PHONY: all build clean

all: build

build:
	"$(PIO)" run --environment "$(PIO_ENV)"
	"$(PIO)" pkg exec --package toolchain-gccarmnoneeabi -- arm-none-eabi-objcopy -O ihex "$(build_dir)/firmware.elf" "$(build_dir)/firmware.hex"

clean:
	"$(PIO)" run --environment "$(PIO_ENV)" --target clean
