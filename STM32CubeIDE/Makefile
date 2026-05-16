# Makefile for PumpLogger — STEVAL-MKBOXPRO Rev_C
# Phase 3: SD-backed logger with sensors + GPS, no BLE yet.

ROOT       = ..
-include $(ROOT)/config.mk
TOOLCHAIN ?= $(HOME)/.software/arm-gnu-toolchain/bin
PREFIX     = $(TOOLCHAIN)/arm-none-eabi-
CC         = $(PREFIX)gcc
AS         = $(PREFIX)gcc -x assembler-with-cpp
CP         = $(PREFIX)objcopy
SZ         = $(PREFIX)size
APP        = ..

CPU = -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard

DEFS = \
  -DUSE_HAL_DRIVER \
  -DSTM32U585xx

INCLUDES = \
  -I$(APP)/Inc \
  -I$(ROOT)/Drivers/STM32U5xx_HAL_Driver/Inc \
  -I$(ROOT)/Drivers/STM32U5xx_HAL_Driver/Inc/Legacy \
  -I$(ROOT)/Drivers/CMSIS/Device/ST/STM32U5xx/Include \
  -I$(ROOT)/Drivers/CMSIS/Include

# Build counter — bumps on every `make` invocation, baked into the binary
# via -DPL_BUILD_NUM=N so ErrLog's boot banner shows "build #N" and the
# SD-card binary copy is named firmware-v<N>.bin. Idea from the Odyssey
# writeup (zdavatz/fp-sns-stbox1). The shell expression runs at Makefile
# parse time — increments + reads in one shot so embedded value matches
# filename in the same build.
BUILD_COUNTER_FILE = .build_counter
BUILD_COUNTER := $(shell \
  if [ ! -f $(BUILD_COUNTER_FILE) ]; then echo 0 > $(BUILD_COUNTER_FILE); fi; \
  N=$$(($$(cat $(BUILD_COUNTER_FILE)) + 1)); \
  echo $$N > $(BUILD_COUNTER_FILE); \
  echo $$N)

# Antenna-test build flag — when set to 1, Logger_Tick beeps N times every
# 10 s with N = satellites in fix. Direct audio feedback for swapping GPS
# antennas without needing the GUI. Default 0 (production: silent buzzer).
# Override per build: `make GPS_ANTENNA_BEEP=1`.
GPS_ANTENNA_BEEP ?= 0

CFLAGS = $(CPU) $(DEFS) $(INCLUDES) -O2 -Wall -Wno-format -Wno-unused-function \
         -fdata-sections -ffunction-sections -g3 \
         -DPL_BUILD_NUM=$(BUILD_COUNTER) \
         -DPL_GPS_ANTENNA_BEEP=$(GPS_ANTENNA_BEEP)

LDSCRIPT = STM32U585AIIXQ_FLASH.ld
LDFLAGS  = $(CPU) -T$(LDSCRIPT) -Wl,--gc-sections \
           -lm -lc --specs=nosys.specs --specs=nano.specs \
           -Wl,-Map=build/PumpLogger.map -u _printf_float

BUILD_DIR = build
TARGET    = PumpLogger

# Application sources
C_SOURCES = \
  $(APP)/Src/main.c \
  $(APP)/Src/sched.c \
  $(APP)/Src/buzzer.c \
  $(APP)/Src/watchdog.c \
  $(APP)/Src/sd_fatfs.c \
  $(APP)/Src/errlog.c \
  $(APP)/Src/sensors_imu.c \
  $(APP)/Src/sensors_mag.c \
  $(APP)/Src/sensors_baro.c \
  $(APP)/Src/sensors_fuel.c \
  $(APP)/Src/gps.c \
  $(APP)/Src/logger.c \
  $(APP)/Src/ble.c \
  $(APP)/Src/stream.c \
  $(APP)/Src/battery.c \
  $(APP)/Src/stm32u5xx_it.c \
  $(APP)/Src/system_stm32u5xx.c \
  Application/User/syscalls.c \
  Application/User/sysmem.c

# STM32 HAL — Phase 3 adds I²C, SPI, UART, SD, IWDG to the Phase 2 set.
C_SOURCES += \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_iwdg.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_sd_ex.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_spi_ex.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart_ex.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_sdmmc.c \
  $(ROOT)/Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_ll_dlyb.c

ASM_SOURCES = Application/Startup/startup_stm32u585aiixq.s

C_OBJS   = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
ASM_OBJS = $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

.PHONY: all clean force-errlog

all: force-errlog $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/firmware.bin
	@echo "build #$(BUILD_COUNTER)"

# Force errlog.c to recompile every build so __DATE__/__TIME__ inside
# ErrLog_Init's banner always reflects the actual binary on disk.
# Otherwise Make caches errlog.o for days and the build-date stamp lies.
force-errlog:
	@rm -f $(BUILD_DIR)/errlog.o

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(CPU) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(C_OBJS) $(ASM_OBJS) | $(BUILD_DIR)
	$(CC) $^ $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary $< $@

$(BUILD_DIR)/firmware.bin: $(BUILD_DIR)/$(TARGET).bin
	cp $< $@
	cp $< $(BUILD_DIR)/firmware-v$(BUILD_COUNTER).bin
	@echo "→ $(BUILD_DIR)/firmware-v$(BUILD_COUNTER).bin (and firmware.bin)"

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)
