##########################################################################################################################
# Makefile for STM32L552ZETxQ with FreeRTOS
##########################################################################################################################

######################################
# target
######################################
TARGET = lab2

######################################
# building variables
######################################
DEBUG = 1
OPT = -Og

#######################################
# paths
#######################################
BUILD_DIR = build

# STM32Cube Firmware path
STM32CUBE_PATH = C:/Users/palim/STM32Cube/Repository/STM32Cube_FW_L5_V1.5.1

# ARM GCC Toolchain path (from STM32CubeIDE)
GCC_PATH = C:/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin

# Derived paths
HAL_PATH = $(STM32CUBE_PATH)/Drivers/STM32L5xx_HAL_Driver
CMSIS_PATH = $(STM32CUBE_PATH)/Drivers/CMSIS
BSP_PATH = $(STM32CUBE_PATH)/Drivers/BSP/STM32L5xx_Nucleo
FREERTOS_PATH = $(STM32CUBE_PATH)/Middlewares/Third_Party/FreeRTOS/Source

######################################
# source files
######################################
# Local C sources
C_SOURCES = \
Core/Src/main.c \
Core/Src/app_freertos.c \
Core/Src/stm32l5xx_it.c \
Core/Src/stm32l5xx_hal_msp.c \
Core/Src/system_stm32l5xx.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c

# HAL Driver sources
HAL_SOURCES = \
$(HAL_PATH)/Src/stm32l5xx_hal.c \
$(HAL_PATH)/Src/stm32l5xx_hal_adc.c \
$(HAL_PATH)/Src/stm32l5xx_hal_adc_ex.c \
$(HAL_PATH)/Src/stm32l5xx_hal_cortex.c \
$(HAL_PATH)/Src/stm32l5xx_hal_dma.c \
$(HAL_PATH)/Src/stm32l5xx_hal_dma_ex.c \
$(HAL_PATH)/Src/stm32l5xx_hal_exti.c \
$(HAL_PATH)/Src/stm32l5xx_hal_flash.c \
$(HAL_PATH)/Src/stm32l5xx_hal_flash_ex.c \
$(HAL_PATH)/Src/stm32l5xx_hal_flash_ramfunc.c \
$(HAL_PATH)/Src/stm32l5xx_hal_gpio.c \
$(HAL_PATH)/Src/stm32l5xx_hal_i2c.c \
$(HAL_PATH)/Src/stm32l5xx_hal_i2c_ex.c \
$(HAL_PATH)/Src/stm32l5xx_hal_icache.c \
$(HAL_PATH)/Src/stm32l5xx_hal_pwr.c \
$(HAL_PATH)/Src/stm32l5xx_hal_pwr_ex.c \
$(HAL_PATH)/Src/stm32l5xx_hal_rcc.c \
$(HAL_PATH)/Src/stm32l5xx_hal_rcc_ex.c \
$(HAL_PATH)/Src/stm32l5xx_hal_tim.c \
$(HAL_PATH)/Src/stm32l5xx_hal_tim_ex.c \
$(HAL_PATH)/Src/stm32l5xx_hal_uart.c \
$(HAL_PATH)/Src/stm32l5xx_hal_uart_ex.c \
$(HAL_PATH)/Src/stm32l5xx_hal_usart.c \
$(HAL_PATH)/Src/stm32l5xx_hal_usart_ex.c

# BSP sources
BSP_SOURCES = \
$(BSP_PATH)/stm32l5xx_nucleo.c

# FreeRTOS sources
FREERTOS_SOURCES = \
$(FREERTOS_PATH)/CMSIS_RTOS_V2/cmsis_os2.c \
$(FREERTOS_PATH)/croutine.c \
$(FREERTOS_PATH)/event_groups.c \
$(FREERTOS_PATH)/list.c \
$(FREERTOS_PATH)/queue.c \
$(FREERTOS_PATH)/stream_buffer.c \
$(FREERTOS_PATH)/tasks.c \
$(FREERTOS_PATH)/timers.c \
$(FREERTOS_PATH)/portable/MemMang/heap_4.c \
$(FREERTOS_PATH)/portable/GCC/ARM_CM33_NTZ/non_secure/port.c \
$(FREERTOS_PATH)/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c

# ASM sources
ASM_SOURCES = Core/Startup/startup_stm32l552zetxq.s

# All sources combined
ALL_C_SOURCES = $(C_SOURCES) $(HAL_SOURCES) $(BSP_SOURCES) $(FREERTOS_SOURCES)

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
CPU = -mcpu=cortex-m33
FPU = -mfpu=fpv5-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# C defines
C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32L552xx

# C includes
C_INCLUDES = \
-ICore/Inc \
-I$(HAL_PATH)/Inc \
-I$(HAL_PATH)/Inc/Legacy \
-I$(CMSIS_PATH)/Device/ST/STM32L5xx/Include \
-I$(CMSIS_PATH)/Include \
-I$(BSP_PATH) \
-I$(FREERTOS_PATH)/include \
-I$(FREERTOS_PATH)/CMSIS_RTOS_V2 \
-I$(FREERTOS_PATH)/portable/GCC/ARM_CM33_NTZ/non_secure

# Compiler flags
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

CFLAGS += -MMD -MP

# Assembler flags
ASFLAGS = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections

#######################################
# LDFLAGS
#######################################
LDSCRIPT = STM32L552ZETXQ_FLASH.ld
LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

#######################################
# build rules
#######################################
# Object files - preserve directory structure to avoid name conflicts
C_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(ALL_C_SOURCES:.c=.o)))
ASM_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

# Default target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	@echo "Build complete!"

# Create build directory
$(BUILD_DIR):
	mkdir -p $@

# Compile local sources
$(BUILD_DIR)/main.o: Core/Src/main.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/app_freertos.o: Core/Src/app_freertos.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_it.o: Core/Src/stm32l5xx_it.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_msp.o: Core/Src/stm32l5xx_hal_msp.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/system_stm32l5xx.o: Core/Src/system_stm32l5xx.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/syscalls.o: Core/Src/syscalls.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/sysmem.o: Core/Src/sysmem.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

# Compile HAL sources
$(BUILD_DIR)/stm32l5xx_hal.o: $(HAL_PATH)/Src/stm32l5xx_hal.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_adc.o: $(HAL_PATH)/Src/stm32l5xx_hal_adc.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_adc_ex.o: $(HAL_PATH)/Src/stm32l5xx_hal_adc_ex.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_cortex.o: $(HAL_PATH)/Src/stm32l5xx_hal_cortex.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_dma.o: $(HAL_PATH)/Src/stm32l5xx_hal_dma.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_dma_ex.o: $(HAL_PATH)/Src/stm32l5xx_hal_dma_ex.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_exti.o: $(HAL_PATH)/Src/stm32l5xx_hal_exti.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_flash.o: $(HAL_PATH)/Src/stm32l5xx_hal_flash.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_flash_ex.o: $(HAL_PATH)/Src/stm32l5xx_hal_flash_ex.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_flash_ramfunc.o: $(HAL_PATH)/Src/stm32l5xx_hal_flash_ramfunc.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_gpio.o: $(HAL_PATH)/Src/stm32l5xx_hal_gpio.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_i2c.o: $(HAL_PATH)/Src/stm32l5xx_hal_i2c.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_i2c_ex.o: $(HAL_PATH)/Src/stm32l5xx_hal_i2c_ex.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_icache.o: $(HAL_PATH)/Src/stm32l5xx_hal_icache.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_pwr.o: $(HAL_PATH)/Src/stm32l5xx_hal_pwr.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_pwr_ex.o: $(HAL_PATH)/Src/stm32l5xx_hal_pwr_ex.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_rcc.o: $(HAL_PATH)/Src/stm32l5xx_hal_rcc.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_rcc_ex.o: $(HAL_PATH)/Src/stm32l5xx_hal_rcc_ex.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_tim.o: $(HAL_PATH)/Src/stm32l5xx_hal_tim.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_tim_ex.o: $(HAL_PATH)/Src/stm32l5xx_hal_tim_ex.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_uart.o: $(HAL_PATH)/Src/stm32l5xx_hal_uart.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_uart_ex.o: $(HAL_PATH)/Src/stm32l5xx_hal_uart_ex.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_usart.o: $(HAL_PATH)/Src/stm32l5xx_hal_usart.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32l5xx_hal_usart_ex.o: $(HAL_PATH)/Src/stm32l5xx_hal_usart_ex.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

# Compile BSP sources
$(BUILD_DIR)/stm32l5xx_nucleo.o: $(BSP_PATH)/stm32l5xx_nucleo.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

# Compile FreeRTOS sources
$(BUILD_DIR)/cmsis_os2.o: $(FREERTOS_PATH)/CMSIS_RTOS_V2/cmsis_os2.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/croutine.o: $(FREERTOS_PATH)/croutine.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/event_groups.o: $(FREERTOS_PATH)/event_groups.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/list.o: $(FREERTOS_PATH)/list.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/queue.o: $(FREERTOS_PATH)/queue.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stream_buffer.o: $(FREERTOS_PATH)/stream_buffer.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/tasks.o: $(FREERTOS_PATH)/tasks.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/timers.o: $(FREERTOS_PATH)/timers.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/heap_4.o: $(FREERTOS_PATH)/portable/MemMang/heap_4.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/port.o: $(FREERTOS_PATH)/portable/GCC/ARM_CM33_NTZ/non_secure/port.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/portasm.o: $(FREERTOS_PATH)/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

# Compile startup assembly
$(BUILD_DIR)/startup_stm32l552zetxq.o: Core/Startup/startup_stm32l552zetxq.s | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

# Generate hex
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@

# Generate bin
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(BIN) $< $@

#######################################
# clean
#######################################
clean:
	rm -rf $(BUILD_DIR)

#######################################
# flash
#######################################
flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $< 0x8000000

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean flash
