CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -O2 -ffunction-sections -fdata-sections -Wall -Wextra
CFLAGS += -DPART_TM4C123GH6PM -DTARGET_IS_TM4C123_RB1

LDFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -T tm4c123gh6pm.ld -Wl,--gc-sections -specs=nosys.specs

INCLUDES = -I. -IFreeRTOS/Source/include -IFreeRTOS/Source/portable/GCC/ARM_CM4F
INCLUDES += -Idriverlib -Iinc

SOURCES = main.c startup.s
SOURCES += FreeRTOS/Source/tasks.c FreeRTOS/Source/queue.c FreeRTOS/Source/list.c
SOURCES += FreeRTOS/Source/timers.c FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
SOURCES += FreeRTOS/Source/portable/MemMang/heap_1.c

OBJECTS = $(SOURCES:.c=.o)
OBJECTS := $(OBJECTS:.s=.o)

TARGET = seat_heater_control

all: $(TARGET).bin

$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^ -ldriverlib

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@
	$(SIZE) $<

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.s
	$(AS) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET).elf $(TARGET).bin

flash: $(TARGET).bin
	lm4flash $(TARGET).bin

.PHONY: all clean flash
