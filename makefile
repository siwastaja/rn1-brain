# This makefile is made to work with the toolchain downloadable at https://launchpad.net/gcc-arm-embedded

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -I. -Os -fno-common -ffunction-sections -ffreestanding -fno-builtin -mthumb -mcpu=cortex-m3 -Wall -fstack-usage -Winline
ASMFLAGS = -S -fverbose-asm
LDFLAGS = -mcpu=cortex-m3 -mthumb -nostartfiles -gc-sections

DEPS = main.h gyro_xcel_compass.h lidar.h optflow.h motcons.h own_std.h flash.h sonar.h comm.h feedbacks.h sin_lut.h
OBJ = stm32init.o main.o gyro_xcel_compass.o lidar.o optflow.o motcons.o own_std.o flash.o sonar.o feedbacks.o sin_lut.o
ASMS = stm32init.s main.s gyro_xcel_compass.s lidar.s optflow.s motcons.s own_std.s flash.s sonar.s feedbacks.s sin_lut.s

all: main.bin

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main.bin: $(OBJ)
	$(LD) -Tstm32.ld $(LDFLAGS) -o main.elf $^ /usr/arm-none-eabi/lib/armv7-m/libm.a
	$(OBJCOPY) -Obinary --remove-section=.ARM* main.elf main_full.bin
	$(OBJCOPY) -Obinary --remove-section=.ARM* --remove-section=.flasher main.elf main.bin
	$(SIZE) main.elf

flash_full: main.bin
	sudo stm32sprog -b 115200 -vw main_full.bin

flash: main.bin
	sudo stm32sprog -b 115200 -vw main.bin

f: main.bin
	../rn1-tools/prog ~/dev/robo ./main.bin

f_local: main.bin
	sudo ../rn1-tools/prog /dev/ttyUSB0 ./main.bin h

ff: main.bin
	scp main.bin hrst@proto4:~/rn1-tools/

stack:
	cat *.su

sections:
	arm-none-eabi-objdump -h main.elf

syms:
	arm-none-eabi-objdump -t main.elf

%.s: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(ASMFLAGS)

asm: $(ASMS)

e: 
	nano main.c gyro_xcel_compass.h gyro_xcel_compass.c lidar.h lidar.c optflow.h optflow.c motcons.h motcons.c stm32init.c flash.h flash.c sonar.h sonar.c comm.h

s:
	sudo screen /dev/ttyUSB0 115200
