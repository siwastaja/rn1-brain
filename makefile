# This makefile is made to work with the toolchain downloadable at https://launchpad.net/gcc-arm-embedded

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy



#MODEL=RN1P4
#MODEL=RN1P7
#MODEL=RN1P6
#MODEL=PULU1
MODEL=PROD1

PCBREV=PCB1B

CFLAGS = -I. -Os -fno-common -ffunction-sections -ffreestanding -fno-builtin -mthumb -mcpu=cortex-m3 -Wall -fstack-usage -Winline -D$(MODEL) -D$(PCBREV)

#CFLAGS += -DHWTEST
#CFLAGS += -DSONARS_INSTALLED
CFLAGS += -DDELIVERY_APP
#CFLAGS += -DOPTFLOW_INSTALLED
CFLAGS += -DPULUTOF1


ASMFLAGS = -S -fverbose-asm
LDFLAGS = -mcpu=cortex-m3 -mthumb -nostartfiles -gc-sections

DEPS = main.h gyro_xcel_compass.h lidar.h optflow.h motcons.h own_std.h flash.h sonar.h comm.h feedbacks.h sin_lut.h navig.h uart.h settings.h
OBJ = stm32init.o main.o gyro_xcel_compass.o lidar.o optflow.o motcons.o own_std.o flash.o sonar.o feedbacks.o sin_lut.o navig.o uart.o hwtest.o settings.o
ASMS = stm32init.s main.s gyro_xcel_compass.s lidar.s optflow.s motcons.s own_std.s flash.s sonar.s feedbacks.s sin_lut.s navig.s uart.s settings.s

all: main.bin

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main.bin: $(OBJ)
	$(LD) -Tstm32.ld $(LDFLAGS) -o main.elf $^ /usr/arm-none-eabi/lib/thumb/v7-m/libm.a
	$(OBJCOPY) -Obinary --remove-section=.ARM* main.elf main_full.bin
	$(OBJCOPY) -Obinary --remove-section=.ARM* --remove-section=.flasher main.elf main.bin
	$(SIZE) main.elf

flash_full: main.bin
	stm32sprog -b 115200 -vw main_full.bin

flash: main.bin
	stm32sprog -b 115200 -vw main.bin

f: main.bin
	scp main.bin hrst@$(robot):~/rn1-tools/

ff: main.bin
	scp main_full.bin hrst@$(robot):~/rn1-tools/main.bin

f_local: main.bin
	../rn1-tools/prog /dev/ttyUSB0 ./main.bin h

f_proto4: main.bin
	scp main.bin hrst@proto4:~/rn1-tools/

f_helsinki1: main.bin
	scp main.bin hrst@helsinki1:~/rn1-tools/

f_proto5: main.bin
	scp main.bin hrst@proto5:~/rn1-tools/

f_proto6: main.bin
	scp main.bin hrst@proto6:~/rn1-tools/

f_pulu1: main.bin
	scp main.bin hrst@pulu1:~/rn1-tools/

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
	gedit --new-window main.c feedbacks.h feedbacks.c navig.h navig.c lidar.h lidar.c lidar_corr.h lidar_corr.c uart.h uart.c motcons.c motcons.h sonar.c sonar.h flash.h flash.c settings.h settings.c &
s:
	screen /dev/ttyUSB0 115200
