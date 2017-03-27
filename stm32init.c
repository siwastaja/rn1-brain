#include <stdint.h>

void stm32init();
void nmi_handler();
void invalid_handler();
void hardfault_handler();

extern void error(int code);
extern void main();
extern void uart_rx_handler();
extern void adc_int_handler();
extern void spi1_inthandler();
extern void i2c1_inthandler();

extern unsigned int _STACKTOP;

// Vector table on page 164 on the Reference Manual RM0033
unsigned int * the_nvic_vector[97] __attribute__ ((section(".nvic_vector"))) =
{
/* 0x0000                    */ (unsigned int *) &_STACKTOP,
/* 0x0004 RESET              */ (unsigned int *) stm32init,
/* 0x0008 NMI                */ (unsigned int *) nmi_handler,
/* 0x000C HARDFAULT          */ (unsigned int *) hardfault_handler,
/* 0x0010                    */ (unsigned int *) invalid_handler,
/* 0x0014                    */ (unsigned int *) invalid_handler,
/* 0x0018                    */ (unsigned int *) invalid_handler,
/* 0x001C                    */ (unsigned int *) invalid_handler,
/* 0x0020                    */ (unsigned int *) invalid_handler,
/* 0x0024                    */ (unsigned int *) invalid_handler,
/* 0x0028                    */ (unsigned int *) invalid_handler,
/* 0x002C                    */ (unsigned int *) invalid_handler,
/* 0x0030                    */ (unsigned int *) invalid_handler,
/* 0x0034                    */ (unsigned int *) invalid_handler,
/* 0x0038                    */ (unsigned int *) invalid_handler,
/* 0x003C                    */ (unsigned int *) invalid_handler,
/* 0x0040                    */ (unsigned int *) invalid_handler,
/* 0x0044                    */ (unsigned int *) invalid_handler,
/* 0x0048                    */ (unsigned int *) invalid_handler,
/* 0x004C                    */ (unsigned int *) invalid_handler,
/* 0x0050                    */ (unsigned int *) invalid_handler,
/* 0x0054                    */ (unsigned int *) invalid_handler,
/* 0x0058                    */ (unsigned int *) invalid_handler,
/* 0x005C                    */ (unsigned int *) invalid_handler,
/* 0x0060                    */ (unsigned int *) invalid_handler,
/* 0x0064                    */ (unsigned int *) invalid_handler,
/* 0x0068                    */ (unsigned int *) invalid_handler,
/* 0x006C                    */ (unsigned int *) invalid_handler,
/* 0x0070                    */ (unsigned int *) invalid_handler,
/* 0x0074                    */ (unsigned int *) invalid_handler,
/* 0x0078                    */ (unsigned int *) invalid_handler,
/* 0x007C                    */ (unsigned int *) invalid_handler,
/* 0x0080                    */ (unsigned int *) invalid_handler,
/* 0x0084                    */ (unsigned int *) invalid_handler,
/* 0x0088                    */ (unsigned int *) invalid_handler,
/* 0x008C                    */ (unsigned int *) invalid_handler,
/* 0x0090                    */ (unsigned int *) invalid_handler,
/* 0x0094                    */ (unsigned int *) invalid_handler,
/* 0x0098                    */ (unsigned int *) invalid_handler,
/* 0x009C                    */ (unsigned int *) invalid_handler,
/* 0x00A0                    */ (unsigned int *) invalid_handler,
/* 0x00A4                    */ (unsigned int *) invalid_handler,
/* 0x00A8                    */ (unsigned int *) invalid_handler,
/* 0x00AC                    */ (unsigned int *) invalid_handler,
/* 0x00B0                    */ (unsigned int *) invalid_handler,
/* 0x00B4                    */ (unsigned int *) invalid_handler,
/* 0x00B8                    */ (unsigned int *) invalid_handler,
/* 0x00BC I2C1 event         */ (unsigned int *) i2c1_inthandler,
/* 0x00C0                    */ (unsigned int *) invalid_handler,
/* 0x00C4                    */ (unsigned int *) invalid_handler,
/* 0x00C8                    */ (unsigned int *) invalid_handler,
/* 0x00CC SPI1               */ (unsigned int *) spi1_inthandler,
/* 0x00D0                    */ (unsigned int *) invalid_handler,
/* 0x00D4                    */ (unsigned int *) invalid_handler,
/* 0x00D8                    */ (unsigned int *) invalid_handler,
/* 0x00DC USART3             */ (unsigned int *) uart_rx_handler,
/* 0x00E0                    */ (unsigned int *) invalid_handler,
/* 0x00E4                    */ (unsigned int *) invalid_handler,
/* 0x00E8                    */ (unsigned int *) invalid_handler,
/* 0x00EC                    */ (unsigned int *) invalid_handler,
/* 0x00F0                    */ (unsigned int *) invalid_handler,
/* 0x00F4                    */ (unsigned int *) invalid_handler,
/* 0x00F8                    */ (unsigned int *) invalid_handler,
/* 0x00FC                    */ (unsigned int *) invalid_handler,
/* 0x0100                    */ (unsigned int *) invalid_handler,
/* 0x0104                    */ (unsigned int *) invalid_handler,
/* 0x0108                    */ (unsigned int *) invalid_handler,
/* 0x010C                    */ (unsigned int *) invalid_handler,
/* 0x0110                    */ (unsigned int *) invalid_handler,
/* 0x0114                    */ (unsigned int *) invalid_handler,
/* 0x0118                    */ (unsigned int *) invalid_handler,
/* 0x011C                    */ (unsigned int *) invalid_handler,
/* 0x0120                    */ (unsigned int *) invalid_handler,
/* 0x0124                    */ (unsigned int *) invalid_handler,
/* 0x0128                    */ (unsigned int *) invalid_handler,
/* 0x012C                    */ (unsigned int *) invalid_handler,
/* 0x0130                    */ (unsigned int *) invalid_handler,
/* 0x0134                    */ (unsigned int *) invalid_handler,
/* 0x0138                    */ (unsigned int *) invalid_handler,
/* 0x013C                    */ (unsigned int *) invalid_handler,
/* 0x0140                    */ (unsigned int *) invalid_handler,
/* 0x0144                    */ (unsigned int *) invalid_handler,
/* 0x0148                    */ (unsigned int *) invalid_handler,
/* 0x014C                    */ (unsigned int *) invalid_handler,
/* 0x0150                    */ (unsigned int *) invalid_handler,
/* 0x0154                    */ (unsigned int *) invalid_handler,
/* 0x0158                    */ (unsigned int *) invalid_handler,
/* 0x015C                    */ (unsigned int *) invalid_handler,
/* 0x0160                    */ (unsigned int *) invalid_handler,
/* 0x0164                    */ (unsigned int *) invalid_handler,
/* 0x0168                    */ (unsigned int *) invalid_handler,
/* 0x016C                    */ (unsigned int *) invalid_handler,
/* 0x0170                    */ (unsigned int *) invalid_handler,
/* 0x0174                    */ (unsigned int *) invalid_handler,
/* 0x0178                    */ (unsigned int *) invalid_handler,
/* 0x017C                    */ (unsigned int *) invalid_handler,
/* 0x0180                    */ (unsigned int *) invalid_handler
};

extern unsigned int _BSS_BEGIN;
extern unsigned int _BSS_END;

extern unsigned int _DATA_BEGIN;
extern unsigned int _DATA_END;
extern unsigned int _DATAI_BEGIN;

extern unsigned int _BOOST_BEGIN;
extern unsigned int _BOOST_END;
extern unsigned int _BOOSTI_BEGIN;

void stm32init(void)
{
	uint32_t* bss_begin = (uint32_t*)&_BSS_BEGIN;
	uint32_t* bss_end   = (uint32_t*)&_BSS_END;
	while(bss_begin < bss_end)
	{
		*bss_begin = 0;
		bss_begin++;
	}

	uint32_t* data_begin  = (uint32_t*)&_DATA_BEGIN;
	uint32_t* data_end    = (uint32_t*)&_DATA_END;
	uint32_t* datai_begin = (uint32_t*)&_DATAI_BEGIN;

	while(data_begin < data_end)
	{
		*data_begin = *datai_begin;
		data_begin++;
		datai_begin++;
	}

	main();
}


void nmi_handler(void)
{
	error(1);
}

void hardfault_handler(void)
{
	error(2);
}

void invalid_handler(void)
{
	error(3);
}
