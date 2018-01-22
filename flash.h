#ifndef _FLASHER_H
#define _FLASHER_H

void unlock_flash() __attribute__((section(".flasher")));
void lock_flash() __attribute__((section(".flasher")));
void flasher() __attribute__((section(".flasher")));
int flash_erase_sector(int sector) __attribute__((section(".flasher")));


#endif
