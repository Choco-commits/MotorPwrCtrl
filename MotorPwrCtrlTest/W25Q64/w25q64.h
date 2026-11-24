#ifndef __W25Q64_H
#define __W25Q64_H

#include <stdint.h>

#define W25Q64_PAGE_SIZE   256
#define W25Q64_SECTOR_SIZE 4096

void W25Q64_Init(void);
uint32_t W25Q64_ReadID(void);                                     // verify chip (expect 0xEF4017)
void W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint16_t len);
void W25Q64_WriteData(uint32_t addr, uint8_t *buf, uint16_t len); // erase + program
void W25Q64_SectorErase(uint32_t addr);

#endif
