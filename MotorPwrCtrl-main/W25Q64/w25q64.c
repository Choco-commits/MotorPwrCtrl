#include "stm32f1xx_hal.h"
#include "w25q64.h"

// hspi1 is the global SPI handle that CubeMX generates in main.c.
extern SPI_HandleTypeDef hspi1;

// Chip-select (PA4) is controlled manually because SPI NSS is in software mode.
#define W25Q64_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define W25Q64_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

void W25Q64_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // CS pin as output, idle high (deselected)
    gpio.Pin   = GPIO_PIN_4;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    W25Q64_CS_HIGH();
}

uint32_t W25Q64_ReadID(void)
{
    uint8_t cmd = 0x9F;
    uint8_t id[3] = {0};

    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, id, 3, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();

    // W25Q64 returns 0xEF 0x40 0x17
    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

static void W25Q64_WaitBusy(void)
{
    uint8_t status;
    do {
        uint8_t cmd = 0x05;
        W25Q64_CS_LOW();
        HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
        W25Q64_CS_HIGH();
    } while (status & 0x01);   // bit0 = BUSY
}

static void W25Q64_WriteEnable(void)
{
    uint8_t cmd = 0x06;
    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
}

void W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t cmd[4] = {0x03, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF};

    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
}

void W25Q64_SectorErase(uint32_t addr)
{
    uint8_t cmd[4] = {0x20, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF};

    W25Q64_WriteEnable();
    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
    W25Q64_WaitBusy();   // erase is slow, must wait
}

void W25Q64_WriteData(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t cmd[4] = {0x02, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF};

    W25Q64_SectorErase(addr);   // 1. erase first
    W25Q64_WriteEnable();       // 2. write enable again (erase clears it)

    W25Q64_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, buf, len, HAL_MAX_DELAY);
    W25Q64_CS_HIGH();
    W25Q64_WaitBusy();          // 3. wait for program to finish
}
