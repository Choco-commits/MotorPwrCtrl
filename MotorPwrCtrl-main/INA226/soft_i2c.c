/*
 * soft_i2c.c - Software (bit-banging) I2C implementation.
 * Works on ANY free GPIO pins, no hardware I2C peripheral required.
 */

#include "stm32f1xx_hal.h"
#include "soft_i2c.h"

// ==================== PIN CONFIGURATION ====================
// Use any free GPIO pins. Here: PB8 (SCL), PB9 (SDA).
#define SOFT_I2C_SCL_PORT   GPIOB
#define SOFT_I2C_SCL_PIN    GPIO_PIN_8

#define SOFT_I2C_SDA_PORT   GPIOB
#define SOFT_I2C_SDA_PIN    GPIO_PIN_9

// Delay for ~100 kHz I2C (decrease to speed up)
#define I2C_DELAY_US  5

// ==================== MACROS ====================
#define SCL_HIGH()  HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET)
#define SCL_LOW()   HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET)
#define SDA_HIGH()  HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET)
#define SDA_LOW()   HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET)
#define SDA_READ()  HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN)

// ==================== DELAY ====================
static void delay_us(uint32_t us) {
    // At 72 MHz: ~72 cycles per us.
    // One loop iteration ~4 cycles -> 18 iterations per us.
    volatile uint32_t count = us * 18;
    while (count--);
}

// ==================== INITIALIZATION ====================
void Soft_I2C_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable the GPIO clock
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // SCL: output open-drain
    GPIO_InitStruct.Pin = SOFT_I2C_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // internal pull-ups (safe even if the module has none)
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SOFT_I2C_SCL_PORT, &GPIO_InitStruct);

    // SDA: output open-drain
    GPIO_InitStruct.Pin = SOFT_I2C_SDA_PIN;
    HAL_GPIO_Init(SOFT_I2C_SDA_PORT, &GPIO_InitStruct);

    // Both lines idle high
    SCL_HIGH();
    SDA_HIGH();
    delay_us(100);
}

// ==================== LOW-LEVEL OPERATIONS ====================

// START condition
static void I2C_Start(void) {
    SDA_HIGH();
    delay_us(I2C_DELAY_US);
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    SDA_LOW();   // START: SDA HIGH->LOW while SCL=HIGH
    delay_us(I2C_DELAY_US);
    SCL_LOW();
    delay_us(I2C_DELAY_US);
}

// STOP condition
static void I2C_Stop(void) {
    SDA_LOW();
    delay_us(I2C_DELAY_US);
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    SDA_HIGH();  // STOP: SDA LOW->HIGH while SCL=HIGH
    delay_us(I2C_DELAY_US);
}

// Send one byte; returns 0=ACK, 1=NACK
static uint8_t I2C_WriteByte(uint8_t data) {
    uint8_t ack;

    // 8 bits, MSB first
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }
        delay_us(I2C_DELAY_US);

        SCL_HIGH();
        delay_us(I2C_DELAY_US);
        SCL_LOW();
        delay_us(I2C_DELAY_US);

        data <<= 1;
    }

    // Read ACK
    SDA_HIGH();  // release the line
    delay_us(I2C_DELAY_US);
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    ack = SDA_READ();  // 0=ACK, 1=NACK
    SCL_LOW();
    delay_us(I2C_DELAY_US);

    return ack;
}

// Read one byte; send_ack: 1=send ACK, 0=send NACK (last byte)
static uint8_t I2C_ReadByte(uint8_t send_ack) {
    uint8_t data = 0;

    SDA_HIGH();  // release for reading

    for (int i = 0; i < 8; i++) {
        delay_us(I2C_DELAY_US);
        SCL_HIGH();
        delay_us(I2C_DELAY_US);

        data <<= 1;
        if (SDA_READ()) {
            data |= 0x01;
        }

        SCL_LOW();
    }

    // Send ACK/NACK
    if (send_ack) {
        SDA_LOW();   // ACK
    } else {
        SDA_HIGH();  // NACK
    }
    delay_us(I2C_DELAY_US);
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    SCL_LOW();
    delay_us(I2C_DELAY_US);
    SDA_HIGH();

    return data;
}

// ==================== PUBLIC FUNCTIONS ====================

uint8_t Soft_I2C_IsDeviceReady(uint8_t addr) {
    uint8_t ack;

    I2C_Start();
    ack = I2C_WriteByte(addr & 0xFE);  // write-mode address
    I2C_Stop();

    return (ack == 0) ? 0 : 1;  // 0=OK, 1=FAIL
}

uint8_t Soft_I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    I2C_Start();

    if (I2C_WriteByte(addr & 0xFE)) {  // address + write
        I2C_Stop();
        return 1;
    }

    if (I2C_WriteByte(reg)) {  // register
        I2C_Stop();
        return 1;
    }

    for (uint16_t i = 0; i < len; i++) {
        if (I2C_WriteByte(data[i])) {
            I2C_Stop();
            return 1;
        }
    }

    I2C_Stop();
    return 0;
}

uint8_t Soft_I2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    I2C_Start();

    if (I2C_WriteByte(addr & 0xFE)) {  // address + write
        I2C_Stop();
        return 1;
    }

    if (I2C_WriteByte(reg)) {  // register
        I2C_Stop();
        return 1;
    }

    // Repeated START
    I2C_Start();

    if (I2C_WriteByte(addr | 0x01)) {  // address + read
        I2C_Stop();
        return 1;
    }

    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 1) {
            data[i] = I2C_ReadByte(0);  // last byte: NACK
        } else {
            data[i] = I2C_ReadByte(1);  // ACK
        }
    }

    I2C_Stop();
    return 0;
}

// Scan the bus for devices (debug helper)
void Soft_I2C_Scan(void) {
    uint8_t found = 0;

    for (uint8_t addr = 1; addr < 128; addr++) {
        if (Soft_I2C_IsDeviceReady(addr << 1) == 0) {
            found++;
            // A device answered at this address
        }
    }
}
