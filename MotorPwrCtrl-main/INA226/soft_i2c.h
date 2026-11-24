#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include <stdint.h>

// Initialize the software I2C (bit-banging) pins
void Soft_I2C_Init(void);

// Check whether a device is present (0=OK, 1=FAIL)
uint8_t Soft_I2C_IsDeviceReady(uint8_t addr);

// Write bytes to a register
uint8_t Soft_I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);

// Read bytes from a register
uint8_t Soft_I2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);

// Scan the bus for devices (optional, for debug)
void Soft_I2C_Scan(void);

#endif
