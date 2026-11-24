#ifndef __INA226_H
#define __INA226_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// INA226 I2C address (A0=GND, A1=GND), 8-bit form (7-bit 0x40 shifted left)
#define INA226_ADDR 0x80

// Registers
#define INA226_REG_CONFIG      0x00
#define INA226_REG_SHUNTVOLT   0x01
#define INA226_REG_BUSVOLT     0x02
#define INA226_REG_POWER       0x03
#define INA226_REG_CURRENT     0x04
#define INA226_REG_CALIBRATION 0x05
#define INA226_REG_MASK_ENABLE 0x06  // Mask/Enable register
#define INA226_REG_ALERT_LIMIT 0x07  // Alert Limit register
#define INA226_REG_MFG_ID      0xFE  // Manufacturer ID
#define INA226_REG_DIE_ID      0xFF  // Die ID

// Mask/Enable register bits
#define INA226_MASK_SOL  (1 << 15)  // Shunt Voltage Over-Voltage
#define INA226_MASK_SUL  (1 << 14)  // Shunt Voltage Under-Voltage
#define INA226_MASK_BOL  (1 << 13)  // Bus Voltage Over-Voltage
#define INA226_MASK_BUL  (1 << 12)  // Bus Voltage Under-Voltage
#define INA226_MASK_POL  (1 << 11)  // Power Over-Limit
#define INA226_MASK_CNVR (1 << 10)  // Conversion Ready
#define INA226_MASK_AFF  (1 << 4)   // Alert Function Flag
#define INA226_MASK_CVRF (1 << 3)   // Conversion Ready Flag
#define INA226_MASK_OVF  (1 << 2)   // Math Overflow Flag
#define INA226_MASK_APOL (1 << 1)   // Alert Polarity (1=active high)
#define INA226_MASK_LEN  (1 << 0)   // Alert Latch Enable

// Parameters
#define INA226_SHUNT_OHMS 0.010f   // R010 = 10 mOhm shunt on this board

// Basic functions
void INA226_Init(void);
float INA226_ReadBusVoltage(void);
float INA226_ReadCurrent(void);
float INA226_ReadPower(void);
float INA226_ReadShuntVoltage(void);
uint16_t INA226_ReadReg(uint8_t reg);
void INA226_WriteReg(uint8_t reg, uint16_t value);

// Alert functions
void INA226_SetCurrentLimit(float current_amps);
float INA226_GetCurrentLimit(void);
uint8_t INA226_IsAlertTriggered(void);
void INA226_ClearAlert(void);
void INA226_EnableCurrentAlert(void);
void INA226_DisableAlert(void);

#endif
