#include "INA226.h"
#include "soft_i2c.h"

#define CURRENT_LSB 0.001f

// Write a 16-bit value to a register (big-endian)
void INA226_WriteReg(uint8_t reg, uint16_t value) {
    uint8_t buf[2];
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
    Soft_I2C_WriteReg(INA226_ADDR, reg, buf, 2);
}

uint16_t INA226_ReadReg(uint8_t reg) {
    uint8_t buf[2] = {0, 0};

    if (Soft_I2C_ReadReg(INA226_ADDR, reg, buf, 2) != 0) {
        return 0xDEAD;   // error sentinel
    }

    return ((uint16_t)buf[0] << 8) | buf[1];
}

void INA226_Init(void) {

    if (Soft_I2C_IsDeviceReady(INA226_ADDR) != 0) {
        return;
    }

    HAL_Delay(10);

    // Reset the chip
    INA226_WriteReg(INA226_REG_CONFIG, 0x8000);
    HAL_Delay(10);

    // Calibration: CAL = 0.00512 / (Current_LSB * Rshunt)
    uint16_t cal = (uint16_t)(0.00512f / (CURRENT_LSB * INA226_SHUNT_OHMS));
    INA226_WriteReg(INA226_REG_CALIBRATION, cal);
    HAL_Delay(5);

    // Configuration: continuous mode, 16-sample averaging.
    //   averaging | config  | conversion time
    //       1     | 0x4127  |  2.2 ms   (noisy)
    //       4     | 0x4327  |  8.8 ms   (better)
    //      16     | 0x4527  | 35.2 ms   (optimal, recommended)
    //      64     | 0x4727  | 140.8 ms  (stable but slow)
    INA226_WriteReg(INA226_REG_CONFIG, 0x4527);

    HAL_Delay(10);
}

float INA226_ReadBusVoltage(void) {
    uint16_t raw = INA226_ReadReg(INA226_REG_BUSVOLT);
    if (raw == 0xDEAD || raw == 0xFFFF) return 0.0f;

    // Bus voltage register: 1.25 mV per LSB.
    // Note: the +0.015 V is a calibration offset; set to 0 if not needed.
    float voltage = (raw * 0.00125f) + 0.015f;

    return voltage;
}

float INA226_ReadCurrent(void) {
    int16_t raw = (int16_t)INA226_ReadReg(INA226_REG_CURRENT);
    if (raw == (int16_t)0xDEAD || raw == (int16_t)0xFFFF) return 0.0f;
    // Current register: value * Current_LSB (with calibration applied)
    return (float)raw * CURRENT_LSB;
}

float INA226_ReadPower(void) {
    uint16_t raw = INA226_ReadReg(INA226_REG_POWER);
    if (raw == 0xDEAD || raw == 0xFFFF) return 0.0f;
    // Power register: Power_LSB = 25 * Current_LSB
    return (float)raw * CURRENT_LSB * 25.0f;
}

float INA226_ReadShuntVoltage(void) {
    int16_t raw = (int16_t)INA226_ReadReg(INA226_REG_SHUNTVOLT);
    if (raw == (int16_t)0xDEAD) return 0.0f;
    // Shunt voltage register: 2.5 uV per LSB
    return (float)raw * 0.0000025f;
}

// ==================== ALERT FUNCTIONS ====================

// Set the over-current alert limit (in amperes).
// The Alert Limit register stores a SHUNT VOLTAGE (2.5 uV per LSB),
// so we first convert amps -> shunt voltage -> register value.
void INA226_SetCurrentLimit(float current_amps) {
    float limit_voltage = current_amps * INA226_SHUNT_OHMS;     // U = I * R
    int16_t limit_raw = (int16_t)(limit_voltage / 0.0000025f); // to 2.5 uV LSB
    INA226_WriteReg(INA226_REG_ALERT_LIMIT, (uint16_t)limit_raw);
}

// Read the current alert limit back (in amperes).
float INA226_GetCurrentLimit(void) {
    int16_t limit_raw = (int16_t)INA226_ReadReg(INA226_REG_ALERT_LIMIT);
    // FIXED: the register holds a shunt voltage, not a current.
    float limit_voltage = (float)limit_raw * 0.0000025f;   // 2.5 uV per LSB -> volts
    return limit_voltage / INA226_SHUNT_OHMS;              // volts -> amps
}

// Enable the over-current alert
void INA226_EnableCurrentAlert(void) {
    // SOL (bit 15) = Shunt Voltage Over-Limit
    // LEN (bit 0)  = Latch enable (requires a clear)
    uint16_t mask = INA226_MASK_SOL | INA226_MASK_LEN;
    INA226_WriteReg(INA226_REG_MASK_ENABLE, mask);
}

// Disable the alert
void INA226_DisableAlert(void) {
    INA226_WriteReg(INA226_REG_MASK_ENABLE, 0x0000);
}

// Check whether the alert has triggered
uint8_t INA226_IsAlertTriggered(void) {
    uint16_t mask = INA226_ReadReg(INA226_REG_MASK_ENABLE);
    // Alert Function Flag (bit 4)
    return (mask & INA226_MASK_AFF) ? 1 : 0;
}

// Clear the latched alert (reading Mask/Enable clears it when LEN=1)
void INA226_ClearAlert(void) {
    volatile uint16_t temp = INA226_ReadReg(INA226_REG_MASK_ENABLE);
    (void)temp;
}
