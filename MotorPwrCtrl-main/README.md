# 基于 STM32F103 的直流电机控制与电源管理系统

基于 STM32F103C8T6 与 FreeRTOS 的直流电机速度闭环控制系统，集成电压电流监测、异常保护与参数掉电保存。

## 功能

- **电机控制**：定时器编码器模式四倍频测速，PID 速度闭环（积分抗饱和），20kHz PWM 驱动
- **电源管理**：INA226 电压电流监测，过压 / 过流保护，故障自动停机
- **掉电保存**：W25Q64 SPI Flash 保存参数，上电自动恢复
- **通信**：USART DMA + 空闲中断接收，实时遥测回传
- **多任务**：FreeRTOS 任务划分（电机控制 / 按键 / 串口收发 / 电流采样 / 参数存储），队列 + 信号量 + 任务通知通信

## 硬件平台

| 模块 | 说明 |
|---|---|
| 主控 | STM32F103C8T6（72MHz） |
| 电机 | JGA25-370 直流电机（含霍尔编码器，11 PPR） |
| 驱动 | TB6612FNG |
| 电流监测 | INA226（10mΩ 采样电阻） |
| 存储 | W25Q64 SPI Flash |
| 交互 | 按键 + 串口 |

## 引脚分配

| 功能 | 引脚 |
|---|---|
| 电机 PWM | PA8（TIM1_CH1，20kHz） |
| 编码器 A / B | PB4 / PB5（TIM3 编码器模式） |
| INA226 SCL / SDA | PB8 / PB9（软件 I2C） |
| W25Q64 CS / SCK / MISO / MOSI | PA4 / PA5 / PA6 / PA7 |
| 按键 | PA0 |
| 串口 TX / RX | PB6 / PB7 |

## 使用说明

1. 上电，串口打印使用说明。
2. 串口发送目标转速（0 ~ 2000 RPM），例如 `1000`。
3. 按键：按一下启动，再按一下停止。
4. 遥测格式：`duty / speed / target / v / i / state`（占空比 / 转速 / 目标 / 电压 / 电流 / 状态）。

## 技术栈

C · FreeRTOS · STM32 HAL · PID 控制 · DMA · 软件 I2C / SPI · UART

## 参考与致谢

本项目参考了以下开源项目与教程，特此致谢：

- [PID-Motor-Controller-STM32F103-FreeRTOS](https://github.com/usman-aslam-dev/PID-Motor-Controller-STM32F103-FreeRTOS) — FreeRTOS 多任务 + PID 电机控制架构参考（MIT License）
- STM32F103-INA226-ST7735-Indicator-Encoder-Protection-PS（作者：Aleksei SUBBOTIN）— INA226 软件 I2C 驱动参考
- SPITest（作者：铁头山羊）— W25Q64 SPI 驱动参考
