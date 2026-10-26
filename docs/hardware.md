# 硬件配置与引脚映射

[返回项目首页](../README.md) · [文档索引](README.md)

## 硬件与机械

### 飞行器

| 项目 | 配置 |
|---|---|
| 主控 | STM32F411CEU6 |
| 电池 | 单节 3.7 V、600 mAh 航模锂电池 |
| 电机 | 4 × 720 空心杯电机 |
| 桨叶 | 720 空心杯电机配套桨叶 |
| 轴距 | 约 `7.5 × √2 cm` |
| 姿态传感器 | ICM20948 |
| 气压计 | BMP280 |
| 遥控通信 | Si24R1 2.4 GHz |
| 调试通信 | ESP8684 UART/Wi-Fi 数据转发 |
| 光流与测距 | 两块独立扩展模块 |

### 遥控器

| 项目 | 配置 |
|---|---|
| 主控 | STM32F103C8T6 |
| 系统时钟 | 8 MHz 外部晶振 |
| 供电 | 单节 3.7 V、600 mAh 航模锂电池 |
| 操纵输入 | 双摇杆，横滚、俯仰、偏航和油门四个 ADC 通道 |
| 功能输入 | 6 个独立按键 |
| 遥控通信 | NRF24L01 2.4 GHz |
| 显示 | I2C OLED |
| 状态提示 | WS2812B、用户 LED 和蜂鸣器 |
| 有线调试 | USB Type-C + CH340N USB 转串口 |
| 下载调试 | SWD |
| 参数保存 | STM32 内部 Flash，用于摇杆校准参数等配置 |

## 飞控接口

下表以现有飞控原理图和旧版驱动为基础，最终以 DreamFly2 的 CubeMX 配置为准。

| MCU 资源 | 引脚 | 用途 | 连接对象 |
|---|---|---|---|
| ADC1_IN0 | PA0 | 电池电压采样 | VBAT 分压电路 |
| GPIO | PA1 | SPI 片选 | 光流模块 |
| USART2_TX | PA2 | 调试串口发送 | ESP8684 |
| USART2_RX | PA3 | 调试串口接收 | ESP8684 |
| GPIO/EXTI | PA4 | 数据就绪中断 | ICM20948 INT |
| SPI1_SCK | PA5 | 光流 SPI 时钟 | 光流模块 |
| SPI1_MISO | PA6 | 光流 SPI 输入 | 光流模块 |
| SPI1_MOSI | PA7 | 光流 SPI 输出 | 光流模块 |
| GPIO | PA8 | 无线使能 | Si24R1 CE |
| USART1_TX | PA9 | 用户调试串口发送 | Debug UART1 |
| USART1_RX | PA10 | 用户调试串口接收 | Debug UART1 |
| SWDIO | PA13 | 程序下载与调试 | SWD |
| SWCLK | PA14 | 程序下载与调试 | SWD |
| TIM3_CH3 | PB0 | 电机 PWM | Motor 3 |
| TIM3_CH4 | PB1 | 电机 PWM | Motor 4 |
| GPIO/EXTI | PB2 | 无线中断 | Si24R1 IRQ |
| GPIO | PB4 | 软件 I2C 时钟 | 激光测距模块 |
| GPIO | PB5 | 软件 I2C 数据 | 激光测距模块 |
| TIM4_CH1 | PB6 | 电机 PWM | Motor 1 |
| TIM4_CH2 | PB7 | 电机 PWM | Motor 2 |
| I2C1_SCL | PB8 | 板载 I2C 时钟 | ICM20948、BMP280 |
| I2C1_SDA | PB9 | 板载 I2C 数据 | ICM20948、BMP280 |
| TIM2_CH3 | PB10 | RGB 数据 | 4 × WS2812B |
| GPIO | PB12 | 无线片选 | Si24R1 CSN（软件控制） |
| SPI2_SCK | PB13 | 无线 SPI 时钟 | Si24R1 |
| SPI2_MISO | PB14 | 无线 SPI 输入 | Si24R1 |
| SPI2_MOSI | PB15 | 无线 SPI 输出 | Si24R1 |
| GPIO | PC13 | 状态指示 | User LED |

## 遥控器接口

下表结合遥控器原理图与已确认的当前 CubeMX 配置整理。

| MCU 资源 | 引脚 | 用途 | 连接对象 |
|---|---|---|---|
| ADC1_IN0 | PA0 | 遥控器电池电压采样 | VBAT 分压电路 |
| ADC1_IN1 | PA1 | 横滚输入 | 右摇杆 X 轴 |
| ADC1_IN2 | PA2 | 俯仰输入 | 右摇杆 Y 轴 |
| GPIO | PA3 | 无线使能 | NRF24L01 CE |
| GPIO | PA4 | 无线片选 | NRF24L01 CSN |
| SPI1_SCK | PA5 | 无线 SPI 时钟 | NRF24L01 |
| SPI1_MISO | PA6 | 无线 SPI 输入 | NRF24L01 |
| SPI1_MOSI | PA7 | 无线 SPI 输出 | NRF24L01 |
| GPIO | PA8 | 状态指示 | User LED |
| USART1_TX | PA9 | 串口发送 | CH340N |
| USART1_RX | PA10 | 串口接收 | CH340N |
| SWDIO | PA13 | 程序下载与调试 | SWD |
| SWCLK | PA14 | 程序下载与调试 | SWD |
| ADC1_IN8 | PB0 | 偏航输入 | 左摇杆 X 轴 |
| ADC1_IN9 | PB1 | 油门输入 | 左摇杆 Y 轴 |
| GPIO | PB5 | 声音提示 | 蜂鸣器驱动电路 |
| I2C1_SCL | PB6 | OLED 时钟 | OLED |
| I2C1_SDA | PB7 | OLED 数据 | OLED |
| GPIO/EXTI | PB3 | 无线中断 | NRF24L01 IRQ |
| TIM3_CH1 | PB4 | RGB 数据 | WS2812B（TIM3 部分重映射） |
| GPIO | PB10 | 按键输入 | Key 1 |
| GPIO | PB11 | 按键输入 | Key 2 |
| GPIO | PB12 | 按键输入 | Key 3 |
| GPIO | PB13 | 按键输入 | Key 4 |
| GPIO | PB14 | 按键输入 | Key 5 |
| GPIO | PB15 | 按键输入 | Key 6 |

> 当前工程已确认无线 IRQ 使用 PB3、WS2812B 使用 PB4。仓库旧版原理图中的 PB8、PB9 标注与当前接线不同，使用旧图时需注意这一差异。

遥控器不使用外部 EEPROM。旧版程序已包含 STM32 内部 Flash 参数读写代码，但摇杆校准参数的部分写入调用处于注释状态；新版需要重新实现并验证参数保存流程。
