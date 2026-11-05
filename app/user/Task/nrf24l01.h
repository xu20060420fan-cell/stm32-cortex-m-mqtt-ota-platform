/*
 * nrf24l01.h  ——  NRF24L01+ 2.4GHz 无线收发模块驱动（SPI2 + GPIO，HAL 版）
 *
 *  Created on: Oct 24, 2026
 *      Author: Administrator
 *
 * 移植自 esp32mqtt 工程的 nrf24.cpp（Arduino/ESP32 版），
 * 改成 STM32 HAL + FreeRTOS 风格：
 *   - SPI 用 HAL_SPI_TransmitReceive 一帧收发（命令字节和数据在同一帧里，
 *     CSN 全程拉低，和 Arduino 的 SPI.beginTransaction/endTransaction 语义一致）
 *   - CE / CSN / IRQ 走普通 GPIO
 *   - 延时不用 HAL_Delay（会阻塞其它任务）：毫秒级用 vTaskDelay 让出 CPU，
 *     微秒级（CE 的 15us 脉冲）用 DWT 周期计数器忙等
 *
 * ------------------------------------------------------------------
 * 【接线 —— 这是新加的接线，要改只改下面这一段宏】
 *
 *   SPI2  SCK  = PB13    工程里 MX_SPI2_Init / HAL_SPI_MspInit 已经配好
 *   SPI2  MISO = PB14    工程里**没有配**（.ioc 里只给了 PB13/PB15），
 *                        不配的话主机模式下 MISO 悬空，读回来的全是 0x00/0xFF，
 *                        本驱动 nrf24_init() 里会自己补成 GPIO_MODE_AF_PP
 *   SPI2  MOSI = PB15    工程里已经配好
 *   CSN           = PB12   新加，STM32_MQTT_OTA_Node.ioc 里空闲
 *   CE            = PB9    新加，STM32_MQTT_OTA_Node.ioc 里空闲
 *   IRQ           = PB1    新加，STM32_MQTT_OTA_Node.ioc 里空闲（本驱动用轮询，不接也能跑）
 *
 *   接线提示：nRF24L01+ 模块 VCC=3.3V、GND 共地，CE/CSN 接 3.3V 逻辑。
 *             模块带 PA+LNA 时供电要能扛住 115mA 发射峰值。
 * ------------------------------------------------------------------
 */
#ifndef TASK_NRF24L01_H_
#define TASK_NRF24L01_H_

#include "stm32f1xx_hal.h"

/* ===================== 接线宏（新加的接线，改这里） ===================== */
#define NRF24_SPI_SCK_GPIO_Port  GPIOB
#define NRF24_SPI_SCK_Pin         GPIO_PIN_13
#define NRF24_SPI_MISO_GPIO_Port  GPIOB
#define NRF24_SPI_MISO_Pin        GPIO_PIN_14
#define NRF24_SPI_MOSI_GPIO_Port  GPIOB
#define NRF24_SPI_MOSI_Pin        GPIO_PIN_15

#define NRF24_CSN_GPIO_Port       GPIOB
#define NRF24_CSN_Pin             GPIO_PIN_12
/* 按 PCB 网络核对（2026-11-02 用户提供实物图）：
 *   3=PB9(CE)，4=PB12(CSN)，5=PB13(SCK)，6=PB15(MOSI)，7=PB14(MISO)，8=PB1(IRQ)。
 * 与下面定义完全一致。 */
#define NRF24_CE_GPIO_Port        GPIOB
#define NRF24_CE_Pin              GPIO_PIN_9
#define NRF24_IRQ_GPIO_Port       GPIOB
#define NRF24_IRQ_Pin             GPIO_PIN_1

/* ===================== 射频参数（照搬 esp32mqtt 参考实现） ===================== */
#define NRF24_ADDR_LEN          5U      /* 5 字节地址 */
#define NRF24_PAYLOAD_SIZE      32U     /* 固定载荷 32 字节 */
/* 76 = 2476MHz，落在 2.4G ISM（2400~2483.5）里。
 * 原来 125 = 2525MHz 已出频段，PA+LNA 模块的带通滤波器会把它衰减掉。 */
#define NRF24_DEFAULT_CHANNEL   76U
#define NRF24_MAX_CHANNEL       125U

/* 默认地址 "NODE1"（'N','O','D','E','1'） */
#define NRF24_DEFAULT_ADDR      { 0x4EU, 0x4FU, 0x44U, 0x45U, 0x31U }

/* ===================== 对外 API ===================== */

/**
 * @brief  初始化 nRF24L01+：配 GPIO、限 SPI 时钟、写全部寄存器并读回校验
 * @retval 1:成功  0:失败（原因已打到串口）
 */
uint8_t nrf24_init(void);

/** 射频是否已初始化成功（1=就绪）。上层用它决定要不要运行期重试。 */
uint8_t nrf24_inited(void);

/**
 * @brief  切换到发送模式（PTX）：重新配置射频参数并写 TX_ADDR
 * @retval 1:成功  0:失败
 * @note   调用后用 nrf24_write() 发包；发完用 nrf24_end_tx() 切回接收
 */
uint8_t nrf24_begin_tx(void);

/**
 * @brief  从发送模式切回接收模式（PRX）：CE 拉高开始监听
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_end_tx(void);

/**
 * @brief  发送一包数据（固定 32 字节，短包补 0）
 * @param  buf: 数据缓冲区
 * @param  len: 有效长度，1~32
 * @retval 1:发送成功（收到 ACK）  0:失败（超时/未收到 ACK）
 */
uint8_t nrf24_write(const uint8_t *buf, uint8_t len);

/**
 * @brief  接收缓冲里有没有待取的数据
 * @retval 1:有  0:没有
 */
uint8_t nrf24_available(void);

/**
 * @brief  取一包数据（固定 32 字节）
 * @param  buf: 至少 32 字节的接收缓冲
 * @retval 32:取到一包  0:没有数据/失败
 */
uint8_t nrf24_read(uint8_t *buf);

/**
 * @brief  设置射频信道（0~125）
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_set_channel(uint8_t ch);

/**
 * @brief  设置发送地址（5 字节），并读回确认
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_set_tx_addr(const uint8_t *addr);

/**
 * @brief  设置接收地址（5 字节，写 pipe 0），并读回确认
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_set_rx_addr(const uint8_t *addr);

/**
 * @brief  读回当前接收地址（pipe 0）
 * @param  addr: 5 字节缓冲
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_get_rx_addr(uint8_t *addr);

/**
 * @brief  读 STATUS 寄存器
 * @param  status: 输出状态值
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_get_status(uint8_t *status);

/**
 * @brief  通用寄存器读（寄存器地址只有 5 位，0x00~0x1F）
 * @param  reg: 寄存器号
 * @param  val: 输出寄存器值
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_read_reg(uint8_t reg, uint8_t *val);

/**
 * @brief  IRQ 引脚当前是否为低（模块拉低表示有事件）
 * @retval 1:有事件  0:无事件
 */
uint8_t nrf24_irq_pending(void);

/**
 * @brief  进入掉电模式（Power-Down）
 * @note   电流 ~26µA，而 RX 模式是 ~12mA —— 这是板子上第二大耗电项。
 *         发射流程（nrf24_begin_tx）本来就先掉电再上电，所以随时可以掉。
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_power_down(void);

/**
 * @brief  从掉电模式唤醒并进入 RX（会先清空 RX FIFO，避免读到掉电前的残留包）
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_power_up_rx(void);

#endif /* TASK_NRF24L01_H_ */
