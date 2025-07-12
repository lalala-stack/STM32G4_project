/**
 ****************************************************************************************************
 * @file        atk_md0240_spi.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MD0240模块SPI接口驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 SCK --> PB13
 SDA --> PB15
 初始函数还初始化了PB14但是此处只需要单向通信，不需要接线
 
 
 */

#ifndef __ATK_MD0240_SPI_H
#define __ATK_MD0240_SPI_H

#include "main.h"

/* SPI接口定义 */
#define ATK_MD0240_SPI_INTERFACE                SPI2
#define ATK_MD0240_SPI_PRESCALER                SPI_BAUDRATEPRESCALER_16
#define ATK_MD0240_SPI_CLK_ENABLE()             do{ __HAL_RCC_SPI2_CLK_ENABLE(); }while(0)

/* 引脚定义 */
#define ATK_MD0240_SPI_SCK_GPIO_PORT            GPIOB
#define ATK_MD0240_SPI_SCK_GPIO_PIN             GPIO_PIN_13
#define ATK_MD0240_SPI_SCK_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)
#define ATK_MD0240_SPI_SDA_GPIO_PORT            GPIOB
#define ATK_MD0240_SPI_SDA_GPIO_PIN             GPIO_PIN_15
#define ATK_MD0240_SPI_SDA_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

/* 操作函数 */
void atk_md0240_spi_init(void);                         /* ATK-MD0240模块SPI接口初始化 */
void atk_md0240_spi_send(uint8_t *buf, uint16_t len);   /* ATK-MD0240模块SPI接口发送数据 */

#endif
