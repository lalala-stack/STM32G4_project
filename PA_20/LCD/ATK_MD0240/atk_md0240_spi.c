/**
 ****************************************************************************************************
 * @file        atk_md0240_spi.c
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
 * 公司网址:www.alientek.comx
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "atk_md0240_spi.h"
#include "spi.h"

static SPI_HandleTypeDef g_spi_handle = {0};

/**
 * @brief       ATK-MD0240模块SPI接口初始化
 * @param       无
 * @retval      无
 */
void atk_md0240_spi_init(void)
{
//    GPIO_InitTypeDef gpio_init_struct = {0};
//    
//    /* 使能时钟 */
//    ATK_MD0240_SPI_CLK_ENABLE();
//    ATK_MD0240_SPI_SCK_GPIO_CLK_ENABLE();
//    ATK_MD0240_SPI_SDA_GPIO_CLK_ENABLE();
//    
//    /* 初始化SCK引脚 */
//    gpio_init_struct.Pin    = ATK_MD0240_SPI_SCK_GPIO_PIN;
//    gpio_init_struct.Mode   = GPIO_MODE_AF_PP;
//    gpio_init_struct.Pull   = GPIO_PULLUP;
//    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(ATK_MD0240_SPI_SCK_GPIO_PORT, &gpio_init_struct);
//    
//    /* 初始化SDA引脚 */
//    gpio_init_struct.Pin    = ATK_MD0240_SPI_SDA_GPIO_PIN;
//    gpio_init_struct.Mode   = GPIO_MODE_AF_PP;
//    gpio_init_struct.Pull   = GPIO_PULLUP;
//    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(ATK_MD0240_SPI_SDA_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化SPI */
//    g_spi_handle.Instance               = ATK_MD0240_SPI_INTERFACE;
//    g_spi_handle.Init.Mode              = SPI_MODE_MASTER;
//    g_spi_handle.Init.Direction         = SPI_DIRECTION_2LINES;
//    g_spi_handle.Init.DataSize          = SPI_DATASIZE_8BIT;
//    g_spi_handle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
//    g_spi_handle.Init.CLKPhase          = SPI_PHASE_2EDGE;
//    g_spi_handle.Init.NSS               = SPI_NSS_SOFT;
//    g_spi_handle.Init.BaudRatePrescaler = ATK_MD0240_SPI_PRESCALER;
//    g_spi_handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
//    g_spi_handle.Init.TIMode            = SPI_TIMODE_DISABLE;
//    g_spi_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
//    g_spi_handle.Init.CRCPolynomial     = 7;
    //HAL_SPI_Init(&g_spi_handle);
}

/**
 * @brief       ATK-MD0240模块SPI接口发送数据
 * @param       无
 * @retval      无
 */
void atk_md0240_spi_send(uint8_t *buf, uint16_t len)
{
    HAL_SPI_Transmit(&hspi2, buf, len, HAL_MAX_DELAY);
}
