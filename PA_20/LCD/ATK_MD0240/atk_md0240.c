/**
 ****************************************************************************************************
 * @file        atk_md0240.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK_MD0240模块驱动代码
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
 */

#include "atk_md0240.h"
#include "atk_md0240_font.h"
#include "atk_md0240_spi.h"

/* 定义ATK-MD0240模块LCD显存 */
#define ATK_MD0240_LCD_BUF_SIZE (ATK_MD0240_LCD_WIDTH * ATK_MD0240_LCD_HEIGHT * sizeof(uint16_t) / 10)
static uint8_t g_atk_md0240_lcd_buf[ATK_MD0240_LCD_BUF_SIZE] = {0};

/**
 * @brief       往ATK-MD0240模块写命令
 * @param       无
 * @retval      无
 */
static void atk_md0240_write_cmd(uint8_t cmd)
{
    ATK_MD0240_WR(0);
    atk_md0240_spi_send(&cmd, sizeof(cmd));
}

/**
 * @brief       往ATK-MD0240模块写数据
 * @param       无
 * @retval      无
 */
static void atk_md0240_write_dat(uint8_t dat)
{
    ATK_MD0240_WR(1);
    atk_md0240_spi_send(&dat, sizeof(dat));
}

/**
 * @brief       往ATK-MD0240模块写16bit数据
 * @param       无
 * @retval      无
 */
static void atk_md0240_write_dat_16b(uint16_t dat)
{
    uint8_t buf[2];
    
    buf[0] = (uint8_t)(dat >> 8) & 0xFF;
    buf[1] = (uint8_t)dat & 0xFF;
    
    ATK_MD0240_WR(1);
    atk_md0240_spi_send(buf, sizeof(buf));
}

/**
 * @brief       平方函数，x^y
 * @param       x: 底数
 *              y: 指数
 * @retval      x^y
 */
static uint32_t atk_md0240_pow(uint8_t x, uint8_t y)
{
    uint8_t loop;
    uint32_t res = 1;
    
    for (loop=0; loop<y; loop++)
    {
        res *= x;
    }
    
    return res;
}

/**
 * @brief       ATK-MD0240模块硬件初始化
 * @param       无
 * @retval      无
 */
static void atk_md0240_hw_init(void)
{
//    GPIO_InitTypeDef gpio_init_struct = {0};
//    
//    /* 使能时钟 */
//    ATK_MD0240_PWR_GPIO_CLK_ENABLE();
//    ATK_MD0240_CS_GPIO_CLK_ENABLE();
//    ATK_MD0240_WR_GPIO_CLK_ENABLE();
//    ATK_MD0240_RST_GPIO_CLK_ENABLE();
//    
//    /* 初始化PWR引脚 */
//    gpio_init_struct.Pin    = ATK_MD0240_PWR_GPIO_PIN;
//    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_PP;
//    gpio_init_struct.Pull   = GPIO_PULLUP;
//    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(ATK_MD0240_PWR_GPIO_PORT, &gpio_init_struct);
//    
//    /* 初始化CS引脚 */
//    gpio_init_struct.Pin    = ATK_MD0240_CS_GPIO_PIN;
//    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_PP;
//    gpio_init_struct.Pull   = GPIO_PULLUP;
//    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(ATK_MD0240_CS_GPIO_PORT, &gpio_init_struct);
//    
//    /* 初始化WR引脚 */
//    gpio_init_struct.Pin    = ATK_MD0240_WR_GPIO_PIN;
//    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_PP;
//    gpio_init_struct.Pull   = GPIO_PULLUP;
//    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(ATK_MD0240_WR_GPIO_PORT, &gpio_init_struct);
//    
//    /* 初始化RST引脚 */
//    gpio_init_struct.Pin    = ATK_MD0240_RST_GPIO_PIN;
//    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_PP;
//    gpio_init_struct.Pull   = GPIO_PULLUP;
//    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(ATK_MD0240_RST_GPIO_PORT, &gpio_init_struct);
    
    ATK_MD0240_CS(0);
}

/**
 * @brief       ATK-MD0240模块硬件复位
 * @param       无
 * @retval      无
 */
static void atk_md0240_hw_reset(void)
{
    ATK_MD0240_RST(0);
    HAL_Delay(10);
    ATK_MD0240_RST(1);
    HAL_Delay(120);
}

/**
 * @brief       ATK-MD0240模块寄存器初始化
 * @param       无
 * @retval      无
 */
static void atk_md0240_reg_init(void)
{
    /* Sleep Out */
    atk_md0240_write_cmd(0x11);
    HAL_Delay(120);
    /* Memory Data Access Control */
    atk_md0240_write_cmd(0x36);
    atk_md0240_write_dat(0x00);
    /* RGB 5-6-5-bit  */
    atk_md0240_write_cmd(0x3A);
    atk_md0240_write_dat(0x65);
    /* Porch Setting */
    atk_md0240_write_cmd(0xB2);
    atk_md0240_write_dat(0x0C);
    atk_md0240_write_dat(0x0C);
    atk_md0240_write_dat(0x00);
    atk_md0240_write_dat(0x33);
    atk_md0240_write_dat(0x33);
    /*  Gate Control */
    atk_md0240_write_cmd(0xB7);
    atk_md0240_write_dat(0x72);
    /* VCOM Setting */
    atk_md0240_write_cmd(0xBB);
    atk_md0240_write_dat(0x3D);
    /* LCM Control */
    atk_md0240_write_cmd(0xC0);
    atk_md0240_write_dat(0x2C);
    /* VDV and VRH Command Enable */
    atk_md0240_write_cmd(0xC2);
    atk_md0240_write_dat(0x01);
    /* VRH Set */
    atk_md0240_write_cmd(0xC3);
    atk_md0240_write_dat(0x19);
    /* VDV Set */
    atk_md0240_write_cmd(0xC4);
    atk_md0240_write_dat(0x20);
    /* Frame Rate Control in Normal Mode */
    atk_md0240_write_cmd(0xC6);
    atk_md0240_write_dat(0x0F);
    /* Power Control 1 */
    atk_md0240_write_cmd(0xD0);
    atk_md0240_write_dat(0xA4);
    atk_md0240_write_dat(0xA1);
    /* Positive Voltage Gamma Control */
    atk_md0240_write_cmd(0xE0);
    atk_md0240_write_dat(0xD0);
    atk_md0240_write_dat(0x04);
    atk_md0240_write_dat(0x0D);
    atk_md0240_write_dat(0x11);
    atk_md0240_write_dat(0x24);
    atk_md0240_write_dat(0x2B);
    atk_md0240_write_dat(0x3F);
    atk_md0240_write_dat(0x54);
    atk_md0240_write_dat(0x4C);
    atk_md0240_write_dat(0x18);
    atk_md0240_write_dat(0x0D);
    atk_md0240_write_dat(0x0B);
    atk_md0240_write_dat(0x1F);
    atk_md0240_write_dat(0x23);
    /* Negative Voltage Gamma Control */
    atk_md0240_write_cmd(0xE1);
    atk_md0240_write_dat(0xD0);
    atk_md0240_write_dat(0x04);
    atk_md0240_write_dat(0x0C);
    atk_md0240_write_dat(0x11);
    atk_md0240_write_dat(0x24);
    atk_md0240_write_dat(0x2C);
    atk_md0240_write_dat(0x3F);
    atk_md0240_write_dat(0x44);
    atk_md0240_write_dat(0x51);
    atk_md0240_write_dat(0x2F);
    atk_md0240_write_dat(0x1F);
    atk_md0240_write_dat(0x1F);
    atk_md0240_write_dat(0x20);
    atk_md0240_write_dat(0x23);
    /* Display Inversion On */
    atk_md0240_write_cmd(0x21);
    atk_md0240_write_cmd(0x29);
}

/**
 * @brief       设置ATK-MD0240模块行列地址
 * @param       xs: 列起始地址
 *              ys: 行起始地址
 *              xe: 列结束地址
 *              ye: 行结束地址
 * @retval      无
 */
void atk_md0240_set_address(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)
{
    atk_md0240_write_cmd(0x2A);
    atk_md0240_write_dat((uint8_t)(xs >> 8) & 0xFF);
    atk_md0240_write_dat((uint8_t)xs & 0xFF);
    atk_md0240_write_dat((uint8_t)(xe >> 8) & 0xFF);
    atk_md0240_write_dat((uint8_t)xe & 0xFF);
    atk_md0240_write_cmd(0x2B);
    atk_md0240_write_dat((uint8_t)(ys >> 8) & 0xFF);
    atk_md0240_write_dat((uint8_t)ys & 0xFF);
    atk_md0240_write_dat((uint8_t)(ye >> 8) & 0xFF);
    atk_md0240_write_dat((uint8_t)ye & 0xFF);
    atk_md0240_write_cmd(0x2C);
}

/**
 * @brief       ATK-MD0240模块初始化
 * @param       无
 * @retval      无
 */
void atk_md0240_init(void)
{
    atk_md0240_hw_init();
    atk_md0240_hw_reset();
    atk_md0240_spi_init();
    atk_md0240_reg_init();
    atk_md0240_set_address(0, 0, ATK_MD0240_LCD_WIDTH - 1, ATK_MD0240_LCD_HEIGHT - 1);
    atk_md0240_display_on();
    atk_md0240_clear(ATK_MD0240_WHITE);
}

/**
 * @brief       开启ATK-MD0240模块LCD背光
 * @param       无
 * @retval      无
 */
void atk_md0240_display_on(void)
{
    ATK_MD0240_PWR(1);
}

/**
 * @brief       关闭ATK-MD0240模块LCD背光
 * @param       无 
 * @retval      无
 */
void atk_md0240_display_off(void)
{
    ATK_MD0240_PWR(0);
}

/**
 * @brief       ATK-MD0240模块LCD区域填充
 * @param       xs   : 区域起始X坐标
 *              ys   : 区域起始Y坐标
 *              xe   : 区域终止X坐标
 *              ye   : 区域终止Y坐标
 *              color: 区域填充颜色
 * @retval      无
 */
void atk_md0240_fill(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color)
{
    uint32_t area_size;
    uint32_t area_remain = 0;
    uint16_t buf_index;
    
    area_size = (xe - xs + 1) * (ye - ys + 1) * sizeof(uint16_t);
    if (area_size > ATK_MD0240_LCD_BUF_SIZE)
    {
        area_remain = area_size - ATK_MD0240_LCD_BUF_SIZE;
        area_size = ATK_MD0240_LCD_BUF_SIZE;
    }
    
    atk_md0240_set_address(xs, ys, xe, ye);
    ATK_MD0240_WR(1);
    while (1)
    {
        for (buf_index=0; buf_index<area_size / sizeof(uint16_t); buf_index++)
        {
            g_atk_md0240_lcd_buf[buf_index * sizeof(uint16_t)] = (uint8_t)(color >> 8) & 0xFF;
            g_atk_md0240_lcd_buf[buf_index * sizeof(uint16_t) + 1] = (uint8_t)color & 0xFF;
        }
        
        atk_md0240_spi_send(g_atk_md0240_lcd_buf, area_size);
        
        if (area_remain == 0)
        {
            break;
        }
        
        if (area_remain > ATK_MD0240_LCD_BUF_SIZE)
        {
            area_remain = area_remain - ATK_MD0240_LCD_BUF_SIZE;
        }
        else
        {
            area_size = area_remain;
            area_remain = 0;
        }
    }
}

/**
 * @brief       ATK-MD0240模块LCD清屏
 * @param       color: 清屏颜色
 * @retval      无
 */
void atk_md0240_clear(uint16_t color)
{
    atk_md0240_fill(0, 0, ATK_MD0240_LCD_WIDTH - 1, ATK_MD0240_LCD_HEIGHT - 1, color);
}

/**
 * @brief       ATK-MD0240模块LCD画点
 * @param       x    : 待画点的X坐标
 *              y    : 待画点的Y坐标
 *              color: 待画点的颜色
 * @retval      无
 */
void atk_md0240_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    atk_md0240_set_address(x, y, x, y);
    atk_md0240_write_dat_16b(color);
}

/**
 * @brief       ATK-MD0240模块LCD画线段
 * @param       x1   : 待画线段端点1的X坐标
 *              y1   : 待画线段端点1的Y坐标
 *              x2   : 待画线段端点2的X坐标
 *              y2   : 待画线段端点2的Y坐标
 *              color: 待画线段的颜色
 * @retval      无
 */
void atk_md0240_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t x_delta;
    uint16_t y_delta;
    int16_t x_sign;
    int16_t y_sign;
    int16_t error;
    int16_t error2;
    
    x_delta = (x1 < x2) ? (x2 - x1) : (x1 - x2);
    y_delta = (y1 < y2) ? (y2 - y1) : (y1 - y2);
    x_sign = (x1 < x2) ? 1 : -1;
    y_sign = (y1 < y2) ? 1 : -1;
    error = x_delta - y_delta;
    
    atk_md0240_draw_point(x2, y2, color);
    
    while ((x1 != x2) || (y1 != y2))
    {
        atk_md0240_draw_point(x1, y1, color);
        
        error2 = error << 1;
        if (error2 > -y_delta)
        {
            error -= y_delta;
            x1 += x_sign;
        }
        
        if (error2 < x_delta)
        {
            error += x_delta;
            y1 += y_sign;
        }
    }
}

/**
 * @brief       ATK-MD0240模块LCD画矩形框
 * @param       x1   : 待画矩形框端点1的X坐标
 *              y1   : 待画矩形框端点1的Y坐标
 *              x2   : 待画矩形框端点2的X坐标
 *              y2   : 待画矩形框端点2的Y坐标
 *              color: 待画矩形框的颜色
 * @retval      无
 */
void atk_md0240_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    atk_md0240_draw_line(x1, y1, x2, y1, color);
    atk_md0240_draw_line(x1, y2, x2, y2, color);
    atk_md0240_draw_line(x1, y1, x1, y2, color);
    atk_md0240_draw_line(x2, y1, x2, y2, color);
}

/**
 * @brief       ATK-MD0240模块LCD画圆形框
 * @param       x    : 待画圆形框原点的X坐标
 *              y    : 待画圆形框原点的Y坐标
 *              r    : 待画圆形框的半径
 *              color: 待画圆形框的颜色
 * @retval      无
 */
void atk_md0240_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    int32_t x_t;
    int32_t y_t;
    int32_t error;
    int32_t error2;
    
    x_t = -r;
    y_t = 0;
    error = 2 - 2 * r;
    
    do {
        atk_md0240_draw_point(x - x_t, y + y_t, color);
        atk_md0240_draw_point(x + x_t, y + y_t, color);
        atk_md0240_draw_point(x + x_t, y - y_t, color);
        atk_md0240_draw_point(x - x_t, y - y_t, color);
        
        error2 = error;
        if (error2 <= y_t)
        {
            y_t++;
            error = error + (y_t * 2 + 1);
            if ((-x_t == y_t) && (error2 <= x_t))
            {
                error2 = 0;
            }
        }
        
        if (error2 > x_t)
        {
            x_t++;
            error = error + (x_t * 2 + 1);
        }
    } while (x_t <= 0);
}

/**
 * @brief       ATK-MD0240模块LCD显示1个字符
 * @param       x    : 待显示字符的X坐标
 *              y    : 待显示字符的Y坐标
 *              ch   : 待显示字符
 *              font : 待显示字符的字体
 *              color: 待显示字符的颜色
 * @retval      无
 */
void atk_md0240_show_char(uint16_t x, uint16_t y, char ch, atk_md0240_lcd_font_t font, uint16_t color)
{
    const uint8_t *ch_code;
    uint8_t ch_width;
    uint8_t ch_height;
    uint8_t ch_size;
    uint8_t ch_offset;
    uint8_t byte_index;
    uint8_t byte_code;
    uint8_t bit_index;
    uint8_t width_index = 0;
    uint8_t height_index = 0;
    
    ch_offset = ch - ' ';
    
    switch (font)
    {
#if (ATK_MD0240_FONT_12 != 0)
        case ATK_MD0240_LCD_FONT_12:
        {
            ch_code = atk_md0240_font_1206[ch_offset];
            ch_width = ATK_MD0240_FONT_12_CHAR_WIDTH;
            ch_height = ATK_MD0240_FONT_12_CHAR_HEIGHT;
            ch_size = ATK_MD0240_FONT_12_CHAR_SIZE;
            break;
        }
#endif
#if (ATK_MD0240_FONT_16 != 0)
        case ATK_MD0240_LCD_FONT_16:
        {
            ch_code = atk_md0240_font_1608[ch_offset];
            ch_width = ATK_MD0240_FONT_16_CHAR_WIDTH;
            ch_height = ATK_MD0240_FONT_16_CHAR_HEIGHT;
            ch_size = ATK_MD0240_FONT_16_CHAR_SIZE;
            break;
        }
#endif
#if (ATK_MD0240_FONT_24 != 0)
        case ATK_MD0240_LCD_FONT_24:
        {
            ch_code = atk_md0240_font_2412[ch_offset];
            ch_width = ATK_MD0240_FONT_24_CHAR_WIDTH;
            ch_height = ATK_MD0240_FONT_24_CHAR_HEIGHT;
            ch_size = ATK_MD0240_FONT_24_CHAR_SIZE;
            break;
        }
#endif
#if (ATK_MD0240_FONT_32 != 0)
        case ATK_MD0240_LCD_FONT_32:
        {
            ch_code = atk_md0240_font_3216[ch_offset];
            ch_width = ATK_MD0240_FONT_32_CHAR_WIDTH;
            ch_height = ATK_MD0240_FONT_32_CHAR_HEIGHT;
            ch_size = ATK_MD0240_FONT_32_CHAR_SIZE;
            break;
        }
#endif
        default:
        {
            return;
        }
    }
    
    if ((x + ch_width > ATK_MD0240_LCD_WIDTH) || (y + ch_height > ATK_MD0240_LCD_HEIGHT))
    {
        return;
    }
    
    for (byte_index=0; byte_index<ch_size; byte_index++)
    {
        byte_code = ch_code[byte_index];
        for (bit_index=0; bit_index<8; bit_index++)
        {
            if ((byte_code & 0x80) != 0)
            {
                atk_md0240_draw_point(x + width_index, y + height_index, color);
            }
            height_index++;
            if (height_index == ch_height)
            {
                height_index = 0;
                width_index++;
                break;
            }
            byte_code <<= 1;
        }
    }
}

/**
 * @brief       ATK-MD0240模块LCD显示字符串
 * @note        会自动换行和换页
 * @param       x    : 待显示字符串的X坐标
 *              y    : 待显示字符串的Y坐标
 *              str  : 待显示字符串
 *              font : 待显示字符串的字体
 *              color: 待显示字符串的颜色
 * @retval      无
 */
void atk_md0240_show_string(uint16_t x, uint16_t y, char *str, atk_md0240_lcd_font_t font, uint16_t color)
{
    uint8_t ch_width;
    uint8_t ch_height;
    
    switch (font)
    {
#if (ATK_MD0240_FONT_12 != 0)
        case ATK_MD0240_LCD_FONT_12:
        {
            ch_width = ATK_MD0240_FONT_12_CHAR_WIDTH;
            ch_height = ATK_MD0240_FONT_12_CHAR_HEIGHT;
            break;
        }
#endif
#if (ATK_MD0240_FONT_16 != 0)
        case ATK_MD0240_LCD_FONT_16:
        {
            ch_width = ATK_MD0240_FONT_16_CHAR_WIDTH;
            ch_height = ATK_MD0240_FONT_16_CHAR_HEIGHT;
            break;
        }
#endif
#if (ATK_MD0240_FONT_24 != 0)
        case ATK_MD0240_LCD_FONT_24:
        {
            ch_width = ATK_MD0240_FONT_24_CHAR_WIDTH;
            ch_height = ATK_MD0240_FONT_24_CHAR_HEIGHT;
            break;
        }
#endif
#if (ATK_MD0240_FONT_32 != 0)
        case ATK_MD0240_LCD_FONT_32:
        {
            ch_width = ATK_MD0240_FONT_32_CHAR_WIDTH;
            ch_height = ATK_MD0240_FONT_32_CHAR_HEIGHT;
            break;
        }
#endif
        default:
        {
            return;
        }
    }
    
    while ((*str >= ' ') && (*str <= '~'))
    {
        if (x + ch_width >= ATK_MD0240_LCD_WIDTH)
        {
            x = 0;
            y += ch_height;
        }
        
        if (y + ch_height >= ATK_MD0240_LCD_HEIGHT)
        {
            y = 0;
            x = 0;
        }
        
        atk_md0240_show_char(x, y, *str, font, color);
        
        x += ch_width;
        str++;
    }
}

/**
 * @brief       ATK-MD0240模块LCD显示数字，可控制显示高位0
 * @param       x    : 待显示数字的X坐标
 *              y    : 待显示数字的Y坐标
 *              num  : 待显示数字
 *              len  : 待显示数字的位数
 *              mode : ATK_MD0240_NUM_SHOW_NOZERO: 数字高位0不显示
 *                     ATK_MD0240_NUM_SHOW_ZERO  : 数字高位0显示
 *              font : 待显示数字的字体
 *              color: 待显示数字的颜色
 * @retval      无
 */
void atk_md0240_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, atk_md0240_num_mode_t mode, atk_md0240_lcd_font_t font, uint16_t color)
{
    uint8_t ch_width;
    uint8_t len_index;
    uint8_t num_index;
    uint8_t first_nozero = 0;
    char pad;
    
    switch (font)
    {
#if (ATK_MD0240_FONT_12 != 0)
        case ATK_MD0240_LCD_FONT_12:
        {
            ch_width = ATK_MD0240_FONT_12_CHAR_WIDTH;
            break;
        }
#endif
#if (ATK_MD0240_FONT_16 != 0)
        case ATK_MD0240_LCD_FONT_16:
        {
            ch_width = ATK_MD0240_FONT_16_CHAR_WIDTH;
            break;
        }
#endif
#if (ATK_MD0240_FONT_24 != 0)
        case ATK_MD0240_LCD_FONT_24:
        {
            ch_width = ATK_MD0240_FONT_24_CHAR_WIDTH;
            break;
        }
#endif
#if (ATK_MD0240_FONT_32 != 0)
        case ATK_MD0240_LCD_FONT_32:
        {
            ch_width = ATK_MD0240_FONT_32_CHAR_WIDTH;
            break;
        }
#endif
        default:
        {
            return;
        }
    }
    
    switch (mode)
    {
        case ATK_MD0240_NUM_SHOW_NOZERO:
        {
            pad = ' ';
            break;
        }
        case ATK_MD0240_NUM_SHOW_ZERO:
        {
            pad = '0';
            break;
        }
        default:
        {
            return;
        }
    }
    
    for (len_index=0; len_index<len; len_index++)
    {
        num_index = (num / atk_md0240_pow(10, len - len_index - 1)) % 10;
        if ((first_nozero == 0) && (len_index < (len - 1)))
        {
            if (num_index == 0)
            {
                atk_md0240_show_char(x + ch_width * len_index, y, pad, font, color);
                continue;
            }
            else
            {
                first_nozero = 1;
            }
        }
        
        atk_md0240_show_char(x + ch_width * len_index, y, num_index + '0', font, color);
    }
}

/**
 * @brief       ATK-MD0240模块LCD显示数字，不显示高位0
 * @param       x    : 待显示数字的X坐标
 *              y    : 待显示数字的Y坐标
 *              num  : 待显示数字
 *              len  : 待显示数字的位数
 *              font : 待显示数字的字体
 *              color: 待显示数字的颜色
 * @retval      无
 */
void atk_md0240_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, atk_md0240_lcd_font_t font, uint16_t color)
{
    atk_md0240_show_xnum(x, y, num, len, ATK_MD0240_NUM_SHOW_NOZERO, font, color);
}

/**
 * @brief       ATK-MD0240模块LCD图片
 * @note        图片取模方式: 水平扫描、RGB565、高位在前
 * @param       x     : 待显示图片的X坐标
 *              y     : 待显示图片的Y坐标
 *              width : 待显示图片的宽度
 *              height: 待显示图片的高度
 *              pic   : 待显示图片数组首地址
 * @retval      无
 */
void atk_md0240_show_pic(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t *pic)
{
    if ((x + width > ATK_MD0240_LCD_WIDTH) || (y + height > ATK_MD0240_LCD_HEIGHT))
    {
        return;
    }
    
    atk_md0240_set_address(x, y, x + width - 1, y + height - 1);
    ATK_MD0240_WR(1);
    atk_md0240_spi_send(pic, width * height * sizeof(uint16_t));
}
