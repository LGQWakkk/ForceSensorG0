#pragma once

#include "spi.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// WS2812 电源电压: 3.7V-5.3V
// 每个灯数据长度24bit[G7-G0 R7-R0 B7-B0] 高位先发送 按照GRB顺序发送

// WS2812 单总线传输特性
// 0码: 高电平时间220ns-380ns 低电平时间580ns-1us
// 1码: 高电平时间580ns-1us   低电平时间220ns-420ns
// 复位码: 低电平时间最少为280us

// T0H: 300ns T0L: 800ns
// T1H: 800ns T1L: 300ns
// 所以要求高低比例大致为3:8即可
// 若使用8bit数据 若周期设置为1200ns 单位150ns
// 高电平使用2bit=300ns 低电平使用6bit=900ns 这个时间是符合要求的

// 填充u8传输单元buffer
// SPI发送大小为8bit 8bit表示WS2812的一个bit
// 单个bit周期为150ns 即SCK频率为6.66MHz
// 周期1200ns 即8bit对应1200ns
// uint8_t one_code  = 0xFC;// 11111100
// uint8_t zero_code = 0xC0;// 11000000

// 实际SCK频率为6.625MHz 一个bit周期为150.943ns
// 8bit周期为1.20755us
// 复位信号需要至少50us 需要至少42个 8bit周期 可以直接使用48bit来作为复位信号

#define R 1
#define G 0
#define B 2

// 将SPI2 MOSI 连接到WS2812 DIN
#define WS2812_SPI_HANDLE hspi2

#define WS2812_LED_NUM 8
#define WS2812_BUFFERSIZE_PERLED 24 // Bytes
#define WS2812_BUFFERSIZE_RESET  48 // Bytes
// 240字节
#define WS2812_BUFFERSIZE (WS2812_LED_NUM * WS2812_BUFFERSIZE_PERLED + WS2812_BUFFERSIZE_RESET)

#define ONE_CODE  0xFC // 11111100
#define ZERO_CODE 0xC0 // 11000000

void ws2812_update_buffer(uint8_t (*led_color)[3]);
void ws2812_update_one_buffer(uint8_t* rgb, uint8_t* buffer);
void ws2812_init_buffer(void);
void ws2812_spi_dma_start(void);
