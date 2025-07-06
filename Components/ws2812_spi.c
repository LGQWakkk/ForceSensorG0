#include "ws2812_spi.h"
// 20250628 Wakkk

uint8_t ws2812_buffer[WS2812_BUFFERSIZE]; // SPI DMA 发送缓存

// 更新所有LED的buffer
void ws2812_update_buffer(uint8_t (*led_color)[3])
{
    for(uint8_t led=0; led<WS2812_LED_NUM; led++){
        uint16_t offset = WS2812_BUFFERSIZE_RESET + led * WS2812_BUFFERSIZE_PERLED;
        ws2812_update_one_buffer(led_color[led], &ws2812_buffer[offset]);
    }
}

// 更新一个LED的buffer buffer操作长度为24字节
// rgb为三元素数组指针 顺序为GRB
void ws2812_update_one_buffer(uint8_t* rgb, uint8_t* buffer)
{
    for(uint8_t color=0; color<3; color++){
        for(uint8_t bit=0; bit<8; bit++){
            if((rgb[color] << bit) & 0x80){ //bit1
                buffer[color*8+bit] = ONE_CODE;
            }else{ // bit0
                buffer[color*8+bit] = ZERO_CODE;
            }
        }
    }
}

// 初始化颜色缓存
void ws2812_init_buffer(void)
{
    for(uint16_t i=0; i<WS2812_BUFFERSIZE; i++){
        ws2812_buffer[i] = 0x00;
    }
}

// 启动SPI DMA 循环发送
void ws2812_spi_dma_start(void)
{
    HAL_SPI_Transmit_DMA(&WS2812_SPI_HANDLE, ws2812_buffer, WS2812_BUFFERSIZE);
}
