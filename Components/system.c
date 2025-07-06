// 20250423 ForceSensorG0
// 单轴无线力传感器

#include "gpio.h"
#include "system.h"
#include "cs1237.h"
#include "si24r1.h"
#include "ws2812_spi.h"

#define LED_ON()        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF()       HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)

#define BUTTON_UP()     HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)
#define BUTTON_DOWN()   (!BUTTON_UP())

uint8_t color[WS2812_LED_NUM][3]; // 颜色缓存
uint8_t si24r1_tx_buffer[32]; // SI24R1发送缓存

// 本文件包含系统整体实现
// 包含了LED和BUTTON的简易控制

// 使用UART2作为传输端口

void system_setup(void)
{
    printf("System Begin\r\n");
    HAL_Delay(100);

    printf("CS1237 ADC Init\r\n");
    cs1237_init();
    cs1237_set_config(2, 2);    // 设置频率 640Hz
    cs1237_set_config(0, 0);    // Channel A
    // cs1237_set_config(0, 3); // 内部短接测试   16771616 -5605
    cs1237_set_config(1, 3);    // PGA=128
    printf("CS1237 ADC Init Done\r\n");
    HAL_Delay(10);

    printf("SI24R1 Init\r\n");
    si24r1_check();
    si24r1_init();
    si24r1_set_power(5);
    si24r1_set_mode(MODE_TX);
    printf("SI24R1 Init Done\r\n");

    // ws2812_init_buffer();
    // // LED0 RED
    // color[0][R] = 255;
    // color[0][G] = 0;
    // color[0][B] = 0;
    // // LED1 GREEN
    // color[1][R] = 0;
    // color[1][G] = 255;
    // color[1][B] = 0;
    // // LED2 BLUE
    // color[2][R] = 0;
    // color[2][G] = 0;
    // color[2][B] = 255;
    // ws2812_update_buffer(color);
    // ws2812_spi_dma_start();
}

void system_loop(void)
{
    // si24r1_tx_32bytes_nack(si24r1_tx_buffer);
    si24r1_tx_payload_nack(si24r1_tx_buffer, 32);
    // HAL_Delay(1);
    // uint32_t adc_value_raw = cs1237_read_adc();     // 读取24位原始二进制补码数值
    // // 将24位二进制补码转换为int32_t
    // if(adc_value_raw & (uint32_t)0x800000){         //bit 24==1 负数
    //     adc_value_raw |= (uint32_t)0xFF000000;      //位拓展
    // }
    // int32_t adc_value = (int32_t)adc_value_raw;  // 转换为int32_t
    // printf("ADC Value: %d\r\n", adc_value);


    // LED_ON();
    // HAL_Delay(100);
    // LED_OFF();
    // HAL_Delay(900);

    // // LED0 RED
    // color[0][R] = 255;
    // color[0][G] = 0;
    // color[0][B] = 0;
    // // LED1 GREEN
    // color[1][R] = 0;
    // color[1][G] = 255;
    // color[1][B] = 0;
    // // LED2 BLUE
    // color[2][R] = 0;
    // color[2][G] = 0;
    // color[2][B] = 255;
    // ws2812_update_buffer(color);
    // HAL_Delay(100);

}
