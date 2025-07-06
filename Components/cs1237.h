#pragma once

// 20250628 Wakkk Update

#include "main.h"
#include "gpio.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// 切换SDA引脚方向
#define SDA_IN() cs1237_sda_in()
#define SDA_OUT() cs1237_sda_out()

// 读取SDA引脚电平
#define SDA_READ() HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin)
// 写入SDA引脚电平
#define SDA_HIGH() HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET)
#define SDA_LOW() HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET)
// 写入SCK引脚电平
#define SCK_HIGH() HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET)
#define SCK_LOW() HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET)

void cs1237_sda_in(void);
void cs1237_sda_out(void);

uint32_t cs1237_read_adc(void);
uint8_t cs1237_read_config(void);
void cs1237_set_config(uint8_t registertowrite, uint8_t valuetowrite);
void cs1237_init(void);
void cs1237_delay_scl(void);
