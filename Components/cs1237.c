#include "cs1237.h"
#include "main.h"
#include "gpio.h"

// 20250628 Wakkk Update

#define CS1237_SCK_DELAY 0x08 // 20250628 @STM32G030F6P6 53MHZ
void cs1237_delay_scl(void)
{
    for(uint16_t i=0;i<CS1237_SCK_DELAY;i++); // 根据MCU和时钟频率调整延时
    // CS1237要求SCK高电平或者低电平脉宽不得小于455ns
}

// SDA引脚切换为输入模式
void cs1237_sda_in(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
}

// SDA引脚切换为输出模式
void cs1237_sda_out(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
}

// 读取原始数据 24位二进制补码
// Data acquisition function - Returns a uint32_t variable
uint32_t cs1237_read_adc(void)
{
    while (SDA_READ()); //Wait for the DRDY/DOUT to fall LOW
    //DRDY was low, so we can proceed further
    uint32_t result = 0; //24-bit output data is stored in this variable
    for (uint8_t i = 0; i < 24; i++) //Read the 24-bits
    {        
        SCK_HIGH();//上升沿
        cs1237_delay_scl();
        //i = 0; MSB @ bit 23
        //i = 1; MSB-1 @ bit 22
        //... i = 23; LSB @ bit 0 (not shifted, just OR'd together with the result)
        SCK_LOW();//下降沿数据读取
        uint32_t bit = SDA_READ(); //Read a bit
        result |= bit << (23 - i); //Read a bit and shift it up
    }
    //Shift bit 25-26-27 as well.
    for (uint8_t i = 0; i < 3; i++){
        SCK_HIGH();
        cs1237_delay_scl();
        SCK_LOW();
    }
    return result;
}

// 读取config寄存器数值
// Command: 0x56
uint8_t cs1237_read_config(void) //Reads all registers. The actual values will be fetched with other, simple functions
{  
    uint8_t registerValue; //Variable that stores the config register value
    //Shift out 27 (24+3) bits
    uint32_t adc = cs1237_read_adc(); //32-bit variable that stores the whole ADC reading
    SDA_OUT();  //After the 27th SCLK pulse, set DOUT to OUTPUT
    for (uint8_t i = 0; i < 2; i++) //Emit 2 pulses (28-29)
    {
        SCK_HIGH();
        cs1237_delay_scl();
        SCK_LOW();
    }
    for (uint8_t i = 0; i < 7; i++) //SCLK 30-36, sending READ word
    {
        SCK_HIGH();
        cs1237_delay_scl();
        if( ((0x56 >> (6 - i)) & 0x01) ){
            SDA_HIGH();
        }else{
            SDA_LOW();
        }
        SCK_LOW();
    }
    for (uint8_t i = 0; i < 1; i++) //Send the 37th SCLK pulse
    {
        SCK_HIGH();
        cs1237_delay_scl();
        SCK_LOW();
    }
    //After the 37th SCLK pulse switch the direction of DOUT. 
    SDA_IN();
    registerValue = 0; //Because we are reading
    for (uint8_t i = 0; i < 8; i++) //38-45 SCLK pulses
    {
        SCK_HIGH();
        cs1237_delay_scl();
        uint8_t bit = SDA_READ(); //Read a bit
        registerValue |= bit << (7 - i); //read out and shift the values into the register_value variable        
        SCK_LOW();
    }
    // send 1 clock pulse, to set the Pins of the ADCs to output and pull high
    for (uint8_t i = 0; i < 1; i++){
        SCK_HIGH();
        cs1237_delay_scl();
        SCK_LOW();
    }
    // At the 46th SCLK, switch DRDY / DOUT to output and pull up DRDY / DOUT. 
    // pinMode(DOUT_DRDY, INPUT_PULLUP); //Ready to receive the DRDY to perform a new acquisition
    return registerValue;
}

// 寄存器选项配置
#define CS1237_REG_CHANNEL  0
#define CS1237_REG_PGA      1
#define CS1237_REG_DRATE    2
#define CS1237_REG_VREF     3
// ADC采样通道设置
#define CS1237_CHANNEL_A        0
#define CS1237_CHANNEL_TEMP     2
#define CS1237_CHANNEL_SHORT    3
// 通道增益设置
#define CS1237_PGA_1            0
#define CS1237_PGA_2            1
#define CS1237_PGA_64           2
#define CS1237_PGA_128          3

// 写入config寄存器
void cs1237_set_config(uint8_t registertowrite, uint8_t valuetowrite)
{
    //"Arbitrary" register numbers
    //0 - Channel
    //1 - PGA
    //2 - Speed
    //3 - REF

    //Config register structure
    //bit 0-1 : Channel. 00 - A, 01 - reserved, 10 - Temperature, 11 - internal short (maybe for offset calibration?)
    //bit 2-3 : PGA. 00 - 1, 01 - 2, 10 - 64, 11 - 128
    //bit 4-5 : speed. 00 - 10 Hz, 01 - 40 Hz, 10 - 640 Hz, 11 - 1280 Hz
    //bit 6   : Reference. Default is enabled which is 0.
    //bit 7   : reserved, don't touch
    //----------------------------------------------------------
    
    uint8_t register_value = cs1237_read_config(); //Variable that stores the config register value. Fill it up with the current reg value
    uint8_t byteMask = 0x00; //Masking byte for writing only 1 register at a time
    switch (registertowrite)
    {
        case 0: //channel
        // 0b11111100 = 0xFC
        byteMask = 0xFC; //when using & operator, we keep all, except channel bits
        register_value = register_value & byteMask; //Update the register_value with mask. This deletes the first two
            switch (valuetowrite)
            {
            case 0: // A // W0 0
                register_value = register_value | 0x00; //Basically keep everything as-is
                // Serial.println("Channel = 0");
                break;
            case 1: // Reserved // W0 1
                //dont implement it!
                // Serial.println("Channel = Reserved, invalid!");
                break;
            case 2: //Temperature // W0 2
                register_value = register_value | 0x02;
                // Serial.println("Channel = Temp");
                break;
            case 3: //Internal short //W0 3
                register_value = register_value | 0x03;
                // Serial.println("Channel = Short");
                break;
            }      
        break;
        //-------------------------------------------------------------------------------------------------------------
        case 1: //PGA
            byteMask = 0xF3; //when using & operator, we keep all, except channel bits
            register_value = register_value & byteMask; //Update the register_value with mask. This deletes the first two

            switch (valuetowrite)
            {
            case 0: // PGA 1 //W1 0
                register_value = register_value | 0x00; //Basically keep everything as-is
                break;
            case 1: // PGA 2 //W1 1
                register_value = register_value | 0x04;
                break;
            case 2: //PGA 64 //W1 2
                register_value = register_value | 0x08;
                break;
            case 3: //PGA 128 //W1 3
                register_value = register_value | 0x0C;
                break;
            }
            break;
            //-------------------------------------------------------------------------------------------------------------
        case 2: //DRATE
            byteMask = 0xCF; //when using & operator, we keep all, except channel bits
            register_value = register_value & byteMask; //Update the register_value with mask. This deletes the first two

            switch (valuetowrite)
            {
            case 0: // 10 Hz //W2 0
                register_value = register_value | 0x00; //Basically keep everything as-is
                // Serial.println("DRATE = 10 Hz");
                break;
            case 1: // 40 Hz //W2 1
                register_value = register_value | 0x10;
                // Serial.println("DRATE = 40 Hz");
                break;
            case 2: //640 Hz //W2 2
                register_value = register_value | 0x20;
                // Serial.println("DRATE = 640 Hz");
                break;
            case 3: //1280 Hz //W2 3
                register_value = register_value | 0x30;
                // Serial.println("DRATE = 1280 Hz");
                break;
            }
            break;
            //-------------------------------------------------------------------------------------------------------------
        case 3: //VREF
            if (valuetowrite == 0) //W3 0
            {
                // bitWrite(register_value, 6, 0); //Enable
                register_value = register_value & 0xBF;//Reset Bit 6
                // Serial.println("VREF ON");
            }
            else if (valuetowrite == 1) //W3 1
            {
                // bitWrite(register_value, 6, 1); //Disable
                register_value = register_value | 0x40; //Set Bit 6
                // Serial.println("VREF OFF");
            }
            else {}//Other values wont trigger anything
            break;
    }
    //Shift out 27 (24+3) bits
    uint32_t adc = cs1237_read_adc(); //32-bit variable that stores the whole ADC reading
    SDA_OUT();
    for (uint8_t i = 0; i < 2; i++) //Emit 2 pulses (28-29)
    {
        SCK_HIGH();
        cs1237_delay_scl();
        SCK_LOW();
    }
    for (uint8_t i = 0; i < 7; i++) //SCLK 30-36, sending READ word
    {
        SCK_HIGH();
        if( ((0x65 >> (6 - i)) & 0x01) ){
            SDA_HIGH();//High 
        }else{
            SDA_LOW();//Low
        }
        cs1237_delay_scl();
        SCK_LOW();
    }
    for (uint8_t i = 0; i < 1; i++) //Send the 37th SCLK pulse
    {
        SCK_HIGH();
        cs1237_delay_scl();
        SCK_LOW();
    } 
    for (uint8_t i = 0; i < 8; i++) //38-45 SCLK pulses
    {
        SCK_HIGH();
        if( ((register_value >> (7 - i)) & 0x01) ){
            SDA_HIGH();//High
        }else{
            SDA_LOW();//Low
        }
        cs1237_delay_scl();
        SCK_LOW();
    }
    // send 1 clock pulse, to set the Pins of the ADCs to output and pull high
    for (uint8_t i = 0; i < 1; i++){
        SCK_HIGH();SCK_LOW();
    }
    // At the 46th SCLK, switch DRDY / DOUT to output and pull up DRDY / DOUT. 
    SDA_IN();
}

void cs1237_init(void)
{
    SDA_IN();  // 初始化SDA为输入模式
    SCK_LOW();  // 初始化SCK输出低电平退出休眠模式
    HAL_Delay(10);
}
