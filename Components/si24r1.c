// 20250628 SI24R1 TX ONLY
#include "si24r1.h"
#include "system.h"

uint8_t spi_read_write_byte(uint8_t TxByte)
{
	uint8_t RxByte = 0x00;
	HAL_SPI_TransmitReceive(&SI24R1_SPI_HANDLE, &TxByte, &RxByte, 1, HAL_MAX_DELAY);
	return RxByte;
}

uint8_t spi_read_reg(uint8_t RegAddr)
{
    uint8_t btmp;
    SI24R1_CS_LOW();
    spi_read_write_byte( NRF_READ_REG | RegAddr );
    btmp = spi_read_write_byte( 0xFF );
    SI24R1_CS_HIGH();
    return btmp;
}

void spi_read_buf(uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
    uint8_t btmp;
    SI24R1_CS_LOW();
    spi_read_write_byte( NRF_READ_REG | RegAddr );
    for( btmp = 0; btmp < len; btmp ++ ){
        *( pBuf + btmp ) = spi_read_write_byte( 0xFF );
    }
    SI24R1_CS_HIGH();
}

void spi_write_reg(uint8_t RegAddr, uint8_t Value)
{
    SI24R1_CS_LOW();
    spi_read_write_byte( NRF_WRITE_REG | RegAddr );
    spi_read_write_byte( Value );
    SI24R1_CS_HIGH();
}

void spi_write_buf(uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    SI24R1_CS_LOW();
    spi_read_write_byte( NRF_WRITE_REG | RegAddr );
    for( i = 0; i < len; i ++ ){
        spi_read_write_byte(*( pBuf + i ));
    }
    SI24R1_CS_HIGH();
}

void si24r1_flush_tx_fifo (void)
{
    SI24R1_CS_LOW();
    spi_read_write_byte(FLUSH_TX);	//清TX FIFO命令
    SI24R1_CS_HIGH();
}

void si24r1_flush_rx_fifo(void)
{
	SI24R1_CS_LOW();
	spi_read_write_byte(FLUSH_RX);	//清RX FIFO命令
	SI24R1_CS_HIGH();
}

void si24r1_reuse_tx_payload(void)
{
	SI24R1_CS_LOW();
	spi_read_write_byte( REUSE_TX_PL );		//重新使用上一包命令
	SI24R1_CS_HIGH();
}

void si24r1_nop(void)
{
	SI24R1_CS_LOW();
	spi_read_write_byte( NOP );		//空操作命令
	SI24R1_CS_HIGH();
}

//读取状态寄存器
uint8_t si24r1_read_status_reg(void)
{
	uint8_t Status;
	SI24R1_CS_LOW();
	Status = spi_read_write_byte( NRF_READ_REG + STATUS );	//读状态寄存器
	SI24R1_CS_HIGH();
	return Status;
}

//清除中断状态
//IRQ_Source:要清除的中断源
//返回值为清除后的状态寄存器值
uint8_t si24r1_clear_irq_flag( uint8_t IRQ_Source )
{
	uint8_t btmp = 0;
	IRQ_Source &= ( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT );	//中断标志处理
	btmp = si24r1_read_status_reg();
	SI24R1_CS_LOW();	
	spi_read_write_byte( NRF_WRITE_REG + STATUS );	//写状态寄存器命令
	spi_read_write_byte( IRQ_Source | btmp );		//清相应中断标志
	SI24R1_CS_HIGH();	
	return ( si24r1_read_status_reg());			    //返回状态寄存器状态
}

//读取中断状态
uint8_t si24r1_read_irq_status( void )
{
  	return ( si24r1_read_status_reg() & (( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT )));	//返回中断状态
}
 
//读FIFO中数据宽度
//返回值为数据宽度
uint8_t si24r1_read_fifo_width( void )
{
	uint8_t btmp;
	SI24R1_CS_LOW();
	spi_read_write_byte( R_RX_PL_WID );	//读FIFO中数据宽度命令
	btmp = spi_read_write_byte( 0xFF );	//读数据
	SI24R1_CS_HIGH();
	return btmp;
}

//发送数据（带应答） 单次数据不超过32字节
void si24r1_tx_payload_ack( uint8_t *pTxBuf, uint8_t len )
{
    uint8_t btmp;
    uint8_t length = ( len > 32 ) ? 32 : len;		//数据长达大约32 则只发送32个
    si24r1_flush_tx_fifo();		//清TX FIFO
    SI24R1_CS_LOW();	
    spi_read_write_byte( WR_TX_PLOAD );	//发送命令
    for( btmp = 0; btmp < length; btmp ++ )
    {
        spi_read_write_byte( *( pTxBuf + btmp ) );	//发送数据
    }
    SI24R1_CS_HIGH();	
}

//发送数据（不带应答） 单次数据不超过32字节
//返回数值为发送完毕之后读取到的状态寄存器值  
//此函数可以达到1.548kHz 32Bytes数据包发送频率 但是接收不确保有此频率(STM32F103:72MHZ SPI:9MHZ)
uint8_t si24r1_tx_payload_nack(uint8_t *pTxBuf, uint8_t len)
{
    if(len > 32 || len == 0){
        return 0;//数据长度大于32 或者等于0 不执行
    }
    SI24R1_CS_LOW();
    spi_read_write_byte(WR_TX_PLOAD_NACK);	//发送命令
    while(len--){
        spi_read_write_byte(*pTxBuf);		//发送数据
        pTxBuf++;
    }
    SI24R1_CS_HIGH();
    // 等待TX_DS中断(应立即发生)
    while(SI24R1_GET_IRQ_STATUS());//此处约等待0.6-1.4ms左右
    uint8_t _status = spi_read_reg(STATUS);
    spi_write_reg(STATUS, _status);// 清除中断
    return _status;
}

//设置发送地址 不大于5字节
void si24r1_set_tx_addr(uint8_t *pAddr, uint8_t len)
{
	len = ( len > 5 ) ? 5 : len;					//地址不能大于5个字节
	spi_write_buf( TX_ADDR, pAddr, len );	//写地址
}

//设置接收通道地址
//PipeNum:通道 0-5
//pAddr:地址存放地址
//Len:地址长度 不大于5字节
void si24r1_set_rx_addr(uint8_t PipeNum, uint8_t *pAddr, uint8_t Len)
{
	Len = ( Len > 5 ) ? 5 : Len;
	PipeNum = ( PipeNum > 5 ) ? 5 : PipeNum;		//通道不大于5 地址长度不大于5个字节
	spi_write_buf( RX_ADDR_P0 + PipeNum, pAddr, Len );	//写入地址
}

void si24r1_set_speed(si24r1_speed_type Speed)
{
	uint8_t btmp = 0;
	btmp = spi_read_reg( RF_SETUP );
	btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	if( Speed == SPEED_250K ){
		btmp |= ( 1<<5 );
	}
	else if( Speed == SPEED_1M ){
   	btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	}
	else if( Speed == SPEED_2M ){
		btmp |= ( 1<<3 );
	}
	spi_write_reg( RF_SETUP, btmp );
}

// 设置功率
// 因为不同芯片具体数值对应功率不同 这里仅仅是对功率等级进行设置
// 功率等级范围0-7 占寄存器低三位
// 注意不同芯片可能有些值不支持设置 请查阅数据手册
void si24r1_set_power(uint8_t power_level)
{
	if(power_level > 7)power_level = 7;
	uint8_t btmp;
	btmp = spi_read_reg( RF_SETUP ) & ~0x07;
	btmp |= power_level;  
	spi_write_reg( RF_SETUP, btmp );
}

void si24r1_set_channel(uint8_t FreqPoint)
{
  	spi_write_reg(RF_CH, FreqPoint & 0x7F);
}

//检测和SI24R1的硬件通信连接是否正常
void si24r1_check(void)
{
	si24r1_debug("si24r1: check\r\n");
	uint8_t i;
	uint8_t buf[5]={ 0XA5, 0XA5, 0XA5, 0XA5, 0XA5 };
	uint8_t read_buf[5] = {0};
	while(1)
	{
		spi_write_buf(TX_ADDR, buf, 5);			//写入5个字节的地址
		spi_read_buf(TX_ADDR, read_buf, 5);		//读出写入的地址  
		for(i = 0; i < 5; i++){
			si24r1_debug("buf[%d] = %d, read_buf[%d] = %d\r\n", i, buf[i], i, read_buf[i]);
			if(buf[ i ] != read_buf[ i ]){
				break;
			}
		}
		if(5 == i){
			break;
		}
		else{
			si24r1_debug("si24r1: check error!\r\n");
		}
		HAL_Delay(200);
		si24r1_debug("si24r1: check again!\r\n");
	}
	si24r1_debug("si24r1: check pass\r\n");
}

//设置发送/接收模式
void si24r1_set_mode(si24r1_mode_t Mode)
{
	uint8_t controlreg = 0;
	controlreg = spi_read_reg(CONFIG);
	if( Mode == MODE_TX ){
		controlreg &= ~( 1<< PRIM_RX );
	}
	else{
		if( Mode == MODE_RX ){ 
			controlreg |= ( 1<< PRIM_RX ); 
		}
	}
	spi_write_reg(CONFIG, controlreg);
}

void si24r1_init(void)
{
    SI24R1_CS_HIGH();
    uint8_t addr[5] = {DEFAULT_TX_ADDR};//设置为默认发送地址20250309
    SI24R1_CE_HIGH();
    si24r1_clear_irq_flag(IRQ_ALL);
    spi_write_reg( DYNPD, ( 1 << 0 ) );//使能通道1动态数据长度
    spi_write_reg( FEATRUE, 0x07 );
    spi_read_reg( DYNPD );
    spi_read_reg( FEATRUE );
    spi_write_reg( CONFIG, /*( 1<<MASK_RX_DR ) |*/    //接收中断
                                    ( 1 << EN_CRC ) | //使能CRC 1个字节
                                    ( 1 << PWR_UP ) );//开启设备
    spi_write_reg( EN_AA, ( 1 << ENAA_P0 ) );       //通道0自动应答
    spi_write_reg( EN_RXADDR, ( 1 << ERX_P0 ) );    //通道0接收
    spi_write_reg( SETUP_AW, AW_5BYTES );           //地址宽度 5个字节
    spi_write_reg( SETUP_RETR, ARD_4000US |
                        ( REPEAT_CNT & 0x0F ) );    //重复等待时间 250us
    spi_write_reg( RF_CH, 60 );                     //初始化通道
    spi_write_reg( RF_SETUP, 0x26 );
    si24r1_set_tx_addr( &addr[0], 5 );              //设置TX地址
    si24r1_set_rx_addr( 0, &addr[0], 5 );           //设置RX地址
}
