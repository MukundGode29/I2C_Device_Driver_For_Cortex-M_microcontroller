#ifndef HAL_I2C_DRIVER_H
#define HAL_I2C_DRIVER_H

#include "stm32f446xx.h"
#include <stdint.h>

//*************************************************
//      
//              I2C Register Bit Defination
//
//**************************************************

//      Bit Defination of I2C CR1 Register

#define I2C_REG_CR1_POS                ((uint32_t) 1 << 11 )

#define I2C_REG_CR1_ACK                ( (uint32_t) 1 << 10 ) 
#define I2C_ACK_ENABLE                 1
#define I2C_ACK_DISABLE                0

#define I2C_REG_CR1_STOP_GEN           ((uint32_t) 1 << 9 )
#define I2C_REG_CR1_START_GEN          ((uint32_t) 1 << 8 )

#define I2C_REG_CR1_NOSTRETCH          ((uint32_t) 1 << 7 )
#define I2C_ENABLE_CLK_STRETCH         0
#define I2C_DISABLE_CLK_STRETCH        1

#define I2C_REG_CR1_ENABLE_I2C         ((uint32_t) 1 << 0 )

//         Bit Defination for I2C CR2 REgister

#define I2C_REG_CR2_BUF_INT_ENABLE      ((uint32_t) 1 << 10)
#define I2C_REG_CR2_EVT_INT_ENABLE      ((uint32_t) 1 << 9 )
#define I2C_REG_CR2_ERR_INT_ENABLE      ((uint32_t) 1 << 8 )

#define I2C_PERIPHERIAL_CLK_FREQ_2MHZ   ((uint32_t) 2 )
#define I2C_PERIPHERIAL_CLK_FREQ_3MHZ   ((uint32_t) 3 )
#define I2C_PERIPHERIAL_CLK_FREQ_4MHZ   ((uint32_t) 4 )
#define I2C_PERIPHERIAL_CLK_FREQ_5MHZ   ((uint32_t) 5 )
#define I2C_PERIPHERIAL_CLK_FREQ_6MHZ   ((uint32_t) 6 )
#define I2C_PERIPHERIAL_CLK_FREQ_7MHZ   ((uint32_t) 7 )
#define I2C_PERIPHERIAL_CLK_FREQ_8MHZ   ((uint32_t) 8 )
#define I2C_PERIPHERIAL_CLK_FREQ_9MHZ   ((uint32_t) 9 )
#define I2C_PERIPHERIAL_CLK_FREQ_10MHZ  ((uint32_t) 10 )
#define I2C_PERIPHERIAL_CLK_FREQ_11MHZ  ((uint32_t) 11 )
#define I2C_PERIPHERIAL_CLK_FREQ_12MHZ  ((uint32_t) 12 )
#define I2C_PERIPHERIAL_CLK_FREQ_13MHZ  ((uint32_t) 13 )
#define I2C_PERIPHERIAL_CLK_FREQ_14MHZ  ((uint32_t) 14 )
#define I2C_PERIPHERIAL_CLK_FREQ_15MHZ  ((uint32_t) 15 )
#define I2C_PERIPHERIAL_CLK_FREQ_16MHZ  ((uint32_t) 16 )

//           Bit Defination of I2C register OAR_1

#define I2C_REG_OAR1_ADDRMODE           ((uint32_t) 1 << 15)
#define I2C_ADDRMODE_7BIT               0
#define I2C_ADDRMODE_10BIT              1

#define I2C_REG_OAR1_14BIT              ((uint32_t) 1 << 14)
#define I2C_REG_OAR_7BIT_ADDRESS_POS    1



//         Bit Defination of I2C SR1 register

#define I2C_REG_SR1_TIMEOUT_FLAG       ((uint32_t)  1 << 14 )
#define I2C_REG_SR1_OVR_FLAG           ((uint32_t)  1 << 11 )
#define I2C_REG_SR1_AF_FAILURE_FLAG    ((uint32_t)  1 << 10 )
#define I2C_REG_SR1_ARLO_FLAG          ((uint32_t)  1 << 9  )
#define I2C_REG_SR1_BUS_ERROR_FLAG     ((uint32_t)  1 << 8  )
#define I2C_REG_SR1_TXE_FLAG           ((uint32_t)  1 << 7  )
#define I2C_REG_SR1_RXNE_FLAG          ((uint32_t)  1 << 6  )
#define I2C_REG_SR1_STOP_DETECTION_FLAG ((uint32_t) 1 << 4  )  //for Slave
#define I2C_REG_SR1_BTF_FLAG           ((uint32_t)  1 << 2  )
#define I2C_REG_SR1_ADDR_FLAG          ((uint32_t)  1 << 1  )
#define I2C_REG_SR1_ADDR_SENT_FLAG     ((uint32_t)  1 << 1  )  //for master
#define I2C_REG_SR1_ADDR_MATCHED_FLAG  ((uint32_t)  1 << 1  )  //for slave
#define I2C_REG_SR1_SB_FLAG            ((uint32_t)  1 << 0  )

//                Bit Defination for I2C SR2 Register

#define I2C_REG_SR2_BUS_BUSY_FLAG      ((uint32_t)  1 << 1)
#define I2C_BUS_IS_BUSY                1
#define I2C_BUS_IS_FREE                0


#define I2C_REG_SR2_MSL_FLAG          ((uint32_t)  1 << 0)
#define I2C_MASTER_MODE               1
#define I2C_SLAVE_MODE                0

#define I2C_REG_SR2_TRA_FLAG          ((uint32_t)  1 << 2)
#define I2C_RX_MODE                   1
#define I2C_TX_MODE                   0


//                Bit defination for I2C CCR Register

#define I2C_REG_CCR_ENABLE_FM        ((uint32_t)  1 << 15 )
#define I2C_ENABLE_SM                0
#define I2C_ENABLE_FM                1

#define I2C_REG_CCR_DUTY             ((uint32_t)  1 << 14 )
#define I2C_FM_DUTY_16BY9            1
#define I2C_FM_DUTY_2                0


//            I2C Peripherial Base Address

#define I2C_1  I2C1
#define I2C_2  I2C2
#define I2C_3  I2C3

// Macros to enable clock for differnet Perherial

#define _HAL_RCC_I2C1_CLK_ENABLE()     (RCC->APB1ENR |= (1 << 21) )
#define _HAL_RCC_I2C2_CLK_ENABLE()     (RCC->APB1ENR |= (1 << 22) )
#define _HAL_RCC_I2C3_CLK_ENABLE()     (RCC->APB1ENR |= (1 << 23) )


/***************************************************************************/
//
//                  Data Structures OF I2C
//
/****************************************************************/

typedef enum
{
	HAL_I2C_STATE_RESET           = 0x00,
	HAL_I2C_STATE_READY           = 0x01,
	HAL_I2C_STATE_BUSY            = 0x02,
	HAL_I2C_STATE_BUSY_TX         = 0x03,
	HAL_I2C_STATE_BUSY_RX         = 0x04,
	HAL_I2C_STATE_ERROR           = 0x05

}hal_i2c_state_t;

/**
  * @brief   I2C Error Code
  */

#define HAL_I2C_ERROR_NONE     ((uint32_t) 0x00000000)
#define HAL_I2C_ERROR_BERR     ((uint32_t) 0x00000001)
#define HAL_I2C_ERROR_ARLO     ((uint32_t) 0x00000002)
#define HAL_I2C_ERROR_AF       ((uint32_t) 0x00000004)
#define HAL_I2C_ERROR_OVR      ((uint32_t) 0x00000008)
#define HAL_I2C_ERROR_TIMEOUT  ((uint32_t) 0x00000010)

/**
  * @brief   SPI Confriguration Structure Defination 
  */

typedef struct
{
	uint32_t ClockSpeed;  //Specifies Clock Speed
	
	uint32_t DutyCycle;  // Specifies the I2C  Fast mode duty cycle
	
	uint32_t OwnAddress1;  // Specifies the first device own address
	
	uint32_t AddressingMode;  //specifies if 7 bit or 10 bit addressing mode is selected
	
	uint32_t OwnAddress2;  //Specifies the secound own addressing mode
	
	uint32_t DualAddressing; // Specifies if dual addressing mode is set
	
	uint32_t GeneralCallMode; //Specifies if general mode is selected
	
	uint32_t NoStretchMode; // Specifies if no stretch mode is selected
	
	uint32_t ack_enable; // handles ACK enable/disable
	
}i2c_init_t;


// I2c handle Structure defination

typedef struct
{
	I2C_TypeDef  *Instance;  //i2c registers base address
	
	i2c_init_t Init;  //i2c communication parameter
	
	uint8_t   *pBuffer;  // Pointer to i2c Buffer
	
	uint32_t  XferSize;  // i2c transfer size
	
	uint32_t  XferCount; //i2c transfer count
	
	hal_i2c_state_t  State;  //i2C Communication State
	
	uint32_t ErrorCode;   //use to hold the errorcode status

}i2c_handle_t;

#define RESET  0
#define SET !RESET

/****************************************************************/
//
//           Driver Exposed APIs
//
/******************************************************************/

void hal_i2c_init(i2c_handle_t *handle);

void hal_i2c_master_tx(i2c_handle_t *handle, uint8_t slave_address, uint8_t *buffer, uint32_t len);

void hal_i2c_master_rx(i2c_handle_t *handle, uint8_t slave_address, uint8_t *buffer, uint32_t len);

void hal_i2c_slave_tx(i2c_handle_t *handle, uint8_t *buffer, uint32_t len);

void hal_i2c_slave_rx(i2c_handle_t *handle, uint8_t *buffer, uint32_t len);

void hal_i2c_handle_error_interrupt(i2c_handle_t *hi2c);

void hal_i2c_handle_event_interrupt(i2c_handle_t *hi2c);

void hal_i2c_error_cb(i2c_handle_t *i2cHandle);

//    Static Helper Function

static void hal_i2c_enable_peripherial(I2C_TypeDef *i2cx);

static void hal_i2c_disable_peripheral(I2C_TypeDef *i2cx);

static void hal_i2c_manage_clock_stretch(I2C_TypeDef *i2cx, uint32_t no_stretch);

static void hal_i2c_set_own_address(I2C_TypeDef *i2cx, uint32_t own_address);

static void hal_i2c_set_addressing_mode(I2C_TypeDef *i2cx, uint32_t adr_mode);

static void hal_i2c_set_fm_duty_cycle(I2C_TypeDef *i2cx, uint32_t duty_cycle);

static void hal_i2c_clk_init(I2C_TypeDef *i2cx, uint32_t clk_speed, uint32_t duty_cycle);

static void hal_i2c_generate_start_condition(I2C_TypeDef *i2cx);

static void hal_i2c_generate_stop_condition(I2C_TypeDef *i2cx);

static void hal_i2c_configure_buffer_interrupt(I2C_TypeDef *i2cx, uint32_t enable);

static void hal_i2c_configure_error_interrupt(I2C_TypeDef *i2cx, uint32_t enable);

static void hal_i2c_configure_event_interrupt(I2C_TypeDef *i2cx, uint32_t enable);

static uint8_t is_bus_busy(I2C_TypeDef *i2cx);

static void i2c_wait_until_sb_set(I2C_TypeDef *i2cx);

static void i2c_wait_until_addr_set(I2C_TypeDef *i2cx);

static void hal_i2c_send_addr_first(I2C_TypeDef *i2cx, uint32_t slave_add);

static void clear_addr_flag(I2C_TypeDef *i2cx);

static void clear_stop_flag(i2c_handle_t *hi2c);

static void hal_i2c_master_handle_TXE_interrupt(i2c_handle_t *hi2c);

static void hal_i2c_master_tx_complt(i2c_handle_t *hi2c);

static void hal_i2c_master_handle_btf(i2c_handle_t *hi2c);

static void hal_i2c_slave_tx_handle_btf(i2c_handle_t *hi2c);

static void hal_i2c_slave_handle_stop_condition(i2c_handle_t *hi2c);

static void  hal_i2c_slave_rx_handle_btf(i2c_handle_t *hi2c);

static void hal_i2c_slave_handle_RXNE_interrupt(i2c_handle_t *hi2c);


#endif