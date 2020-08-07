
#include <stm32f446xx.h>
#include <stdint.h>
#include "hal_i2c_driver.h"
#include "gpio_driver.h"
#include "Board_LED.h"
#include "Board_Buttons.h"

#define I2C1_SCL_LINE  6
#define I2C1_SDA_LINE  9

#define GPIO_BUTTON_PIN  13
#define GPIO_BUTTON_PORT  GPIOC

#define ALT_FUN_4             0x04
#define GPIO_PIN_AF4_I2C123   ALT_FUN_4

#define SLAVE_ADDRESS  (uint8_t)01

#define I2Cx_ER_IRQn  32
#define I2Cx_EV_IRQn  33

#define EXTIx_IRQn            EXTI0_IRQn
#define EXTIx_IRQHandler      EXTI0_Handler

#define I2C_MASTER_MODE_EN

//bufer to hold Master TX and RX
uint8_t master_tx_buffer[5] = {0xa5, 0x55, 0xa5, 0x55, 0xa5};
uint8_t master_rx_buffer[5];

uint8_t slave_tx_buffer[5];
uint8_t slave_rx_buffer[5];

uint8_t master_write_req;
uint8_t master_read_req;

uint8_t WRITE_LEN;
uint8_t MASTER_WRITE_CMD;

uint8_t READ_LEN;
uint8_t MASTER_READ_CMD;

uint8_t SLAVE_ADDRESS_WRITE;
uint8_t SLAVE_ADDRESS_READ;


uint8_t slave_rcv_cmd;
uint8_t slave_data_len;

//handle variable for i2c device
i2c_handle_t i2c_handle;

uint8_t TestReady = 0;

void i2c_gpio_init(void);

void EXTIx_IRQHandler(void);

void i2cx_ER_IRQHandler(i2c_handle_t *hi2cx);

void i2cx_EV_IRQHandler(i2c_handle_t *hi2cx);

uint8_t Buffercmp(uint8_t buff1[], uint8_t buff2[], uint8_t len);

void delay_gen();