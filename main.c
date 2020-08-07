#include "main.h"

int main(void)
{

	
	LED_Initialize();
	
	i2c_gpio_init();
	
	#ifdef I2C_MASTER_MODE
	Buttons_Initialize();
	#endif
	
	_HAL_RCC_I2C1_CLK_ENABLE();
	
	
	i2c_handle.Instance = I2C_1;
	i2c_handle.Init.ack_enable = I2C_ACK_ENABLE;
	i2c_handle.Init.AddressingMode = I2C_ADDRMODE_7BIT;
	i2c_handle.Init.ClockSpeed = 400000;
	i2c_handle.Init.DutyCycle = I2C_FM_DUTY_2;
	i2c_handle.Init.GeneralCallMode = 0;
	i2c_handle.Init.NoStretchMode = I2C_ENABLE_CLK_STRETCH;
	i2c_handle.Init.OwnAddress1 = SLAVE_ADDRESS;
	
	//Enable iRQ in NVIC 
	NVIC_EnableIRQ(I2Cx_ER_IRQn);  //Interrupt for Error
	NVIC_EnableIRQ(I2Cx_EV_IRQn);  //Interrupt for Event
	
	hal_i2c_init(&i2c_handle);
	
	i2c_handle.State = HAL_I2C_STATE_READY;
	
	
	#ifdef I2C_MASTER_MODE_EN
	
	while(1)
	{
	 //write to slave
	//first send master write command to slave
		master_write_req = MASTER_WRITE_CMD;
		hal_i2c_master_tx(&i2c_handle, SLAVE_ADDRESS_WRITE, (uint8_t*)&master_write_req, 1);
		
		//Application can block here or can do other task until tx finishes
		while(i2c_handle.State != HAL_I2C_STATE_READY);
		
		//Now send the number of bytes to be written
		master_write_req = WRITE_LEN;
		hal_i2c_master_tx(&i2c_handle, SLAVE_ADDRESS_WRITE, (uint8_t*)&master_write_req, 1);
		
		//Application can block here or can do other task until tx finishes
		while(i2c_handle.State != HAL_I2C_STATE_READY);
		
		//Now send the data Stream
	  hal_i2c_master_tx(&i2c_handle, SLAVE_ADDRESS_WRITE, master_tx_buffer, WRITE_LEN);
		
		//Application can block here or can do other task until tx finishes
		while(i2c_handle.State != HAL_I2C_STATE_READY);
		 
		 
		 //Read from Slave
		 
		 //send master read command to slave
		 
		 master_read_req = MASTER_READ_CMD;
		 hal_i2c_master_tx(&i2c_handle, SLAVE_ADDRESS_WRITE, (uint8_t)&master_read_req, 1);
		
		 while(i2c_handle.State != HAL_I2C_STATE_READY);
		 
		 //Now send number of bytes to be read
		 master_read_req = READ_LEN;
		 hal_i2c_master_tx(&i2c_handle, SLAVE_ADDRESS_WRITE, (uint8_t)&master_read_req, 1);
		 
		 while(i2c_handle.State != HAL_I2C_STATE_READY);
		 
		 //Now read the data stream
		 hal_i2c_master_tx(&i2c_handle, SLAVE_ADDRESS_READ, master_rx_buffer, READ_LEN);
		 
		 while(i2c_handle.State != HAL_I2C_STATE_READY);
		 
		 //Compare what received with what slave suppose to be send
		 if(Buffercmp(slave_tx_buffer, master_rx_buffer, READ_LEN))
		 {
			 //ERROR, this is not what slave suppose to send
		 
		 }
		 else
		 {
			 //SUCCESS
		 
		 }
		 
		 delay_gen();
		 
	//	 #else 
	 //if device is slave
		 
		 //first recive the command from the master
		 hal_i2c_slave_rx(&i2c_handle, &slave_rcv_cmd, 1);
		 
		  while(i2c_handle.State != HAL_I2C_STATE_READY);
		 
		 //is it a master write cmd
		 if(slave_rcv_cmd == MASTER_WRITE_CMD)
		 {
		  //Prepare to rcv from master
			 
			 //first recive the number of byte written by master
			 hal_i2c_slave_rx(&i2c_handle, &slave_rcv_cmd, 1);
			 
			  while(i2c_handle.State != HAL_I2C_STATE_READY);
			 
		//	 memset(slave_rx_buffer, 0, sizeof(slave_rx_buffer));
			 
			 //recieve the data from master
			 hal_i2c_slave_rx(&i2c_handle, slave_rx_buffer, slave_rcv_cmd);
			 while(i2c_handle.State != HAL_I2C_STATE_READY);
			 
			 
			 //Compare what recive agaainst what master suppose to recieve
			 if( Buffercmp(slave_rx_buffer, master_tx_buffer, READ_LEN))
			 {
				 //ERROR
			 
			 }
			 else
			 {
				 //SUCCESS
			 
			 }
		 
		 }
		 //is it master read command 
		 
		 {
			 //Prepare to send to master
			 hal_i2c_slave_rx(&i2c_handle, &slave_data_len, 1);
			 
			 while(i2c_handle.State != HAL_I2C_STATE_READY);
			 
			 hal_i2c_slave_tx(&i2c_handle, slave_tx_buffer, slave_data_len);
			 
			 while(i2c_handle.State != HAL_I2C_STATE_READY);
		 
		 
		 }
		 
		 
	
	}		
	
	#endif
	
	return 0;
}

void i2c_gpio_init(void)
{
	gpio_pin_conf_t i2c_sda, i2c_scl;
	
	_HAL_RCC_GPIOB_CLK_ENABLE();
	
	//Configure GPIO Pin 6 Port B for I2C SCL Functionality
	
	i2c_scl.pin = I2C1_SCL_LINE;
	i2c_scl.mode = GPIO_PIN_ALTERNATE_FUNCTION_MODE;
	i2c_scl.op_type = GPIO_PIN_OUTPUT_OPEN_DRAIN;
	i2c_scl.pull = GPIO_PIN_PULL_UP;
	i2c_scl.speed = GPIO_SPEED_HIGH;
	
	hal_gpio_configure_pin_alternate_function(GPIOB, I2C1_SCL_LINE, GPIO_PIN_AF4_I2C123);
	//hal_gpio_init(GPIOB, &i2c_scl);
	
	
	i2c_sda.pin = I2C1_SDA_LINE;
	i2c_sda.mode = GPIO_PIN_ALTERNATE_FUNCTION_MODE;
	i2c_sda.op_type = GPIO_PIN_OUTPUT_OPEN_DRAIN;
	i2c_sda.pull = GPIO_PIN_PULL_UP;
	i2c_sda.speed = GPIO_SPEED_HIGH;
	
		hal_gpio_configure_pin_alternate_function(GPIOB, I2C1_SDA_LINE, GPIO_PIN_AF4_I2C123);
	

}

void EXTIx_IRQHandler(void)
{
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	
	//Do YOur Task here
	TestReady = SET;

}

void i2cx_ER_IRQHandler(i2c_handle_t *hi2cx)
{
	hal_i2c_handle_error_interrupt(hi2cx);
}

void i2cx_EV_IRQHandler(i2c_handle_t *hi2cx)
{
	hal_i2c_handle_event_interrupt(hi2cx);
}

uint8_t Buffercmp(uint8_t buff1[], uint8_t buff2[], uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
		if(buff1[i] == buff2[i])
		{
			return 0;
		
		}
		
	}
 return 0; 
}

void delay_gen()
{
 for(uint8_t i = 0; i < 10000; i++);
}
