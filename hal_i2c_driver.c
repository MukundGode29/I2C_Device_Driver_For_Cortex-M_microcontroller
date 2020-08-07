#include "hal_i2c_driver.h"

static void hal_i2c_enable_peripherial(I2C_TypeDef *i2cx)
{
	i2cx->CR1 = I2C_REG_CR1_ENABLE_I2C;

}

static void hal_i2c_disable_peripheral(I2C_TypeDef *i2cx)
{
	i2cx->CR1 = ~I2C_REG_CR1_ENABLE_I2C;
}

static void hal_i2c_manage_clock_stretch(I2C_TypeDef *i2cx, uint32_t no_stretch)
{
	if(no_stretch)
	{
		i2cx->CR1 = I2C_ENABLE_CLK_STRETCH;
	}
	else
	{
		I2C_DISABLE_CLK_STRETCH;        
	}

}

static void hal_i2c_set_own_address(I2C_TypeDef *i2cx, uint32_t own_address)
{
	i2cx->OAR1 &= ~( 0x7f << 1 );
	i2cx->OAR1 |= (own_address << 1);
}

static void hal_i2c_set_addressing_mode(I2C_TypeDef *i2cx, uint32_t adr_mode)
{
	if(adr_mode == I2C_ADDRMODE_7BIT)
	{
		i2cx->OAR1 &= ~(I2C_REG_OAR1_ADDRMODE);
	}
	else
	{
		i2cx->OAR1 |= I2C_REG_OAR1_ADDRMODE;
	}
}

static void hal_i2c_set_fm_duty_cycle(I2C_TypeDef *i2cx, uint32_t duty_cycle)
{
	if(duty_cycle == I2C_FM_DUTY_16BY9)
	{
		i2cx->CCR |= I2C_REG_CCR_DUTY;
	}
	else
	{
		i2cx->CCR &= ~(I2C_REG_CCR_DUTY);
	}
}

static void hal_i2c_clk_init(I2C_TypeDef *i2cx, uint32_t clk_speed, uint32_t duty_cycle)
{
	 uint32_t PCLK = I2C_PERIPHERIAL_CLK_FREQ_10MHZ;
	 i2cx->CR2 &= ~(0x3f);
	 i2cx->CR2 |= (PCLK & 0x3f);
	 hal_i2c_configure_ccr(i2cx, pclk, clkspeed, duty_cycle);
	 hal_i2c_rise_time_configuration(i2cx, pclk, clkspeed);
	
}

static void hal_i2c_generate_start_condition(I2C_TypeDef *i2cx)
{
	i2cx->CR1 = I2C_REG_CR1_START_GEN;
}

static void hal_i2c_generate_stop_condition(I2C_TypeDef *i2cx)
{
	i2cx->CR1 = I2C_REG_CR1_STOP_GEN;
}

static void hal_i2c_configure_buffer_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
	if(enable)
	{
	  i2cx->CR2 |= I2C_REG_CR2_BUF_INT_ENABLE;
	}
	else
	{
		i2cx->CR2 &= ~I2C_REG_CR2_BUF_INT_ENABLE;
	}
}

static void hal_i2c_configure_error_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
	if(enable)
	{
	  i2cx->CR2 |= I2C_REG_CR2_ERR_INT_ENABLE;
	}
	else
	{
		i2cx->CR2 &= ~I2C_REG_CR2_ERR_INT_ENABLE;
	}
}

static void hal_i2c_configure_event_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
	if(enable)
	{
	  i2cx->CR2 |= I2C_REG_CR2_EVT_INT_ENABLE;
	}
	else
	{
		i2cx->CR2 &= ~I2C_REG_CR2_EVT_INT_ENABLE;
	}
}

static uint8_t is_bus_busy(I2C_TypeDef *i2cx)
{
	if(i2cx->SR2 & I2C_BUS_IS_BUSY)
	{
		return 1;
	}
	else
	{
	 return 0;
	}
} 

static void i2c_wait_until_sb_set(I2C_TypeDef *i2cx)
{
	while( ! (i2cx->SR1 & I2C_REG_SR1_SB_FLAG));
}

static void i2c_wait_until_addr_set(I2C_TypeDef *i2cx)
{
	while( ! (i2cx->SR1 & I2C_REG_SR1_ADDR_FLAG));
}

static void hal_i2c_send_addr_first(I2C_TypeDef *i2cx, uint32_t slave_add)
{
 i2cx->DR &= 0x0;
 i2cx->DR |= slave_add;
}

static void clear_addr_flag(I2C_TypeDef *i2cx)
{
	  uint32_t tempreg;
	
		i2cx->SR1 &= ~(I2C_REG_SR1_ADDR_SENT_FLAG);
	tempreg = i2cx->SR2;

}

static void clear_stop_flag(i2c_handle_t *hi2c)
{
	hi2c->Instance->CR1 &= ~(I2C_REG_CR1_STOP_GEN);
}
	

static void hal_i2c_master_handle_TXE_interrupt(i2c_handle_t *hi2c)
{
	//write Data into Data Register
	hi2c->Instance->DR = (*hi2c->pBuffer++);
	hi2c->XferCount--;
	
	if(hi2c->XferCount == 0)
	{
		hi2c->Instance->CR2 &= ~I2C_REG_CR2_BUF_INT_ENABLE;
	}

}

static void hal_i2c_slave_handle_TXE_interrupt(i2c_handle_t *hi2c)
{
	if(hi2c->XferCount != 0)
	{
	 hi2c->Instance->DR = (*hi2c->pBuffer++);
	 hi2c->XferCount--;
	}

}

static void hal_i2c_slave_tx_handle_btf(i2c_handle_t *hi2c)
{
	
	if(hi2c->XferCount != 0)
	{
		hi2c->Instance->DR = (*hi2c->pBuffer++);
		hi2c->XferCount--;
	
	}

}

static void hal_i2c_slave_handle_RXNE_interrupt(i2c_handle_t *hi2c)
{
	if(hi2c->XferCount != 0)
	{
		//Read from Data Register
		(*hi2c->pBuffer++) = (uint8_t)hi2c->Instance->DR;
		hi2c->XferCount--;
	}
}

static void hal_i2c_slave_rx_handle_btf(i2c_handle_t *hi2c)
{
	if(hi2c->XferCount != 0)
	{
		//Read from Data Register
		(*hi2c->pBuffer++) = (uint8_t)hi2c->Instance->DR;
		hi2c->XferCount--;
}
}


static void hal_i2c_master_tx_complt(i2c_handle_t *hi2c)
{
	//call application callback here if needed
}

static void hal_i2c_slave_rx_complt(i2c_handle_t *hi2c)
{
	//call application callback here if needed
}

static void hal_i2c_slave_tx_complt(i2c_handle_t *hi2c)
{
	//call application callback here if needed
}

static void hal_i2c_master_handle_btf(i2c_handle_t *hi2c)
{
	if(hi2c->XferCount != 0)
	{
		hi2c->Instance->DR = (*hi2c->pBuffer++);
		hi2c->XferCount--;
	}
	else
	{
		hi2c->Instance->CR2 &= ~I2C_REG_CR2_BUF_INT_ENABLE;
	  hi2c->Instance->CR2 &= ~I2C_REG_CR2_EVT_INT_ENABLE;
		hi2c->Instance->CR2 &= ~I2C_REG_CR2_ERR_INT_ENABLE;
		
		//Generate Stop Condition
		hi2c->Instance->CR1 |= I2C_REG_CR1_STOP_GEN;
		
		hi2c->State = HAL_I2C_STATE_READY;
		
		hal_i2c_master_tx_complt(hi2c);
		
	}
}

static void hal_i2c_slave_handle_stop_condition(i2c_handle_t *hi2c)
{
	
		hi2c->Instance->CR2 &= ~I2C_REG_CR2_BUF_INT_ENABLE;
	  hi2c->Instance->CR2 &= ~I2C_REG_CR2_EVT_INT_ENABLE;
		hi2c->Instance->CR2 &= ~I2C_REG_CR2_ERR_INT_ENABLE;
		
	  clear_stop_flag(hi2c);
	
	  //Disable Acknowledgemnt
	  hi2c->Instance->CR2 &= ~I2C_CR1_ACK;
	
	hi2c->State = HAL_I2C_STATE_READY;
	
	hal_i2c_slave_rx_complt(hi2c);
	

}

static void hal_i2c_slave_handle_ack_failure(i2c_handle_t *hi2c)
{
	//disable evt, buff, error interrupt
	hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
	hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
	hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;
	
	//CLear AF flags
	hi2c->Instance->SR1 &= ~(I2C_REG_SR1_AF_FAILURE_FLAG);
	
	//Disable Acknowledgement
	hi2c->Instance->CR1 &= ~I2C_REG_CR1_ACK;
	
	hi2c->State = HAL_I2C_STATE_READY;
	
	hal_i2c_slave_tx_complt(hi2c);

}



	
//***********************************************************/

void hal_i2c_master_tx(i2c_handle_t *handle, uint8_t slave_address, uint8_t *buffer, uint32_t len)
{
	handle->pBuffer = buffer;
	handle->XferCount = len;
	handle->XferSize = len;
	
	handle->State = HAL_I2C_STATE_BUSY_TX;
	
	hal_i2c_enable_peripherial(handle->Instance);
	
	hal_i2c_generate_start_condition(handle->Instance);
	
	i2c_wait_until_sb_set(handle->Instance);
	
	hal_i2c_send_addr_first(handle->Instance, slave_address);
	
	i2c_wait_until_addr_set(handle->Instance);
	
	//if you are here, that means addr is set and clock is stretched (wait state)
	//clear ADDR flags makes I2C to come out of wait state
	
	clear_addr_flag(handle->Instance);
	
	hal_i2c_configure_buffer_interrupt(handle->Instance, 1);
	hal_i2c_configure_error_interrupt(handle->Instance, 1);
	hal_i2c_configure_event_interrupt(handle->Instance, 1);
	
	

}

void hal_i2c_master_rx(i2c_handle_t *handle, uint8_t slave_address, uint8_t *buffer, uint32_t len)
{
	handle->pBuffer = buffer;
	handle->XferCount = len;
	handle->XferSize = len;
	
	handle->State = HAL_I2C_STATE_BUSY_TX;
	
	hal_i2c_enable_peripherial(handle->Instance);
	
	//make sure the POS BIt is disable
	handle->Instance->CR1 &= ~I2C_REG_CR1_POS;
	
	//ACK is enable
	handle->Instance->CR1 |= I2C_REG_CR1_ACK;
	
	hal_i2c_generate_start_condition(handle->Instance);
	
	i2c_wait_until_sb_set(handle->Instance);
	
	hal_i2c_send_addr_first(handle->Instance, slave_address);
	
	i2c_wait_until_addr_set(handle->Instance);
	
	//if you are here, that means addr is set and clock is stretched (wait state)
	//clear ADDR flags makes I2C to come out of wait state
	
	clear_addr_flag(handle->Instance);
	
	//ENable the interrupt
	hal_i2c_configure_buffer_interrupt(handle->Instance, 1);
	hal_i2c_configure_error_interrupt(handle->Instance, 1);
	hal_i2c_configure_event_interrupt(handle->Instance, 1);
	

}

void hal_i2c_slave_tx(i2c_handle_t *handle, uint8_t *buffer, uint32_t len)
{
	handle->pBuffer = buffer;
	handle->XferCount = len;
	handle->XferSize = len;
	
	handle->State = HAL_I2C_STATE_BUSY_TX;
	
	handle->Instance->CR1 &= ~I2C_CR1_POS;
	
	hal_i2c_enable_peripherial(handle->Instance);
	
	handle->Instance->CR1 |=  I2C_CR1_ACK;
	
	
	//Enable the interrupt
	hal_i2c_configure_buffer_interrupt(handle->Instance, 1);
	hal_i2c_configure_error_interrupt(handle->Instance, 1);
	hal_i2c_configure_event_interrupt(handle->Instance, 1);
	
	
}


void hal_i2c_slave_rx(i2c_handle_t *handle, uint8_t *buffer, uint32_t len)
{
	handle->pBuffer = buffer;
	handle->XferCount = len;
	handle->XferSize = len;
	
	handle->State = HAL_I2C_STATE_BUSY_RX;
	
	handle->Instance->CR1 &= ~I2C_CR1_POS;
	
	hal_i2c_enable_peripherial(handle->Instance);
	
	handle->Instance->CR1 |=  I2C_CR1_ACK;
	
	
	//Enable the interrupt
	hal_i2c_configure_buffer_interrupt(handle->Instance, 1);
	hal_i2c_configure_error_interrupt(handle->Instance, 1);
	hal_i2c_configure_event_interrupt(handle->Instance, 1);
	
	
}

void hal_i2c_handle_event_interrupt(i2c_handle_t *hi2c)
{
 uint32_t tempreg = (hi2c->Instance->SR2);
 uint32_t temp1;
 uint32_t temp2;
 uint32_t temp3;
 uint32_t temp4;
	
 if(tempreg & I2C_REG_SR2_MSL_FLAG)//master mode
 {
	 if(tempreg & I2C_REG_SR2_TRA_FLAG) //Rx mode
	 {
	 
	 }
	 else//TX mode
	 {
	 }
	 
 }
 //slave mode
 else 
 {
	 //if we are here then interrupt event occur for the slave
	 //lets cheack why this event occur for slave by checking the status register
	 temp1 = ( hi2c->Instance->SR2 & I2C_REG_SR1_ADDR_FLAG);
	 temp2 = ( hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE);
	 temp3 = ( hi2c->Instance->SR1 & I2C_REG_SR1_STOP_DETECTION_FLAG);
   temp4 = ( hi2c->Instance->SR2 & I2C_REG_SR2_TRA_FLAG);
	 
	 //ADDR Set SLave Address matched
	 if((temp1) && (temp2))
	 {
		 //Do Led blink here for indication
		 
		 //until you clear ADDR flag, I2C1 clock will be stretched
		 clear_addr_flag(hi2c->Instance);
	 }
	 
	 else if((temp3) && (temp2))
	 {
		 hal_i2c_slave_handle_stop_condition(hi2c);
	 }
	 
	 //I2c in transmit mode
	 else if(temp4)
	 {
		 temp1 = ( hi2c->Instance->SR2 & I2C_REG_SR1_TXE_FLAG);
	   temp2 = ( hi2c->Instance->CR2 & I2C_REG_CR2_BUF_INT_ENABLE);
	   temp3 = ( hi2c->Instance->SR1 & I2C_REG_SR1_BTF_FLAG);
     temp4 = ( hi2c->Instance->SR2 & I2C_REG_CR2_EVT_INT_ENABLE);
	 
	 //TXE set and BTF reset
		 if((temp1) && (temp2) && (!temp3) )
		 {
			 hal_i2c_slave_handle_TXE_interrupt(hi2c);
		 }
		 else if((temp3) && (temp4))
		 {
			 hal_i2c_slave_tx_handle_btf(hi2c);
		 
		 }
	 
	 }
	 //I2C in recive mode
	 else
	 {
	   temp1 = ( hi2c->Instance->SR2 & I2C_REG_SR1_RXNE_FLAG);
	   temp2 = ( hi2c->Instance->CR2 & I2C_REG_CR2_BUF_INT_ENABLE);
	   temp3 = ( hi2c->Instance->SR1 & I2C_REG_SR1_BTF_FLAG);
     temp4 = ( hi2c->Instance->SR2 & I2C_REG_CR2_EVT_INT_ENABLE);
	 
		 if((temp1) && (temp2) && (!temp3) )
		 {
			 hal_i2c_slave_handle_RXNE_interrupt(hi2c);
		 }
		 else if((temp3) && (temp4))
		 {
			 hal_i2c_slave_rx_handle_btf(hi2c);
		 
		 }
	 
		 
	 }
		 
 }
 
}

void hal_i2c_handle_error_interrupt(i2c_handle_t *hi2c)
{
	uint32_t temp1 = 0, temp2 = 0, temp3 = 0;
	
	temp1 = (hi2c->Instance->SR1 & I2C_REG_SR1_BUS_ERROR_FLAG);
	temp2 = (hi2c->Instance->SR1 & I2C_REG_CR2_ERR_INT_ENABLE);
	
	//I2C bus error interruot occur
	if((temp1) && (temp2))
	{
		hi2c->ErrorCode |= HAL_I2C_ERROR_BERR;
		
		//Clear BERR Flag
		hi2c->Instance->SR1 &= ~(I2C_REG_SR1_BUS_ERROR_FLAG);
	
	}
	
	temp1 = (hi2c->Instance->SR1 & I2C_REG_SR1_ARLO_FLAG);
	temp2 = (hi2c->Instance->SR1 & I2C_REG_CR2_ERR_INT_ENABLE);
	
	//I2C Arbritration error interruot occur
	if((temp1) && (temp2))
	{
		hi2c->ErrorCode |= HAL_I2C_ERROR_ARLO;
		
		//Clear BERR Flag
		hi2c->Instance->SR1 &= ~(I2C_REG_SR1_ARLO_FLAG);
	
	}
	
	temp1 = (hi2c->Instance->SR1 & I2C_REG_SR1_AF_FAILURE_FLAG);
	temp2 = (hi2c->Instance->SR1 & I2C_REG_CR2_ERR_INT_ENABLE);
	
	//I2C Acknowledge failure error interruot occur
	if((temp1) && (temp2))
	{
		temp1 = ( hi2c->Instance->SR2 & I2C_REG_SR2_MSL_FLAG);
		temp2 = hi2c->XferCount;
		temp3 = hi2c->State;
		
	  if((!temp1) && (temp2 == 0) && (temp3 == HAL_I2C_STATE_BUSY_TX))
		{
			//if ack failure happens for slave
			//then slave should assume that master dont want more data
			//SO it should stop sending more data to master
			 hal_i2c_slave_handle_ack_failure(hi2c);
		
		}
		else
		{
		hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
		
		//Clear AF Flag
		hi2c->Instance->SR1 &= ~(I2C_REG_SR1_AF_FAILURE_FLAG);
	  }
	}
	
	
	temp1 = (hi2c->Instance->SR1 & I2C_REG_SR1_OVR_FLAG);
	temp2 = (hi2c->Instance->SR1 & I2C_REG_CR2_ERR_INT_ENABLE);
	
	//I2C Overrun error interruot occur
	if((temp1) && (temp2))
	{
		hi2c->ErrorCode |= HAL_I2C_ERROR_OVR;
		
		//Clear AF Flag
		hi2c->Instance->SR1 &= ~(I2C_REG_SR1_OVR_FLAG);
	
	}
	
	if(hi2c->ErrorCode |= (HAL_I2C_ERROR_NONE))
	{
		hi2c->State = HAL_I2C_STATE_READY;
		
		hi2c->Instance->CR1 &= ~I2C_REG_CR1_POS;
		
		hal_i2c_error_cb(hi2c);
	}
	
}

void hal_i2c_error_cb(i2c_handle_t *i2cHandle)
	{
		//Blink the led to indicate error interrupt
	
	}
	
