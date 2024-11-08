/*
 * stm32f_i2c_driver.c
 *
 *  Created on: Nov 8, 2024
 *      Author: armen
 */
#include "stm32f_i2c_driver.h"


i2c_status_t stm32f_i2c_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// CONFIG PB7 and PB8 as AF function
	GPIOB->MODER |= (2 << 14) | (2 << 16);

	// CONFIG PB7 and PB8 as Open Drain
	GPIOB->OTYPER |= (1 << 7) | (1 << 8);

	// CONFIG PB7 and PB8 as full speed
	GPIOB->OSPEEDR |= (3 << 14) | (3 << 16);

	// CONFIG PB7 and PB8 as pull-up
	GPIOB->PUPDR   |= (1 << 14) | (1 << 16);

	// CONFIG PB7 and PB8 as AF4 (I2C)
	GPIOB->AFR[0] |= (4 << 28);
	GPIOB->AFR[1] |= (4 << 0);

	// RESET I2C
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;

	// Correct input clock and timing
	// Setting 48Mhz to frequency FREQ[5:0]
	I2C1->CR2 |= (48 << 0);

	// CCR = ( Tr(SCL) + Tw(SCLH) ) / Tpclk
	I2C1->CCR = (240 << 0);

	// Configure rise time register
	// TRISE = Tr(SCL) / Tpclk + 1
	I2C1->TRISE = 49;

	// Enable I2C
	I2C1->CR1 |= I2C_CR1_PE;

	return I2C_OK;
}


i2c_status_t stm32f_i2c_start(void)
{
	// ENABLE ACK
	I2C1->CR1 |= I2C_CR1_ACK;

	// START MASTER MODE
	I2C1->CR1 |= I2C_CR1_START;

	// WAIT MASTER MODE START SB FLAG
	uint32_t _start_time = HAL_GetTick();

	while(!(I2C1->SR1 & I2C_SR1_SB))
	{
		// RETURN WITH TIMEOUT
		if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
	}
	return I2C_OK;
}


i2c_status_t stm32f_i2c_write_byte(uint8_t data)
{
	uint32_t _start_time = HAL_GetTick();

	// WAIT TRANSMIT COMPLETE BIT
	while(!(I2C1->SR1 & I2C_SR1_TXE))
	{
		// RETURN WITH TIMEOUT
		if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
	}

	// WRITE DATA
	I2C1->DR = data;

	_start_time = HAL_GetTick();

	// WAIT FOR BTF DONE
	while(!(I2C1->SR1 & I2C_SR1_BTF))
	{
		// RETURN WITH TIMEOUT
		if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
	}

	return I2C_OK;
}


i2c_status_t stm32f_i2c_address(uint8_t address)
{
	uint32_t _start_time = HAL_GetTick();

	// SEND THE ADDRESS
	I2C1->DR = address;

	// WAIT ADDR bit to set
	while(!(I2C1->SR1 & I2C_SR1_ADDR))
	{
		// RETURN WITH TIMEOUT
		if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
	}

	// READ SR1 and SR2 to clear ADDR bit
	uint8_t __temp__ = I2C1->SR1 | I2C1->SR2;

	UNUSED(__temp__);

	return I2C_OK;
}


i2c_status_t stm32f_i2c_stop(void)
{
	// STOP I2C
	I2C1->CR1 |= I2C_CR1_STOP;

	return I2C_OK;
}


i2c_status_t stm32f_i2c_write_buff(uint8_t *buff, uint8_t size)
{
	uint32_t _start_time = HAL_GetTick();

	// WAIT TRANSMIT COMPLETE BIT
	while(!(I2C1->SR1 & I2C_SR1_TXE))
	{
		// RETURN WITH TIMEOUT
		if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
	}

	while(size)
	{
		_start_time = HAL_GetTick();

		// WAIT TRANSMIT COMPLETE BIT
		while(!(I2C1->SR1 & I2C_SR1_TXE))
		{
			if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
		}

		// Send BYTE
		I2C1->DR = (volatile uint32_t )*buff++;
		size--;
	}

	_start_time = HAL_GetTick();

	// WAIT FOR BTF DONE
	while(!(I2C1->SR1 & I2C_SR1_BTF))
	{
		if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
	}

	return I2C_OK;
}


i2c_status_t stm32f_i2c_read_buff(uint8_t address, uint8_t *buff, uint8_t size)
{
	uint32_t _start_time = HAL_GetTick();

	uint8_t remaining_bytes = size;

	if(size == 1)
	{
		// SEND ADDRESS
		I2C1->DR = address;

		// WAIT ADDR bit set
		while(!(I2C1->SR1 & I2C_SR1_ADDR))
		{
			// RETURN WITH TIMEOUT
			if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
		}

		// CLEAR ACK BIT
		I2C1->CR1 &= ~I2C_CR1_ACK;

		// READ SR1 and SR2 to clear ADDR bit
		uint8_t __temp__ = I2C1->SR1 | I2C1->SR2;
		UNUSED(__temp__);

		// STOP I2C
		I2C1->CR1 |= I2C_CR1_STOP;

		_start_time = HAL_GetTick();

		// WAIT RXNE SET
		while(!(I2C1->SR1 & I2C_SR1_RXNE))
		{
			if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
		}

		// READ DATA FROM REGISER
		buff[size - remaining_bytes] = I2C1->DR;
	}
	else
	{
		// SEND ADDRESS
		I2C1->DR = address;

		// WAIT ADDR bit set
		while(!(I2C1->SR1 & I2C_SR1_ADDR))
		{
			// RETURN WITH TIMEOUT
			if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
		}

		// READ SR1 and SR2 to clear ADDR bit
		uint8_t __temp__ = I2C1->SR1 | I2C1->SR2;

		UNUSED(__temp__);

		while(remaining_bytes > 2)
		{
			_start_time = HAL_GetTick();

			// WAIT RXNE SET
			while(!(I2C1->SR1 & I2C_SR1_RXNE))
			{
				// RETURN WITH TIMEOUT
				if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
			}

			// READ DATA FROM REGISER
			buff[size - remaining_bytes] = I2C1->DR;

			// SET ACK BIT
			I2C1->CR1 |= I2C_CR1_ACK;

			remaining_bytes--;
		}

		_start_time = HAL_GetTick();

		// READ SECOND LAST BYTE
		while(!(I2C1->SR1 & I2C_SR1_RXNE))
		{
			if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
		}

		// READ DATA FROM REGISER
		buff[size - remaining_bytes] = I2C1->DR;

		// CLEAR ACK BIT
		I2C1->CR1 &= ~I2C_CR1_ACK;

		// STOP I2C
		I2C1->CR1 |= I2C_CR1_STOP;

		remaining_bytes--;

		_start_time = HAL_GetTick();

		// READ SECOND LAST BYTE
		while(!(I2C1->SR1 & I2C_SR1_RXNE))
		{
			if((HAL_GetTick() - _start_time) > I2C_TIMEOUT) { return I2C_ERROR; }
		}

		// READ DATA FROM REGISER
		buff[size - remaining_bytes] = I2C1->DR;
	}
	return I2C_OK;
}


i2c_status_t stm32f_i2c_device_write_byte(uint8_t address, uint8_t reg, uint8_t data)
{
	if(stm32f_i2c_start() != I2C_OK) { return I2C_ERROR; }
	if(stm32f_i2c_address(address) != I2C_OK) { return I2C_ERROR; }
	if(stm32f_i2c_write_byte(reg) != I2C_OK) { return I2C_ERROR; }
	if(stm32f_i2c_write_byte(data) != I2C_OK) { return I2C_ERROR; }
	return stm32f_i2c_stop();
}


i2c_status_t stm32f_i2c_device_write_buff(uint8_t address, uint8_t reg, uint8_t *buff, uint8_t size)
{
	if(stm32f_i2c_start() != I2C_OK) { return I2C_ERROR; }
	if(stm32f_i2c_address(address) != I2C_OK) { return I2C_ERROR; }
	if(stm32f_i2c_write_byte(reg) != I2C_OK) { return I2C_ERROR; }
	if(stm32f_i2c_write_buff(buff, size) != I2C_OK) { return I2C_ERROR; }
	return stm32f_i2c_stop();
}


i2c_status_t stm32f_i2c_device_read_buff(uint8_t address,  uint8_t reg, uint8_t *buff, uint8_t size)
{
	if(stm32f_i2c_start() != I2C_OK) { return I2C_ERROR; }
	if(stm32f_i2c_address(address) != I2C_OK) { return I2C_ERROR; }
	if(stm32f_i2c_start() != I2C_OK) { return I2C_ERROR; }
	if(stm32f_i2c_read_buff(address, buff, size) != I2C_OK) { return I2C_ERROR; }
	return stm32f_i2c_stop();
}


i2c_status_t stm32f_i2c_error_handler(void)
{
	// STOP I2C IF CATCHED SOME ERROR
	return stm32f_i2c_stop();
}






