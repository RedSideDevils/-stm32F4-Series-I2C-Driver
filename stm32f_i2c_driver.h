/*
 * stm32f_i2c_driver.h
 *
 *  Created on: Nov 8, 2024
 *      Author: armen
 */

#ifndef MAIN_DRIVERS_STM32F_I2C_DRIVER_STM32F_I2C_DRIVER_H_
#define MAIN_DRIVERS_STM32F_I2C_DRIVER_STM32F_I2C_DRIVER_H_

#include "main.h"

#define I2C_TIMEOUT	10			// ms

typedef enum
{
	I2C_ERROR = 0,
	I2C_OK
} i2c_status_t;

// =============== [LOCAL] ===============
i2c_status_t stm32f_i2c_start(void);
i2c_status_t stm32f_i2c_write_byte(uint8_t data);
i2c_status_t stm32f_i2c_address(uint8_t address);
i2c_status_t stm32f_i2c_stop(void);
i2c_status_t stm32f_i2c_write_buff(uint8_t *buff, uint8_t size);
i2c_status_t stm32f_i2c_read_buff(uint8_t address, uint8_t *buff, uint8_t size);

// =============== [USER] ===============
i2c_status_t stm32f_i2c_init(void);
i2c_status_t stm32f_i2c_device_write_byte(uint8_t address, uint8_t reg, uint8_t data);
i2c_status_t stm32f_i2c_device_write_buff(uint8_t address, uint8_t reg, uint8_t *buff, uint8_t size);
i2c_status_t stm32f_i2c_device_read_buff(uint8_t address,  uint8_t reg, uint8_t *buff, uint8_t size);

// =============== [CALL IN ERROR HANDLER] ===============
i2c_status_t stm32f_i2c_error_handler(void);

#endif /* MAIN_DRIVERS_STM32F_I2C_DRIVER_STM32F_I2C_DRIVER_H_ */
