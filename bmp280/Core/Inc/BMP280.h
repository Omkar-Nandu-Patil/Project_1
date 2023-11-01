/**
 * @file BMP280.h
 * @author Omkar Nandu Patil (Patilom983@gmail.com)
 * @brief 
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "main.h"
#include "stm32f1xx_hal_conf.h"
#include "stm32f1xx_it.h"
#include <stdbool.h>

//Delay
#define I2C_DELAY			(1000)
#define UART_DElAY			(100)

//mode
typedef enum power_mode{
	// for sleep mode set 0xF4 register last 2 bit 00
	sleep_mode ,

	// for force mode set 0XF4 register last 2 bit 01
	force_mode ,

	// for normal mode set 0xF4 register last 2 bit 10 
	normal_mode   
}power_mode_t;

//OSRS_P for Pressure
typedef enum osrs_pressure{
	// for pressure disable set 3rd,4th and 5th bit 000
	pressure_sampling_disable,

	// oversampling ×1
	pressure_sampling_1,

  // oversampling ×2
	pressure_sampling_2,

  // oversampling ×3
	pressure_sampling_4,

  // oversampling ×4
	pressure_sampling_8,

  // oversampling x5
	pressure_sampling_16    
}osrs_pressure_t;

//OSRS_P for Temperature
typedef enum osrs_temperature{
  //for temperature disable set 6th, 7th and 8th bit 000
	temp_sampling_disable,

  // oversampling x1
	temp_sampling_1,

  // oversampling x2 
	temp_sampling_2,

  // oversampling x4
	temp_sampling_4,

  // oversampling x8
	temp_sampling_8,

  // oversampling x16
	temp_sampling_16  
}osrs_temperature_t;


//Parameters for 0xF5 Register

//time stability
typedef enum time_stability {
  //0.5ms time stability
	time_stability_0_5,

  //62.5ms time stability 
	time_stability_62_5,

  //125ms time stability
	time_stability_125,

  //250ms time stability  
	time_stability_250,

  //500ms time stability   
	time_stability_500,

  //1000 time stability
	time_stability_1000,

  //2000 time stability
	time_stability_2000,

  //4000 time stability 
	time_stability_4000    
}time_stability_t;

//filter
typedef enum filter{
  // filter off ,when coefficient is selected as one.
	filter_1,

  // filter coefficient 2
	filter_2,

  // filter coefficient 4
	filter_4,

  // filter coefficient 8
	filter_8,

  // filter coefficient 16
	filter_16 	
}filter_t;

/**
 * @brief The “ctrl_meas” register sets temperature oversampling, pressure oversampling and mode
 * @param osrs_t [IN] sampling mode for temperature @ref osrsTemperature_t
 * @param osrs_p [IN] sampling mode for pressure  @ref osrsPressure_t
 * @param mode [IN]  mode @ref powerMode_t
 * @retval true on success
 */
bool config_f4_register(osrs_temperature_t osrs_t, osrs_pressure_t osrs_p, power_mode_t mode);

/**
 * @brief It set 0xF5 config register
 * The “config” register sets the rate, filter and interface options of the device
 * @param tstandy [IN] sampling mode for time stability @ref time_stability_t
 * @param filter [IN] sampling mode for filter @ref filter_t
 * @retval ture on success
 */
bool config_f5(	time_stability_t tstandy,filter_t filter);
/**
 * @brief Set 0xF4 and 0xF5 register
 * @param osrs_t [IN] sampling mode for temperature @ref osrsTemperature_t
 * @param osrs_p [IN] sampling mode for pressure  @ref osrsPressure_t
 * @param mode [IN]  mode @ref powerMode_t
 * @param tstandy [IN] sampling mode for time stability @ref time_stability_t
 * @param filter [IN] sampling mode for filter @ref filter_t
 * @retval return true on success
 */
bool init_bmp(osrs_temperature_t temp_mode, osrs_pressure_t pressure_mode,
		power_mode_t p_mode, filter_t filter_mode, time_stability_t time_stability_mode);

/**
 * @brief read row temperature value from register 0xFA....0XFC
 * Using row value and  formula  temperature is calculated
 * @param temperature [OUT] calculated temperature @ref float *temperature
 * @retval true on success
 */
bool read_temperature(float *temperature);

/**
 * @brief read row pressure value from register 0xF7....0XF9
 * Using row value and  formula  pressure  is calculated
 * @param pressure [OUT] calculated pressure  @ref float *pressure
 * @retval true on success
 */
bool read_pressure(double *pressure);

#endif /* INC_BMP280_H_ */

