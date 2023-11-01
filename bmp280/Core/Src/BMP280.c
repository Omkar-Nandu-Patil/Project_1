/**
 * @file BMP280.c
 * @author Omkar Nandu Patil (patilom983@gmail.com)
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "math.h"
#include "BMP280.h"
#include "stm32f1xx_hal.h"
#include "math.h"
#include <stdbool.h>

extern I2C_HandleTypeDef		hi2c1;
extern UART_HandleTypeDef		huart1;
#define BMP280_I2C				&hi2c1
#define BMP280_ADDRESS			(0xEC)


#define BMP280_REG_ID			(0xD0)
#define BMP280_REG_RESET		(0xE0)
#define BMP280_REG_STATUS		(0xF3)
#define BMP280_REG_CTRL_MEAS	(0xF4)
#define BMP280_REG_CONFIG		(0xF5)
#define BMP280_REG_PRESS		(0xF7)
#define BMP280_REG_TEMP			(0xFA)
#define BMP280_REG_CTRL			(0xF4)
#define BMP280_REG_CALIB		(0x88)
#define RESET_VALUE				(0xB6)

//Temperature sampling
#define OSRS_TEMP_MASK			(0x1F)
#define OSRS_TEMP_SAMPLING_0	(0)
#define OSRS_TEMP_SAMPLING_1	(0x01 << 5)
#define OSRS_TEMP_SAMPLING_2	(0x02 << 5)
#define OSRS_TEMP_SAMPLING_3	(0x03 << 5)
#define OSRS_TEMP_SAMPLING_4	(0x04 << 5)
#define OSRS_TEMP_SAMPLING_5	(0x05 << 5)

//pressure sampling
#define OSRS_PRESSURE_MASK		(0xE3)
#define OSRS_PRESSURE_SAMPLE_0	(0)
#define OSRS_PRESSURE_SAMPLE_1	(0X01 << 2)
#define OSRS_PRESSURE_SAMPLE_2	(0X02 << 2)
#define OSRS_PRESSURE_SAMPLE_3	(0x03 << 2)
#define OSRS_PRESSURE_SAMPLE_4	(0X04 << 2)
#define OSRS_PRESSURE_SAMPLE_5	(0X05 << 2)

//Mode
#define MODE_MASK		(0xFC)
#define MODE_0			(0)
#define MODE_1			(0X01)
#define MODE_2			(0X03)

//T stand by
#define TIME_STANDBY_MASK	(0x1F)
#define T_STANDBY_0			(0)
#define T_STANDBY_1			(0X01 << 5)
#define T_STANDBY_2			(0X02 << 5)
#define T_STANDBY_3			(0X03 << 5)
#define T_STANDBY_4			(0X04 << 5)
#define T_STANDBY_5			(0X05 << 5)
#define T_STANDBY_6			(0X06 << 5)
#define T_STANDBY_7			(0X07 << 5)

// SET filter
#define FILTER_MASK			(0XE3)
#define FILTER_0			(0)
#define FILTER_1			(0X01 << 2)
#define FILTER_2			(0X02 << 2)
#define FILTER_3			(0X03 << 2)
#define FILTER_4			(0X04 << 2)

//set SPI
#define SPI_MASK			(0xFE)
#define SPI_DIS 			(0x00)
#define SPI_EN				(0x01)



//Macros to calculate the temperature
#define MASK1				(16384.0)
#define MASK2				(1024.0)
#define MASK3				(131072.0)
#define MASK4				(8192.0)
#define MASK5				(5120.0)
#define MASK6				(40.59)

#define CALLIB_DATA_SIZE			(24)
#define TEMPERATURE__DATA_SIZE		(6)
#define PRESSURE_DATA_SIZE			(6)

// TO convert ferhanite to degree
#define TEMPERATURE_CELCIUS(T)   ( ( (T-32)*(5.0))/(9.0))

//Macros to calculate the pressure
#define MASKP1				(2.0)
#define MASKP2				(64000.0)
#define MASKP3				(32768.0)
#define MASKP4				(65536.0)
#define MASKP5				(524288.0)
#define MASKP6				(32768.0)
#define MASKP7				(1.0)
#define MASKP8				(1048576.0)
#define MASKP9				(4096.0)
#define MASKP10				(6250.0)
#define MASKP11				(2147483648.0)
#define MASKP12				(16.0)
#define MASKP13				(2.5)

#define SIZE_OF_REGISTER				(1)

static int32_t t_fine;
static int32_t var1, var2;

//calliberated data
uint16_t dig_temp1; /**< dig_temp1 cal register. */
int16_t dig_temp2;  /**<  dig_temp2 cal register. */
int16_t dig_temp3;  /**< dig_temp3 cal register. */

uint16_t dig_pressure1; /**< dig_pressure1 cal register. */
int16_t dig_pressure2;  /**< dig_pressure2 cal register. */
int16_t dig_pressure3;  /**< dig_pressure3 cal register. */
int16_t dig_pressure4;  /**< dig_pressure4 cal register. */
int16_t dig_pressure5;  /**< dig_pressure5 cal register. */
int16_t dig_pressure6;  /**< dig_pressure6 cal register. */
int16_t dig_pressure7;  /**< dig_pressure7 cal register. */
int16_t dig_pressure8;  /**< dig_pressure8 cal register. */
int16_t dig_pressure9;  /**< dig_pressure9 cal register. */


/**
 * @brief it rest the BMP280 sensor.
 * @retval  true on success
 */
 static bool bmp280_reset();

 /**
  * @brief it set temperature configuration
  * @param mode [IN] sampling mode @ref osrsTemperature_t
  * @ retval true on success
  */
 static bool osrs_t_config(osrs_temperature_t mode);

 /**
  * @brief it set pressure configuration
  * @param pres_sample [IN] sampling mode @ref osrsPressure_t
  * @retval true on succes
  */
 static bool osrs_p_config(osrs_pressure_t pres_sample);

 /**
  * @brief it set mode configuration
  * @param mode [IN] sampling mode @ref powerMode_t
  * @retval true on success
  */
 static bool mode_config(power_mode_t mode);

 /**
  * @brief it configure time stability
  * @param tstandby [IN] sampling mode @ref time_stability_t
  * @retval true on success
  */
 static bool config_t_sb(time_stability_t tstandby);

 /**
  * @brief it configure the filter
  * @param filter_mode [IN] sampling mode @ref filter_t
  * @retval true on success
  */
 static bool config_filter(filter_t filter_mode);

/**
 * @brief it disable SPI
 * @retval true on success
 */
 static bool spi3w_disable();

/**
 * @brief it enable the SPI
 * @retval true on success
 */
 static bool spi3w_en();

 /**
  * @brief it callibarated data from 0x88 register
  * @ retval true on success
  */
 static bool read_calliberation_data_280 (void);

 /**
  * @brief it read row data from sensor using I2c
  * @param raw_temp [OUT] sampling mode for read_row_data_temp @ref uint32_t *raw_t
  * @retval true on success
  */
 static bool read_row_data_temp(uint32_t * raw_temp);

/**
  * @brief it read row data from sensor using I2c
  * @param  mode [OUT] sampling mode for read_row_data_pressure @ref int32_t *raw_p
  * @retval true on success
  */
static bool read_row_data_pressure(int32_t *raw_pressure);

static bool bmp280_reset()
{
	bool status = false;
	int8_t data = RESET_VALUE;

	//Writting reset value in reset register using I2C protocal
	if( HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS,
			BMP280_REG_RESET, SIZE_OF_REGISTER, &data, sizeof(data), I2C_DELAY)
				==HAL_OK)
	{
		status = true;
		HAL_Delay(10);
	}

	return status;

}


static bool osrs_t_config(osrs_temperature_t mode)
{
	bool status = false;
	uint8_t data;

	//Reading data from control register (0xF4)
	if(HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS,
			BMP280_REG_CTRL_MEAS, SIZE_OF_REGISTER, &data, sizeof(data), HAL_MAX_DELAY)
				==HAL_OK)
	{
		status = true;
		switch(mode)
		{
		     case  temp_sampling_disable:
		    	 data = data & OSRS_TEMP_MASK;
		    	 data = data |OSRS_TEMP_SAMPLING_0;
		    	 break;

		     case  temp_sampling_1:
		    	 data = data & OSRS_TEMP_MASK;
		    	 data = data | OSRS_TEMP_SAMPLING_1;
		    	 break;

		     case  temp_sampling_2:
		    	 data = data & OSRS_TEMP_MASK;
		    	 data = data | OSRS_TEMP_SAMPLING_2;
		    	 break;

		     case  temp_sampling_4:
		    	 data = data & OSRS_TEMP_MASK;
		    	 data = data | OSRS_TEMP_SAMPLING_3;
		    	 break;

		     case  temp_sampling_8:
		    	 data = data & OSRS_TEMP_MASK;
		    	 data = data | OSRS_TEMP_SAMPLING_4;
		    	 break;

		     case  temp_sampling_16:
		    	 data = data & OSRS_TEMP_MASK;
		    	 data = data | OSRS_TEMP_SAMPLING_5;
		    	 break;

		     default :
		    	 status = false;
		}
	}

	if(status == true)
	{
		//after bitmasking data is written in control register (0xF4)
		if(HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS,
				BMP280_REG_CTRL_MEAS, SIZE_OF_REGISTER, &data, sizeof(data), I2C_DELAY)
					== HAL_OK)
		{
			status = true;
		}
	}

	return status;
}


static bool osrs_p_config(osrs_pressure_t pressure_sample)
{
	 bool status = false;
	 uint8_t data;

	 //Reading data from control register (0xF4)
	 if(HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS,
			 BMP280_REG_CTRL_MEAS, SIZE_OF_REGISTER, &data, sizeof(data), HAL_MAX_DELAY)
			 	 == HAL_OK)
	 {
		 status = true;
		 switch(pressure_sample)
		 {
		 	  case pressure_sampling_disable :
		 		  data = data & OSRS_PRESSURE_MASK;
		 		  data = data | OSRS_PRESSURE_SAMPLE_0;
		 		  break;

		 	   case pressure_sampling_1  :
		 		   data = data & OSRS_PRESSURE_MASK;
		 		   data = data | OSRS_PRESSURE_SAMPLE_1;
		 		   break;

		 	   case pressure_sampling_2  :
		 		   data = data & OSRS_PRESSURE_MASK;
		 		   data = data | OSRS_PRESSURE_SAMPLE_2;
		 		   break;

		 	   case pressure_sampling_4 :
		 		   data = data & OSRS_PRESSURE_MASK;
		 		   data = data | OSRS_PRESSURE_SAMPLE_3;
		 		   break;

		 	   case pressure_sampling_8  :
		 		   data = data & OSRS_PRESSURE_MASK;
		 		   data = data | OSRS_PRESSURE_SAMPLE_4;
		 		   break;

		 	   case pressure_sampling_16  :
		 		   data = data & OSRS_PRESSURE_MASK;
		 		   data = data | OSRS_PRESSURE_SAMPLE_5;
		 		   break;

		 	   default:
		 		   status = false;
		 }
          ////after bitmasking data is written in control register (0xF4)
		 if(status == false)
		 {
			 if(HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS,
					 BMP280_REG_CTRL_MEAS, SIZE_OF_REGISTER, &data, sizeof(data), I2C_DELAY)
					 == HAL_OK)
			 {
				 status = true;
			 }
		 }
	 }

	 return status;
 }


static bool mode_config(power_mode_t mode)
{
	bool status = false;
	uint8_t data;

	//Reading data from control register (0xF4)
	if(HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS,
			BMP280_REG_CTRL_MEAS, SIZE_OF_REGISTER, &data, sizeof(data), HAL_MAX_DELAY)
				== HAL_OK)
	{
		status = true;
		switch(mode)
		{
			case sleep_mode:
				data = data & MODE_MASK;
				data = data | MODE_0;
				break;

			case force_mode:
				data = data & MODE_MASK;
				data = data | MODE_1;
				break;

			case normal_mode:
				data = data & MODE_MASK;
				data = data | MODE_2;
				break;

			default:
				status = false;
		}

		if(status == true)
		{
			//after bitmasking data is written in control register (0xF4)
			if(HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS,
					BMP280_REG_CTRL_MEAS, SIZE_OF_REGISTER, &data, sizeof(data), I2C_DELAY)
						== HAL_OK)
			{
				status = true;
			}
		}
	}

	return status;
}


bool config_f4_register(osrs_temperature_t osrs_temp,osrs_pressure_t osrs_pressure,	power_mode_t mode)
{
	bool status = false;

	if(osrs_t_config(osrs_temp) && osrs_p_config(osrs_pressure) && mode_config(mode))
	{
		status = true;
	}

	if(status == false)
	{
		unsigned char MSG[30] = "Failed to set mode\r\n";
		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), UART_DElAY);
	}

	return status;
}


static bool config_t_sb(time_stability_t time_stability)
{
	bool status = false;
	uint8_t data;

	//Reading data from config register (0xF5)
	if(HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS,
			BMP280_REG_CONFIG, SIZE_OF_REGISTER, &data, SIZE_OF_REGISTER, HAL_MAX_DELAY)
				==HAL_OK)
	{
		status = true;
		switch(time_stability)
		{
			case time_stability_0_5:
				data = data & TIME_STANDBY_MASK;
				data = data | T_STANDBY_0;
				break;

			case time_stability_62_5:
				data = data & TIME_STANDBY_MASK;
				data = data | T_STANDBY_1;
				break;

			case time_stability_125:
				data = data & TIME_STANDBY_MASK;
				data = data | T_STANDBY_2;
				break;

			case time_stability_250:
				data = data & TIME_STANDBY_MASK;
				data = data | T_STANDBY_3;
				break;

			case time_stability_500:
				data = data & TIME_STANDBY_MASK;
				data = data | T_STANDBY_4;
				break;

			case time_stability_1000:
				data = data & TIME_STANDBY_MASK;
				data = data |T_STANDBY_5;
				break;

			case time_stability_2000:
				data = data & TIME_STANDBY_MASK;
				data = data |T_STANDBY_6;
				break;

			case time_stability_4000:
				data = data & TIME_STANDBY_MASK;
				data = data |T_STANDBY_7;
				break;

			default:
				status = false;

		}

		if(status == true)
		{
			// after bitmasking data is written in config register
			if(HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS,
					BMP280_REG_CONFIG, SIZE_OF_REGISTER, &data, sizeof(data), I2C_DELAY)
						==HAL_OK)
			{
				status = true;
			}
		}
	}

	return status;
}


static bool config_filter(filter_t filter_mode)
{
	bool status = false;
	uint8_t data;

	//Reading data from config register (0xF5)
	if(HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS,
			BMP280_REG_CONFIG, SIZE_OF_REGISTER, &data, sizeof(data), HAL_MAX_DELAY)
				==HAL_OK)
	{
		status = true;
		switch(filter_mode)
		{
			case filter_1:
				data = data & FILTER_MASK;
				data = data | FILTER_0;
				break;

			case filter_2:
				data = data & FILTER_MASK;
				data = data | FILTER_1;
				break;

			case filter_4:
				data = data & FILTER_MASK;
				data = data | FILTER_2;
				break;

			case filter_8:
				data = data & FILTER_MASK;
				data = data | FILTER_3;
				break;

			case filter_16:
				data = data & FILTER_MASK;
				data = data | FILTER_4;
				break;

			default :
				status = false;

		}

		if(status == true)
		{
			// after bitmasking data is written in config register
			if(HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS,
					BMP280_REG_CONFIG, SIZE_OF_REGISTER, &data, sizeof(data), I2C_DELAY)
						==HAL_OK)
			{
				status = true;
			}
		}
	}

	return status;
}


static bool spi3w_disable()
{
	uint8_t data;
	bool status = false;

	//Reading data from config register (0xF5)
	if(HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS,
			BMP280_REG_CONFIG, SIZE_OF_REGISTER, &data, sizeof(data), HAL_MAX_DELAY)
				==HAL_OK)
	{
		data = data & SPI_MASK;
		data = data | SPI_DIS;

		// after bitmasking data is written in config register
		if(HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS,
				BMP280_REG_CONFIG, SIZE_OF_REGISTER, &data, sizeof(data), I2C_DELAY)
					==HAL_OK)
		{
			status = true;
		}
	}

	return status;
}

static bool spi3w_en()
{
	uint8_t data;
	bool status = false;

	//Reading data from config register (0xF5)
	if(HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS,
			BMP280_REG_CONFIG, SIZE_OF_REGISTER, &data, sizeof(data), HAL_MAX_DELAY)
				==HAL_OK)
	{
		data = data & SPI_MASK;
		data = data | SPI_EN;

		// after bitmasking data is written in config register
		if(HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS,
				BMP280_REG_CONFIG, SIZE_OF_REGISTER, &data, sizeof(data), I2C_DELAY)
					==HAL_OK)
		{
			status = true;
		}
	}

	return status;

}

bool config_f5(	time_stability_t time_stability,filter_t filter)
{
	bool status = false;

	if(config_t_sb(time_stability) && config_filter(filter) && spi3w_disable())
	{
		status =  true;
	}

	if(status== false)
	{
		unsigned char MSG[50] = "Failed to configure F5 register\r\n";
		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), UART_DElAY);
	}

	return status;
}


static bool read_calliberation_data_280 (void)
{
	bool status = false;
	uint8_t callib_data[CALLIB_DATA_SIZE] = {0};

	//Reading data from register (0x88)
	if(HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS,
			BMP280_REG_CALIB, SIZE_OF_REGISTER, callib_data, sizeof(callib_data), HAL_MAX_DELAY)
				==HAL_OK)
	{
		dig_temp1 = ((callib_data[0] << 8) | callib_data[1]);
		dig_temp2 = ((callib_data[2] << 8) | callib_data[3]);
		dig_temp3 = ((callib_data[4] << 8) | callib_data[5]);

		dig_pressure1 = ((callib_data[6] << 8) | callib_data[7]);
		dig_pressure2 = ((callib_data[8] << 8) | callib_data[9]);
		dig_pressure3 = ((callib_data[10] << 8) | callib_data[11]);
		dig_pressure4 = ((callib_data[12] << 8) | callib_data[13]);
		dig_pressure5 = ((callib_data[14] << 8) | callib_data[15]);
		dig_pressure6 = ((callib_data[16] << 8) | callib_data[17]);
		dig_pressure7 = ((callib_data[18] << 8) | callib_data[19]);
		dig_pressure8 = ((callib_data[20] << 8) | callib_data[21]);
		dig_pressure9 = ((callib_data[22] << 8) | callib_data[23]);
		status = true;
	}

	return status;
}

#define TEMPERATURE__DATA_SIZE		(6)
static bool  read_row_data_temp(uint32_t * raw_t)
{
	uint8_t temp[TEMPERATURE__DATA_SIZE] = { 0 };
     bool status = false;

    // row data for temperature calculation read from 0xFA..0XFC
	if(HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, BMP280_REG_PRESS,
			SIZE_OF_REGISTER, temp, sizeof(temp), HAL_MAX_DELAY)
				==HAL_OK)
	{
		*raw_t= (int32_t) ((((int32_t) (temp[3])) << 12) | (((int32_t) (temp[4])) << 4) |
				(((int32_t) (temp[5])) >> 4));
		status = true;
	}

	return status;
}

bool read_temperature(float * temperature)
{
	read_calliberation_data_280();
	bool status = false;
	uint32_t  adc_t;

	//read row data for temperature calculation
	status = read_row_data_temp(&adc_t);

	if(status == true)
	{
		var1 = (((double)adc_t)/MASK1-((double)dig_temp1)/MASK2)*((double)dig_temp2);

		var2 = (((double)adc_t)/MASK3-((double)dig_temp1)/MASK4)*(((double)adc_t)/MASK3-
				(((double)dig_temp1)/MASK4))*((double)dig_temp3);

		t_fine = var1 + var2;

		*temperature = ((var1 + var2)/MASK5)- MASK6;

		*temperature = TEMPERATURE_CELCIUS(*temperature);

		HAL_Delay(100);
	}

	return status;
}

static bool read_row_data_pressure(int32_t *raw_pressure)
{
	uint8_t pressure[PRESSURE_DATA_SIZE] = {0};
	bool status = false;
	//row data for pressure calculation data is read from register 0XF7...0xF9
	if( HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, BMP280_REG_PRESS,
			SIZE_OF_REGISTER, pressure, sizeof(pressure), HAL_MAX_DELAY)
				==HAL_OK)
	{
		*raw_pressure = (int32_t) ((((int32_t) (pressure[0])) << 12) | (((int32_t) (pressure[1])) << 4) |
				(((int32_t) (pressure[2])) >> 4));
		status = true;
	}

	return status;
}

bool read_pressure(double *pressure)
{
	bool status = false;
	int32_t pressure_raw ;
	status =  read_row_data_pressure(&pressure_raw);

	if(status == true)
	{
		var1 = ((double)t_fine/MASKP1)-MASKP2;

		var2 =var1*var1*((double)dig_pressure6)/MASKP3;

		var2=var2+var1*((double)dig_pressure5)*MASKP1;

		var2=(var2/(MASKP1*MASKP1))+(((double)dig_pressure4)*MASKP4);

		var1=(((double)dig_pressure3)*var1*var1/MASKP5 +((double)dig_pressure2)*var1)/MASKP5;

		var1=(MASKP7+var1/MASKP6)*((double)dig_pressure1);

		*pressure = MASKP8-(double)pressure_raw;

		*pressure=(*pressure-(var2/MASKP9))*MASKP10/var1;

		var1=((double)dig_pressure9)*(*pressure)*(*pressure)/MASKP11;

		var2=(*pressure)*((double)dig_pressure8)/MASKP6;

		*pressure=*pressure + (var1+var2+((double)dig_pressure7))/MASKP12;

		*pressure = *pressure/MASKP13;

	}
	return status;
}

bool init_bmp(osrs_temperature_t temp_mode, osrs_pressure_t pressure_mode,
		power_mode_t power_mode, filter_t filter_mode,
		time_stability_t time_stability_mode)
{
	bool status = false;

	status=bmp280_reset();
	HAL_Delay(10);

	if(status==true)
	{
		status = osrs_t_config(temp_mode);
	}

	if(status == true)
	{
		status = osrs_p_config(pressure_mode);
	}

	if(status == true)
	{
		status = mode_config(power_mode);
	}

	if(status == true)
	{
		status = config_filter(filter_mode);
	}

	if(status == true)
	{
		status = config_t_sb(time_stability_mode);
	}

	if(status == true )
	{
		status =  spi3w_disable();
	}

	if(status==true)
	{
		status=read_calliberation_data_280();
	}

	return status;
}