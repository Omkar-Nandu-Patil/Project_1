
## Authors

- [@Omkar Nandu Patil](https://github.com/Omkar-Nandu-Patil)


# Interfacing the BMP280 sensor with STM32F103C8T6

This project involves measuring data from the BMP280 sensor, which is a versatile environmental sensor capable of measuring temperature, pressure, and humidity. The BMP280 sensor is often used in various applications, including weather monitoring, indoor climate control, and more. This README will provide an overview of the project, its purpose, how to set it up, and any additional information that may be helpful for users and developers




## Documentation

[DataSheet of BMP 280 sensor ](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)


## Project Setup Requirements

To set up and use this project, you will need the following components and tools:

- STM32CubeIDE (Integrated Development Environment for STM32 microcontrollers)
- STM32CubeMX (Configuration and initialization tool for STM32 microcontrollers)
- BMP280 Sensor
- STM32F103C8T6 microcontroller
- Appropriate wiring and connections
## Connection 

#### I2C Communication Diagram

To establish I2C communication between the STM32F103C8T6 microcontroller and the BMP280 sensor, you can refer to the following diagram:


STM32F103C8T6    <---------------------------->              BMP280 

PB7 (SDA)         ---------◄--------►    SDA (Data) 

 PB6 (SCL)        ---------◄--------►   SCL (Clock) 

 VCC (Power) ----------------► VIN (Power) 

GND (Ground) --------------► GND (Ground) 


The STM32F103C8T6 microcontroller is connected to the BMP280 Sensor.
- PB7 (SDA) on the microcontroller is connected to the SDA (Data) pin on the BMP280.
- PB6 (SCL) on the microcontroller is connected to the SCL (Clock) pin on the BMP280.
- VCC (Power) from the microcontroller is connected to VIN (Power) on the BMP280.
- GND (Ground) from the microcontroller is connected to GND (Ground) on the BMP280.

This setup enables I2C communication and power supply between the microcontroller and the BMP280 sensor.



## BMP280 Sensor API Functions

This project provides the following API functions to initialize and interact with the BMP280 sensor:

#### Initialize BMP280 Sensor

```c
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
```
#### Read Temperature from BMP280 Sensor

```c
/**
 * @brief read raw temperature value from register 0xFA....0XFC
 * Using raw value and formula, temperature is calculated
 * @param temperature [OUT] calculated temperature @ref float *temperature
 * @retval true on success
 */
bool read_temperature(float *temperature);
```
#### Read Pressure from BMP280 Sensor

```c
/**
 * @brief read raw pressure value from register 0xF7....0XF9
 * Using raw value and formula, pressure is calculated
 * @param pressure [OUT] calculated pressure  @ref float *pressure
 * @retval true on success
 */
bool read_pressure(double *pressure);
```

## Build and Flash
- Compile your code and build the project.
- Flash the compiled code onto your STM32F103C8T6 microcontroller.



##### This README.md includes detailed steps for setting up and using the BMP280 sensor with your STM32F103C8T6 microcontroller, along with the API functions for reference.
