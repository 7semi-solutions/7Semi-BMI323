/**
 * 7Semi BMI323 Arduino Library
 *
 * - Lightweight Arduino wrapper for Bosch BMI323 IMU
 * - Supports I2C and SPI communication interfaces
 * - Provides easy-to-use APIs for accelerometer, gyroscope, FIFO, and interrupts
 *
 * Architecture:
 * - Uses Bosch Sensortec BMI3 Sensor API (separate component)
 * - This library acts as a wrapper / abstraction layer
 *
 * Important Notice:
 * - This library DOES NOT modify Bosch BMI3 driver source
 * - Bosch driver remains under its original license
 * - Only wrapper code is licensed under MIT
 *
 * Bosch BMI3 Sensor API:
 * - Copyright (c) Bosch Sensortec GmbH
 * - Licensed under Bosch Sensortec license terms
 * - Source: https://github.com/boschsensortec
 *
 * 7Semi Contributions:
 * - Arduino-friendly interface layer
 * - Simplified configuration APIs
 * - Data scaling (g, dps, °C)
 * - FIFO parsing helpers
 * - I2C/SPI abstraction
 *
 * License (7Semi wrapper only):
 * - MIT License
 *
 * Disclaimer:
 * - Bosch Sensortec is NOT affiliated with 7Semi
 * - Use Bosch driver according to its original license
 */

#ifndef BMI323_7SEMI_H
#define BMI323_7SEMI_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "bmi3.h"

/**
 * BMI323 Chip ID
 * - Expected value from sensor
 */
#define BMI323_CHIP_ID 0x43

const static uint8_t I2C_BUFFER_LIMIT = 32;


class BMI323_7Semi
{
public:

    BMI323_7Semi();   

    /**
     * Initialize BMI323 over I2C
     *
     * - Configures I2C interface and verifies device connected or not
     * - Initializes Bosch sensor driver
     * - Applies default sensor configuration
     *
     * Parameters:
     * - i2cAddr  → device I2C address
     * - wirePort → I2C interface (default Wire)
     * - i2cSpeed → I2C clock (e.g. 400kHz)
     * - i2cSDA   → optional SDA pin (ESP32 only)
     * - i2cSCL   → optional SCL pin (ESP32 only)
     *
     * Returns:
     * - true  → success
     * - false → communication/init failure
     */
    bool beginI2C(uint8_t i2cAddr = 0x76, TwoWire &wirePort = Wire, uint32_t i2cSpeed = 400000, uint8_t i2cSDA = -1, uint8_t i2cSCL = -1);

    /**
     * Initialize BMI323 over SPI
     *
     * - Configures SPI interface and chip select
     * - Initializes Bosch sensor driver
     * - Applies default sensor configuration
     *
     * Parameters:
     * - cs        → chip select pin
     * - spiPort   → SPI interface
     * - spiSpeed  → SPI clock (e.g. 5MHz)
     * - sck/miso/mosi → optional pins (ESP32 only)
     *
     * Returns:
     * - true  → success
     * - false → init failure
     */
    bool beginSPI(uint8_t cs = 10, SPIClass &spiPort = SPI, uint32_t spiSpeed = 1000000, uint8_t sck = -1, uint8_t miso = -1, uint8_t mosi = -1);


    /**
     * Apply default sensor configuration
     *
     * - Performs soft reset
     * - Verifies chip ID
     * - Configures accelerometer and gyroscope
     *
     * Default setup:
     * - Accel: 100Hz, 2G, normal mode
     * - Gyro : 100Hz, 2000dps, normal mode
     *
     * Returns:
     * - true  → configuration success
     * - false → failure
     */
    bool configureSensor();

    /**
     * Software reset of BMI323 sensor
     *
     * - Sends reset command using Bosch driver
     * - Resets all registers to default state
     * - Clears internal configuration and FIFO
     *
     * Important:
     * - Sensor MUST be reconfigured after reset
     * - All settings (ODR, range, filters) are lost
     * - Allow proper delay for sensor restart
     *
     * Returns:
     * - true  → reset successful
     * - false → reset failed
     */
    bool softReset();

    /**
     * Read BMI323 CHIP ID
     *
     * - Reads chip ID register directly from sensor
     * - Verifies communication and device identity
     *
     * Expected:
     * - 0x43 → BMI323
     *
     * Returns:
     * - CHIP ID value (0x43 if valid)
     * - 0x00 or invalid → communication issue
     */
    uint8_t getChipId();

    /**
     * Configure accelerometer using accel config structure
     *
     * - Applies full accelerometer configuration
     * - Stores range for scaling
     *
     * Returns:
     * - true  → success
     * - false → failure
     */
    bool setAccelConfig(bmi3_accel_config accel_config);

    /**
     * Get current accelerometer configuration
     *
     * - Reads configuration from sensor
     * - Returns full accel config structure
     *
     * Returns:
     * - true  → success
     * - false → failure
     */
    bool getAccelConfig(bmi3_accel_config &accel_config);

    /**
     * Configure accelerometer
     *
     * - Sets ODR, bandwidth, mode, range and averaging
     * - Includes safety validation
     * - Stores range for scaling
     *
     * Returns:
     * - true  → success
     * - false → invalid input or failure
     */
    bool setAccelConfig(uint8_t odr, uint8_t bwp, uint8_t mode, uint8_t range, uint8_t avg);


     /**
     * Read current accelerometer configuration
     *
     * - Fetches active configuration from BMI323
     * - Returns ODR, bandwidth, mode, range and averaging
     *
     * Useful for:
     * - Debugging sensor setup
     * - Verifying configuration after init
     *
     * Returns:
     * - true  → read successful
     * - false → read failed
     */
    bool getAccelConfig(uint8_t &odr, uint8_t &bwp, uint8_t &mode, uint8_t &range, uint8_t &avg);
    
    /**
     * Get current gyroscope configuration
     *
     * - Reads configuration from sensor
     * - Returns full gyro config structure
     *
     * Returns:
     * - true  → success
     * - false → failure
     */
    bool getGyroConfig(bmi3_gyro_config &gyro_config);

    /**
     * Configure gyroscope using gyro config structure
     *
     * - Applies full gyro configuration
     * - Stores range for scaling
     */
    bool setGyroConfig(bmi3_gyro_config gyro_config);

    /**
     * Read current gyroscope configuration
     *
     * - Fetches active gyro configuration from BMI323
     * - Returns ODR, bandwidth, mode, range and averaging
     *
     * Useful for:
     * - Debugging gyro setup
     * - Verifying configuration after initialization
     *
     * Returns:
     * - true  → read successful
     * - false → read failed
     */
    bool setGyroConfig(uint8_t odr, uint8_t bwp, uint8_t mode, uint8_t range, uint8_t avg);
    
     /**
     * Read current accelerometer configuration
     *
     * - Fetches active configuration from BMI323
     * - Returns ODR, bandwidth, mode, range and averaging
     *
     * Useful for:
     * - Debugging sensor setup
     * - Verifying configuration after init
     *
     * Returns:
     * - true  → read successful
     * - false → read failed
     */
    bool getGyroConfig(uint8_t &odr, uint8_t &bwp, uint8_t &mode, uint8_t &range, uint8_t &avg);

    /**
     * Read raw accelerometer data
     *
     * - Fetches accelerometer data from BMI323
     * - Returns raw acceleration values 
     *
     * Notes:
     * - Sensor must be enabled before calling
     * - Values are NOT scaled (use readAccel() for g conversion)
     *
     * Returns:
     * - true  → read successful
     * - false → read failed or invalid data
     */
    bool readAccelRaw(int16_t &x, int16_t &y, int16_t &z);

    /**
     * Read raw gyroscope data
     *
     * - Fetches gyro data from BMI323
     * - Returns raw angular velocity (LSB)
     *
     * Notes:
     * - Sensor must be enabled before calling
     * - Values are NOT scaled (use readGyro() for DPS)
     *
     * Returns:
     * - true  → read successful
     * - false → read failed
     */
    bool readGyroRaw(int16_t &x, int16_t &y, int16_t &z);

    /**
     * Read accelerometer data (in g)
     *
     * - Reads raw accel data
     * - Converts to physical units (g)
     *
     * Notes:
     * - Scaling depends on configured range
     * - Range must be set before calling
     *
     * Returns:
     * - true  → success
     * - false → read failed
     */
    bool readAccel(float &x, float &y, float &z);

    /**
     * Read gyroscope data (in degrees per second)
     *
     * - Reads raw gyro data
     * - Converts to DPS
     *
     * Notes:
     * - Scaling depends on configured range
     * - Range must be set before calling
     *
     * Returns:
     * - true  → success
     * - false → read failed
     */
    bool readGyro(float &x, float &y, float &z);

    /**
    * Read raw temperature data
    *
    * - Fetches temperature data from BMI323
    * - Returns raw temperature 
    *
    * Notes:
    * - Temperature is signed (int16_t)
    * - Use getTemperature() for °C conversion
    *
    * Returns:
    * - true  → success
    * - false → read failed or invalid data
    */
    bool getTemperatureRaw(int16_t &raw_temperature);

    /**
     * Read temperature in degree Celsius
     *
     * - Converts raw sensor value to °C
     * - Uses Bosch conversion formula
     *
     * Formula:
     * - T(°C) = (raw / 512) + 23
     *
     * Returns:
     * - true  → success
     * - false → read failed
     */
    bool getTemperature(float &temperature);

    /**
     * Read sensor internal timestamp
     *
     * - Returns sensor time counter
     * - Useful for synchronization and FIFO alignment
     *
     * Notes:
     * - Units depend on ODR / internal clock
     * - Typically used for relative timing
     *
     * Returns:
     * - true  → success
     * - false → read failed
     */
    bool getSensorTime(uint32_t &sensor_time);

    /**
     * Set FIFO configuration
     *
     * - Configures FIFO behavior and enabled data streams
     * - Controls what data is stored inside FIFO
     * 
     * Parameters:
     * - fifo_config → bitmask configuration (Bosch defined)
     * * - BMI3_FIFO_STOP_ON_FULL
     * * - BMI3_FIFO_TIME_EN
     * * - BMI3_FIFO_ACC_EN
     * * - BMI3_FIFO_GYR_EN
     * * - BMI3_FIFO_TEMP_EN
     * * - BMI3_FIFO_ALL_EN
     *
     * Notes:
     * - Use Bosch macros to enable accel, gyro, temp, etc.
     *
     * Returns:
     * - true  → configuration successful
     * - false → configuration failed
     */
    bool setFifoConfig(uint16_t fifo_config, bool enable);

    /**
     * Get FIFO configuration
     *
     * - Reads current FIFO configuration from sensor
     * - Returns enabled data sources and settings
     *
     * Parameters:
     * - fifo_config → output bitmask configuration
     * * - BMI3_FIFO_STOP_ON_FULL
     * * - BMI3_FIFO_TIME_EN
     * * - BMI3_FIFO_ACC_EN
     * * - BMI3_FIFO_GYR_EN
     * * - BMI3_FIFO_TEMP_EN
     * * - BMI3_FIFO_ALL_EN
     * 
     * Returns:
     * - true  → read successful
     * - false → read failed
     */
    bool getFifoConfig(uint16_t &fifo_config);

    /**
     * Get FIFO data length
     *
     * - Reads number of bytes currently stored in FIFO
     *
     * Parameters:
     * - fifo_length → number of bytes available in FIFO
     *
     * Notes:
     * - Use this before reading FIFO to avoid overflow
     *
     * Returns:
     * - true  → read successful
     * - false → read failed
     */
    bool getFifoLength(uint16_t &fifo_length);

    /**
     * Set FIFO watermark level
     *
     * - Defines threshold for FIFO interrupt trigger
     *
     * Parameters:
     * - fifo_watermark → number of bytes before interrupt triggers
     *
     * Notes:
     * - Lower value → faster response
     * - Higher value → fewer interrupts
     *
     * Returns:
     * - true  → configuration successful
     * - false → configuration failed
     */
    bool setFifoWatermark(uint16_t fifo_watermark);

    /**
     * Get FIFO watermark level
     *
     * - Reads configured FIFO watermark threshold
     *
     * Parameters:
     * - fifo_watermark → current watermark value
     *
     * Returns:
     * - true  → read successful
     * - false → read failed
     */
    bool getFifoWatermark(uint16_t &fifo_watermark);

    /**
     * Read FIFO data
     *
     * - Reads raw FIFO buffer from sensor
     * - Stores data into user-provided buffer
     *
     * Parameters:
     * - fifo → FIFO frame structure (Bosch API)
     *
     * Notes:
     * - fifo.data must point to a valid buffer
     * - fifo.length must be set before calling
     * - fifo.watermark must be set before calling
     *
     * Example:
     * - fifo.data = buffer;
     * - fifo.length = sizeof(buffer);
     *
     * Returns:
     * - true  → read successful
     * - false → read failed
     */
    bool getFifoData(struct bmi3_fifo_frame &fifo);

    /**
     * Configure interrupt pin (INT1 / INT2)
     *
     * - Sets polarity and output mode
     *
     * Parameters:
     * - pin         → BMI3_INT1 or BMI3_INT2
     * - active_high → true  = active high
     *                 false = active low
     * - open_drain  → true  = open drain
     *                 false = push-pull
     *
     * Returns:
     * - true  → configuration successful
     * - false → invalid input or failure
     */
    bool setInterruptConfig(uint8_t pin, bool active_high, bool open_drain);

    /**
     * Read interrupt pin configuration
     *
     * - Returns polarity and output mode of selected pin
     *
     * Parameters:
     * - pin         → BMI3_INT1 or BMI3_INT2
     * - active_high → output polarity
     * - open_drain  → output mode
     *
     * Returns:
     * - true  → read successful
     * - false → failure
     */
    bool getInterruptConfig(uint8_t pin, bool &active_high, bool &open_drain);

    /**
     * Enable or disable interrupt output
     *
     * - Controls pin output enable
     * - Configures latch behavior
     *
     * Parameters:
     * - pin      → BMI3_INT1 or BMI3_INT2
     * - enable   → true = enable output
     * - latching → true = latched interrupt
     *
     * Returns:
     * - true  → success
     * - false → failure
     */
    bool enableInterrupt(uint8_t pin, bool enable, bool latching);

    /**
     * Read INT1 interrupt status
     *
     * - Returns interrupt flags from INT1
     *
     * Returns:
     * - true  → read successful
     * - false → read failed
     */
    bool getINT1(uint16_t &status);

    /**
     * Read INT2 interrupt status
     *
     * - Returns interrupt flags from INT2
     *
     * Returns:
     * - true  → read successful
     * - false → read failed
     */
    bool getINT2(uint16_t &status);

    /**
     * Perform BMI323 self-test
     *
     * - Runs internal self-test for accel / gyro
     * - Uses Bosch API self-test routine
     * - Checks both API status and test result flags
     *
     * Notes:
     * - Sensor should be stationary
     * - Run after initialization
     *
     * Returns:
     * - true  → self-test passed
     * - false → API failure or test failed
     */
    bool selfTest(uint8_t st_selection, struct bmi3_st_result &st_result_status);

    /**
     * Set axis remapping configuration
     *
     * - Remaps sensor X, Y, Z axes
     * - Used when sensor is mounted in different orientation
     *
     * Parameters:
     * - remap → struct containing axis mapping and sign
     *
     * Example:
     * - Swap X and Y
     * - Invert Z axis
     *
     * Notes:
     * - Must be called after initialization
     * - Affects both accel and gyro output
     *
     * Returns:
     * - true  → success
     * - false → failure
     */
    bool setRemapAxis(const bmi3_axes_remap &remap);

    /**
     * Get axis remapping configuration
     *
     * - Reads current axis remap settings
     * - Used for board orientation correction
     *
     * Returns:
     * - true  → read successful
     * - false → failure
     */
    bool setRemapAxis(struct bmi3_axes_remap &remapped_axis);


    /**
     * Get axis remapping configuration
     *
     * - Reads current axis remap settings
     * - Used for board orientation correction
     *
     * Returns:
     * - true  → read successful
     * - false → failure
     */
    bool getRemapAxis(struct bmi3_axes_remap &remapped_axis);

    /**
     * Perform gyroscope self-calibration
     *
     * - Runs gyro self-calibration (SC)
     * - Optionally applies correction
     *
     * Parameters:
     * - mode             → calibration mode (Bosch defined)
     * - apply_correction → true = apply offset correction
     *
     * Notes:
     * - Sensor must be stationary
     * - No motion during calibration
     *
     * Returns:
     * - true  → calibration successful
     * - false → failed or invalid result
     */
    bool calibrateGyro(uint8_t mode, bool apply_correction);

    
    /**
     * Perform accelerometer FOC calibration
     *
     * - Only ONE axis must be selected
     * - Sensor must be stationary
     * - Axis must align with gravity (±1g)
     *
     * Parameters:
     * - x, y, z → axis selection (only one must be true)
     * - negative → true  = -1g
     *              false = +1g
     *
     * Returns:
     * - true  → success
     * - false → invalid input or failure
     */
    bool calibrateAccelFOC(bool x, bool y, bool z, bool negative);


    /**
     * Get sensor status register
     *
     * - Reads sensor status flags
     * - Useful for debugging and health monitoring
     *
     * Returns:
     * - true  → success
     * - false → failure
     */
    bool getSensorStatus(uint16_t &sensor_status);

    /**
     * Get last API status
     *
     * - Returns last Bosch driver status code
     * - Useful for debugging failures
     *
     * Returns:
     * - status code (BMI3_OK or error)
     */
    int8_t getLastStatus();

private:

    struct bmi3_dev bmi3;

    /* Interface */
    TwoWire *i2c;
    uint8_t i2c_address;

    SPIClass *spi;
    uint8_t cs_pin;
    uint32_t spi_speed;

    /* Scaling */
    uint8_t accel_range = BMI3_ACC_RANGE_2G;
    uint8_t gyro_range = BMI3_GYR_RANGE_2000DPS;

    int8_t status = BMI3_OK;

    static int8_t i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr);
    static int8_t i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr);

    static int8_t spi_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr);
    static int8_t spi_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr);

    static void delay_us(uint32_t period, void *intf_ptr);
};

#endif