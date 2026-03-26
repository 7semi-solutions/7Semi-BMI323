#include "7Semi_BMI323.h"

BMI323_7Semi::BMI323_7Semi() {}

bool BMI323_7Semi::beginI2C(
    uint8_t i2cAddr,
    TwoWire &wirePort,
    uint32_t i2cSpeed,
    uint8_t i2cSDA,
    uint8_t i2cSCL)
{
    i2c = &wirePort;
    i2c_address = i2cAddr;

#if defined(ESP32)
    /**
     * - Initialize I2C with custom pins if provided
     * - Otherwise use default pins
     */
    if ((i2cSDA != 255u) && (i2cSCL != 255u))
        i2c->begin(i2cSDA, i2cSCL);
    else
        i2c->begin();
#else
    (void)i2cSDA;
    (void)i2cSCL;
    i2c->begin();
#endif

    // Set I2C clock speed
    i2c->setClock(i2cSpeed);

    /**
     * - Verify device presence on I2C bus
     * - Prevents false initialization
     */
    i2c->beginTransmission(i2c_address);
    if (i2c->endTransmission() != 0)
        return false;

    /**
     * - Configure Bosch driver interface
     * - intf_ptr used to access class instance inside callbacks
     */
    bmi3.intf = BMI3_I2C_INTF;
    bmi3.read = i2c_read;
    bmi3.write = i2c_write;
    bmi3.delay_us = delay_us;
    bmi3.intf_ptr = this;

    // Initialize sensor
    if (bmi3_init(&bmi3) != BMI3_OK)
        return false;

    return configureSensor();
}

bool BMI323_7Semi::beginSPI(
    uint8_t cs,
    SPIClass &spiPort,
    uint32_t spiSpeed,
    uint8_t sck,
    uint8_t miso,
    uint8_t mosi)
{
    spi = &spiPort;
    cs_pin = cs;
    spi_speed = spiSpeed;

    // Configure chip select
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

#if defined(ESP32)
    /**
     * - Initialize SPI with custom pins if provided
     * - Otherwise use default SPI pins
     */
    if ((sck != 255u) && (miso != 255u) && (mosi != 255u))
        spi->begin(sck, miso, mosi, cs_pin);
    else
        spi->begin();
#else
    (void)sck;
    (void)miso;
    (void)mosi;
    spi->begin();
#endif

    /**
     * - Configure Bosch driver interface
     */
    bmi3.intf = BMI3_SPI_INTF;
    bmi3.read = spi_read;
    bmi3.write = spi_write;
    bmi3.delay_us = delay_us;
    bmi3.intf_ptr = this;

    if (bmi3_init(&bmi3) != BMI3_OK)
        return false;

    return configureSensor();
}

bool BMI323_7Semi::configureSensor()
{
    // Reset sensor
    if (!softReset())
        return false;

    delay(50);

    /**
     * - Verify correct device
     * - Prevents wrong chip / wiring issue
     */
    uint8_t chip_id = getChipId();
    if (chip_id != BMI323_CHIP_ID)
        return false;

    /**
     * Configure accelerometer
     * - ODR: 100Hz
     * - Bandwidth: ODR/4
     * - Range: ±2G
     * - Mode: normal
     */
    if (!setAccelConfig(
            BMI3_ACC_ODR_100HZ,
            BMI3_ACC_BW_ODR_QUARTER,
            BMI3_ACC_MODE_NORMAL,
            BMI3_ACC_RANGE_2G,
            BMI3_ACC_AVG1))
        return false;

    /**
     * Configure gyroscope
     * - ODR: 100Hz
     * - Range: ±2000 dps
     * - Mode: normal
     */
    if (!setGyroConfig(
            BMI3_GYR_ODR_100HZ,
            BMI3_GYR_BW_ODR_QUARTER,
            BMI3_GYR_MODE_NORMAL,
            BMI3_GYR_RANGE_2000DPS,
            BMI3_GYR_AVG1))
        return false;

    return true;
}

bool BMI323_7Semi::softReset()
{
    /**
     * Execute Bosch soft reset command
     */
    status = bmi3_soft_reset(&bmi3);

    if (status != BMI3_OK)
        return false;

    /**
     * Delay for sensor reboot
     *
     * - Required for stable operation after reset
     * - Recommended > 10ms (safe: 50ms)
     */
    delay(50);

    return true;
}

uint8_t BMI323_7Semi::getChipId()
{
    uint8_t chip_id = 0;

    /**
     * Read CHIP ID register
     */
    status = bmi3_get_regs(BMI3_REG_CHIP_ID, &chip_id, 1, &bmi3);

    if (status != BMI3_OK)
        return 0;

    return chip_id;
}

bool BMI323_7Semi::setAccelConfig(bmi3_accel_config accel_config)
{
    struct bmi3_sens_config config = {0};

    // Set correct sensor type
    config.type = BMI3_ACCEL;

    // Assign accel config
    config.cfg.acc = accel_config;

    // Apply configuration
    status = bmi3_set_sensor_config(&config, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    // Store range for scaling
    accel_range = accel_config.range;

    return true;
}

bool BMI323_7Semi::getAccelConfig(bmi3_accel_config &accel_config)
{
    struct bmi3_sens_config config = {0};

    // Set correct sensor type
    config.type = BMI3_ACCEL;

    // Read configurations from sensor
    status = bmi3_get_sensor_config(&config, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    // Copy accel config
    accel_config = config.cfg.acc;

    return true;
}

bool BMI323_7Semi::setAccelConfig(uint8_t odr, uint8_t bwp, uint8_t mode, uint8_t range, uint8_t avg_num)
{
    struct bmi3_sens_config config = {0};

    /**
     * Validate inputs
     */
    if (odr > BMI3_ACC_ODR_6400HZ ||
        bwp > BMI3_ACC_BW_ODR_QUARTER ||
        range > BMI3_ACC_RANGE_16G ||
        avg_num > BMI3_ACC_AVG64 ||
        mode > BMI3_ACC_MODE_HIGH_PERF)
    {
        return false; // Invalid inputs
    }

    // Select config sensor
    config.type = BMI3_ACCEL;

    /**
     * Get current config (preserve other fields)
     */
    status = bmi3_get_sensor_config(&config, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    /**
     * Apply new settings
     */
    config.cfg.acc.odr = odr;
    config.cfg.acc.range = range;
    config.cfg.acc.bwp = bwp;
    config.cfg.acc.avg_num = avg_num;
    config.cfg.acc.acc_mode = mode;

    /**
     * Write config to sensor
     */
    status = bmi3_set_sensor_config(&config, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    /**
     * Store range for scaling
     */
    accel_range = range;

    return true;
}

bool BMI323_7Semi::getAccelConfig(uint8_t &odr, uint8_t &bwp, uint8_t &mode, uint8_t &range, uint8_t &avg_num)
{
    struct bmi3_sens_config config = {0};

    config.type = BMI3_ACCEL;

    // Read current configuration from sensor
    status = bmi3_get_sensor_config(&config, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    // Extract values from sensor config
    odr = config.cfg.acc.odr;
    range = config.cfg.acc.range;
    bwp = config.cfg.acc.bwp;
    avg_num = config.cfg.acc.avg_num;
    mode = config.cfg.acc.acc_mode;

    return true;
}

bool BMI323_7Semi::getGyroConfig(bmi3_gyro_config &gyro_config)
{
    struct bmi3_sens_config config = {0};

    // Select sensor type to read config
    config.type = BMI3_GYRO;

    // Read from sensor
    status = bmi3_get_sensor_config(&config, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    // Copy gyro config
    gyro_config = config.cfg.gyr;

    return true;
}

bool BMI323_7Semi::setGyroConfig(bmi3_gyro_config gyro_config)
{
    struct bmi3_sens_config config = {0};

    // Set correct sensor type
    config.type = BMI3_GYRO;

    // Assign gyro config (NOT accel)
    config.cfg.gyr = gyro_config;

    // Apply configuration
    status = bmi3_set_sensor_config(&config, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    // Store range
    gyro_range = gyro_config.range;

    return true;
}

bool BMI323_7Semi::getGyroConfig(uint8_t &odr, uint8_t &bwp, uint8_t &mode, uint8_t &range, uint8_t &avg_num)
{
    struct bmi3_sens_config config = {0};

    config.type = BMI3_GYRO;

    // Read current configuration
    status = bmi3_get_sensor_config(&config, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    // Extract values
    odr = config.cfg.gyr.odr;
    range = config.cfg.gyr.range;
    bwp = config.cfg.gyr.bwp;
    avg_num = config.cfg.gyr.avg_num;
    mode = config.cfg.gyr.gyr_mode;

    return true;
}

bool BMI323_7Semi::setGyroConfig(uint8_t odr, uint8_t bwp, uint8_t mode, uint8_t range, uint8_t avg_num)
{
    struct bmi3_sens_config cfg = {0};

    /**
     * Validate inputs (use Bosch limits)
     */
    if (odr > BMI3_GYR_ODR_6400HZ ||
        bwp > BMI3_GYR_BW_ODR_QUARTER ||
        range > BMI3_GYR_RANGE_2000DPS ||
        avg_num > BMI3_GYR_AVG64 ||
        mode > BMI3_GYR_MODE_HIGH_PERF)
    {
        return false;
    }

    cfg.type = BMI3_GYRO;

    /**
     * Get current config (preserve other fields)
     */
    status = bmi3_get_sensor_config(&cfg, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    /**
     * Apply new settings
     */
    cfg.cfg.gyr.odr = odr;
    cfg.cfg.gyr.range = range;
    cfg.cfg.gyr.bwp = bwp;
    cfg.cfg.gyr.avg_num = avg_num;
    cfg.cfg.gyr.gyr_mode = mode;

    /**
     * Write config to sensor
     */
    status = bmi3_set_sensor_config(&cfg, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    /**
     * Store range for scaling
     */
    gyro_range = range;

    return true;
}

bool BMI323_7Semi::readAccelRaw(int16_t &x, int16_t &y, int16_t &z)
{
    struct bmi3_sensor_data data = {0};

    /**
     * Request only accelerometer data
     */
    data.type = BMI3_ACCEL;

    status = bmi3_get_sensor_data(&data, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    /**
     * Extract raw data
     */
    x = data.sens_data.acc.x;
    y = data.sens_data.acc.y;
    z = data.sens_data.acc.z;

    /**
     * Validate data (important for real hardware)
     *
     * - Reject all-zero frames
     * - Reject invalid overflow value
     */
    if ((x == 0 && y == 0 && z == 0) || x == -32768)
        return false;

    return true;
}

bool BMI323_7Semi::readGyroRaw(int16_t &x, int16_t &y, int16_t &z)
{
    struct bmi3_sensor_data data = {0};

    /**
     * Request only gyro data
     */
    data.type = BMI3_GYRO;

    status = bmi3_get_sensor_data(&data, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    /**
     * Extract data
     */
    x = data.sens_data.gyr.x;
    y = data.sens_data.gyr.y;
    z = data.sens_data.gyr.z;

    /**
     * Basic validation
     */
    if ((x == 0 && y == 0 && z == 0) || x == -32768)
        return false;

    return true;
}

bool BMI323_7Semi::readAccel(float &x, float &y, float &z)
{
    int16_t rx, ry, rz;

    if (!readAccelRaw(rx, ry, rz))
        return false;

    /**
     * Select scaling factor based on range
     *
     * - Converts raw  → g
     */
    float scale;

    switch (accel_range)
    {
    case BMI3_ACC_RANGE_2G:
        scale = 16384.0f;
        break;
    case BMI3_ACC_RANGE_4G:
        scale = 8192.0f;
        break;
    case BMI3_ACC_RANGE_8G:
        scale = 4096.0f;
        break;
    case BMI3_ACC_RANGE_16G:
        scale = 2048.0f;
        break;
    default:
        scale = 16384.0f;
    }

    /**
     * Convert to g
     */
    x = rx / scale;
    y = ry / scale;
    z = rz / scale;

    return true;
}

bool BMI323_7Semi::readGyro(float &x, float &y, float &z)
{
    int16_t rx, ry, rz;

    if (!readGyroRaw(rx, ry, rz))
        return false;

    /**
     * Select scaling factor
     *
     * - Converts raw LSB → DPS
     */
    float scale;

    switch (gyro_range)
    {
    case BMI3_GYR_RANGE_125DPS:
        scale = 262.4f;
        break;
    case BMI3_GYR_RANGE_250DPS:
        scale = 131.2f;
        break;
    case BMI3_GYR_RANGE_500DPS:
        scale = 65.6f;
        break;
    case BMI3_GYR_RANGE_1000DPS:
        scale = 32.8f;
        break;
    case BMI3_GYR_RANGE_2000DPS:
        scale = 16.4f;
        break;
    default:
        scale = 16.4f;
    }

    /**
     * Convert to DPS
     */
    x = rx / scale;
    y = ry / scale;
    z = rz / scale;

    return true;
}

bool BMI323_7Semi::getTemperatureRaw(int16_t &raw_temperature)
{
    struct bmi3_sensor_data data = {0};

    /**
     * Request temperature data
     */
    data.type = BMI3_TEMP;

    status = bmi3_get_sensor_data(&data, 1, &bmi3);
    if (status != BMI3_OK)
        return false;

    /**
     * Extract raw temperature
     */
    raw_temperature = (int16_t)data.sens_data.temp.temp_data;

    /**
     * Basic validation
     *
     * - Reject invalid extreme value
     */
    if (raw_temperature == -32768)
        return false;

    return true;
}

bool BMI323_7Semi::getTemperature(float &temperature)
{
    int16_t raw_temperature;

    if (!getTemperatureRaw(raw_temperature))
        return false;

    /**
     * Convert to Celsius
     */
    temperature = (raw_temperature / 512.0f) + 23.0f;

    return true;
}

bool BMI323_7Semi::getSensorTime(uint32_t &sensor_time)
{
    // Read sensor time
    status = bmi3_get_sensor_time(&sensor_time, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::setFifoConfig(uint16_t fifo_config, bool enable)
{
    status = bmi3_set_fifo_config(fifo_config, enable, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::getFifoConfig(uint16_t &fifo_config)
{
    status = bmi3_get_fifo_config(&fifo_config, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::getFifoLength(uint16_t &fifo_length)
{
    status = bmi3_get_fifo_length(&fifo_length, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::setFifoWatermark(uint16_t fifo_watermark)
{
    status = bmi3_set_fifo_wm(fifo_watermark, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::getFifoWatermark(uint16_t &fifo_watermark)
{
    status = bmi3_get_fifo_wm(&fifo_watermark, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::getFifoData(struct bmi3_fifo_frame &fifo)
{
    uint16_t fifo_length;
    uint16_t fifo_watermark = 0;

    /*
     * Validate buffer
     */
    if (fifo.data == nullptr)
        return false;
    if (!getFifoLength(fifo_length))
        return false;

    if (fifo_length == 0)
        return false;

    // Clamp to user buffer size
    if (fifo.length == 0)
    {
        // Read watermark to set fifo length
        status = bmi3_get_fifo_wm(&fifo_watermark, &bmi3);
        if (status != BMI3_OK)
            return false;

        if (fifo_watermark == 0)
            fifo_watermark = 150;
        if (fifo_watermark > fifo_length)
            fifo.length = fifo_length;
        else
            fifo.length = fifo_watermark;
        fifo.wm_lvl = fifo_watermark;
    }

    // Read FIFO data
    status = bmi3_read_fifo_data(&fifo, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}

/**
 * Parse FIFO into struct array
 *
 * Parameters:
 * - data         → raw FIFO buffer
 * - length       → number of bytes
 * - out          → output struct array
 * - max_frames   → max structs user allocated
 *
 * Returns:
 * - number of valid frames parsed
 */
// uint16_t parseFIFO_ALL(const uint8_t *data,
//                        uint16_t length,
//                        bmi3_fifo_data *fifi_data,
//                        uint16_t max_frames)
// {
//     const uint8_t FRAME_SIZE = 12;

//     if (length < FRAME_SIZE)
//         return 0;

//     uint16_t total_frames = length / FRAME_SIZE;
//     uint16_t valid_count = 0;

//     for (uint16_t i = 0; i < total_frames; i++)
//     {
//         if (valid_count >= max_frames)
//             break;

//         uint16_t idx = i * FRAME_SIZE;

//         int16_t ax = (int16_t)(data[idx + 1] << 8 | data[idx + 0]);
//         int16_t ay = (int16_t)(data[idx + 3] << 8 | data[idx + 2]);
//         int16_t az = (int16_t)(data[idx + 5] << 8 | data[idx + 4]);

//         int16_t gx = (int16_t)(data[idx + 7] << 8 | data[idx + 6]);
//         int16_t gy = (int16_t)(data[idx + 9] << 8 | data[idx + 8]);
//         int16_t gz = (int16_t)(data[idx + 11] << 8 | data[idx + 10]);

//         int16_t gz = (int16_t)(data[idx + 11] << 8 | data[idx + 10]);

//         //  Filter invalid frames
//         if ((ax == 0 && ay == 0 && az == 0 &&
//              gx == 0 && gy == 0 && gz == 0) ||
//             ax == -32768)
//         {
//             continue;
//         }

//         // Store in struct
//         fifi_data[valid_count].ax = ax;
//         fifi_data[valid_count].ay = ay;
//         fifi_data[valid_count].az = az;

//         fifi_data[valid_count].gx = gx;
//         fifi_data[valid_count].gy = gy;
//         fifi_data[valid_count].gz = gz;

//         if ((ax == 0 && ay == 0 && az == 0 &&
//              gx == 0 && gy == 0 && gz == 0) ||
//             ax == -32768)
//         {
//             continue;
//         }

//         valid_count++;
//     }

//     return valid_count;
// }


bool BMI323_7Semi::setInterruptConfig(uint8_t pin, bool active_high, bool open_drain)
{
    struct bmi3_int_pin_config int_cfg = {0};

    /**
     * Validate pin
     */
    if (pin != BMI3_INT1 && pin != BMI3_INT2)
        return false;

    /**
     * Read current configuration
     */
    status = bmi3_get_int_pin_config(&int_cfg, &bmi3);
    if (status != BMI3_OK)
        return false;

    uint8_t idx = (pin == BMI3_INT1) ? 0 : 1;

    /**
     * Apply configuration
     */
    int_cfg.pin_cfg[idx].lvl = active_high ? BMI3_INT_ACTIVE_HIGH : BMI3_INT_ACTIVE_LOW;
    int_cfg.pin_cfg[idx].od  = open_drain ? BMI3_INT_OPEN_DRAIN : BMI3_INT_PUSH_PULL;

    /**
     * Write back to sensor
     */
    status = bmi3_set_int_pin_config(&int_cfg, &bmi3);
    if (status != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::getInterruptConfig(uint8_t pin, bool &active_high, bool &open_drain)
{
    struct bmi3_int_pin_config int_cfg = {0};

    /**
     * Validate pin
     */
    if (pin != BMI3_INT1 && pin != BMI3_INT2)
        return false;

    /**
     * Read configuration
     */
    status = bmi3_get_int_pin_config(&int_cfg, &bmi3);
    if (status != BMI3_OK)
        return false;

    uint8_t idx = (pin == BMI3_INT1) ? 0 : 1;

    /**
     * Extract values
     */
    active_high = (int_cfg.pin_cfg[idx].lvl == BMI3_INT_ACTIVE_HIGH);
    open_drain  = (int_cfg.pin_cfg[idx].od  == BMI3_INT_OPEN_DRAIN);

    return true;
}


bool BMI323_7Semi::enableInterrupt(uint8_t pin, bool enable, bool latching)
{
    struct bmi3_int_pin_config int_cfg = {0};

    /**
     * Validate pin
     */
    if (pin != BMI3_INT1 && pin != BMI3_INT2)
        return false;

    /**
     * Read current configuration
     */
    status = bmi3_get_int_pin_config(&int_cfg, &bmi3);
    if (status != BMI3_OK)
        return false;

    uint8_t idx = (pin == BMI3_INT1) ? 0 : 1;

    /**
     * Apply configuration
     */
    int_cfg.pin_cfg[idx].output_en = enable;
    int_cfg.int_latch = latching;

    /**
     * Write back to sensor
     */
    status = bmi3_set_int_pin_config(&int_cfg, &bmi3);
    if (status != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::getINT1(uint16_t &int_status)
{
    int8_t rslt = bmi3_get_int1_status(&int_status, &bmi3);

    if (rslt != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::getINT2(uint16_t &int_status)
{
    int8_t rslt = bmi3_get_int2_status(&int_status, &bmi3);

    if (rslt != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::selfTest(uint8_t st_selection,
                            struct bmi3_st_result &st_result_status)
{
    /**
     * Execute self-test
     */
    status = bmi3_perform_self_test(st_selection,
                                   &st_result_status,
                                   &bmi3);

    if (status != BMI3_OK)
        return false;

    /**
     * Validate result flags
     * - Any non-zero error indicates failure
     */
    if (!st_result_status.self_test_rslt & BMI3_ST_RESULT_MASK)
        return false;

    return true;
}

bool BMI323_7Semi::setRemapAxis(const bmi3_axes_remap &remap)
{
    /**
     * Apply remap configuration using Bosch API
     */
    status = bmi3_set_remap_axes(remap, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::getRemapAxis(bmi3_axes_remap &remapped_axis)
{
    status = bmi3_get_remap_axes(&remapped_axis, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}

bool BMI323_7Semi::calibrateGyro(uint8_t mode, bool apply_correction)
{
    bmi3_self_calib_rslt result = {0};

    /**
     * Run gyro self-calibration
     */
    status = bmi3_perform_gyro_sc(mode,
                                 apply_correction,
                                 &result,
                                 &bmi3);

    if (status != BMI3_OK)
        return false;

    /**
     * Check calibration result
     * - Extract result bits safely
     */
    uint8_t sc_result = (result.sc_error_status & BMI3_GYRO_SC_RESULT_MASK);

    if (sc_result != 0)
        return false;

    return true;
}

bool BMI323_7Semi::calibrateAccelFOC(bool x, bool y, bool z, bool negative)
{
    bmi3_accel_foc_g_value foc = {0};

    uint8_t axis_count = 0;

    /**
     * Select axis
     */
    if (x) { foc.x = 1; axis_count++; }
    if (y) { foc.y = 1; axis_count++; }
    if (z) { foc.z = 1; axis_count++; }

    /**
     * Validate: only ONE axis allowed
     */
    if (axis_count != 1)
        return false;

    /**
     * Set direction (+g / -g)
     */
    foc.sign = negative ? 1 : 0;

    /**
     * Execute FOC calibration
     */
    status = bmi3_perform_accel_foc(&foc, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}


bool BMI323_7Semi::getSensorStatus(uint16_t &sensor_status)
{
    status = bmi3_get_sensor_status(&sensor_status, &bmi3);

    if (status != BMI3_OK)
        return false;

    return true;
}

int8_t BMI323_7Semi::getLastStatus()
{
    return status;
}

int8_t BMI323_7Semi::i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr)
{
    BMI323_7Semi *bus = (BMI323_7Semi *)intf_ptr;

    while (len > 0) // read till length become 0
    {
        // calculate request data length at once 
        // Ardino I2C maximum buffer is only 32 bytes, you can changeas per your controller support 
        uint8_t request = (len > I2C_BUFFER_LIMIT) ? I2C_BUFFER_LIMIT : len;

        bus->i2c->beginTransmission(bus->i2c_address);
        // Set register address 
        bus->i2c->write(reg);

        if (bus->i2c->endTransmission(false) != 0)
            return BMI3_E_COM_FAIL;
        
        // Request data to read
        uint8_t received = bus->i2c->requestFrom(bus->i2c_address, request);

        if (received != request)
            return BMI3_E_COM_FAIL;

        //Read Data
        for (uint8_t i = 0; i < request; i++)
        {
            *data++ = bus->i2c->read();
        }

        len -= request; // reduce remaining bytes
    }

    return BMI3_INTF_RET_SUCCESS;
}

int8_t BMI323_7Semi::i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    BMI323_7Semi *bus = (BMI323_7Semi *)intf_ptr;

    bus->i2c->beginTransmission(bus->i2c_address);
    bus->i2c->write(reg);

    for (uint32_t i = 0; i < len; i++)
        bus->i2c->write(data[i]);

    return bus->i2c->endTransmission();
}

int8_t BMI323_7Semi::spi_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr)
{
    BMI323_7Semi *bus = (BMI323_7Semi *)intf_ptr;

    bus->spi->beginTransaction(SPISettings(bus->spi_speed, MSBFIRST, SPI_MODE0));

    digitalWrite(bus->cs_pin, LOW);

    delayMicroseconds(1);

    bus->spi->transfer(reg | 0x80); // Read bit

    for (uint32_t i = 0; i < len; i++)
        data[i] = bus->spi->transfer(0x00);

    digitalWrite(bus->cs_pin, HIGH);

    bus->spi->endTransaction();

    return BMI3_INTF_RET_SUCCESS;
}

int8_t BMI323_7Semi::spi_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    BMI323_7Semi *bus = (BMI323_7Semi *)intf_ptr;

    bus->spi->beginTransaction(SPISettings(bus->spi_speed, MSBFIRST, SPI_MODE0));

    digitalWrite(bus->cs_pin, LOW);

    bus->spi->transfer(reg & 0x7F); // Write mode

    for (uint32_t i = 0; i < len; i++)
        bus->spi->transfer(data[i]);
    digitalWrite(bus->cs_pin, HIGH);

    bus->spi->endTransaction();

    return BMI3_INTF_RET_SUCCESS;
}

/**
 * Delay wrapper
 */
void BMI323_7Semi::delay_us(uint32_t period, void *intf_ptr)
{
    delayMicroseconds(period);
}
