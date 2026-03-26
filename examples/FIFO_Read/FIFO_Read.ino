/**
 * 7Semi BMI323 - FIFO I2C Example
 *
 * --------------------------------------------------------
 * - Demonstrates FIFO usage using BMI323 over I2C
 * - Uses watermark-based polling (no interrupt required)
 * - Reads multiple frames from FIFO and parses them
 *
 * Key Flow:
 * - Configure sensor → Enable FIFO → Set watermark
 * - Poll FIFO length → Read → Parse → Print
 *
 * --------------------------------------------------------
 * Hardware Connection (I2C):
 *
 * BMI323        →    MCU
 * -----------------------------------
 * VCC           →    3.3V
 * GND           →    GND
 * SDA           →    SDA
 * SCL           →    SCL
 *
 * Notes:
 * - Use 4.7kΩ pull-ups on SDA/SCL
 * - Default I2C address: 0x68
 *
 * --------------------------------------------------------
 */

#include <7Semi_BMI323.h>

#define FIFO_BUFFER_SIZE 512   // Raw FIFO buffer
#define MAX_FRAMES       20    // Max parsed frames (if needed)

BMI323_7Semi imu;

/**
 * FIFO raw data buffer
 */
uint8_t fifo_buffer[FIFO_BUFFER_SIZE];

void setup()
{
    Serial.begin(115200);

    Serial.println("BMI323 FIFO I2C Example");

    /**
     * Initialize I2C bus
     */
    Wire.begin();
    Wire.setClock(400000);

    /**
     * Initialize IMU
     */
    if (!imu.beginI2C(0x68))
    {
        Serial.println("IMU init failed!");
        while (1);
    }

    /**
     * Configure accelerometer
     */
    imu.setAccelConfig(
        BMI3_ACC_ODR_100HZ,
        BMI3_ACC_BW_ODR_QUARTER,
        BMI3_ACC_MODE_NORMAL,
        BMI3_ACC_RANGE_2G,
        BMI3_ACC_AVG1);

    /**
     * Configure gyroscope
     */
    imu.setGyroConfig(
        BMI3_GYR_ODR_100HZ,
        BMI3_GYR_BW_ODR_QUARTER,
        BMI3_GYR_MODE_NORMAL,
        BMI3_GYR_RANGE_2000DPS,
        BMI3_GYR_AVG1);

    /**
     * Enable FIFO for accel + gyro
     */
    imu.setFifoConfig(BMI3_FIFO_ALL_EN, true);

    /**
     * Set FIFO watermark (in BYTES)
     *
     * - 160 bytes ≈ ~13 frames (12 bytes per frame)
     */
    imu.setFifoWatermark(160);

    Serial.println("FIFO configured");
}

void loop()
{
    uint16_t fifo_length = 0;

    /**
     * Get FIFO data length
     */
    if (!imu.getFifoLength(fifo_length))
        return;

    /**
     * No data available
     */
    if (fifo_length == 0)
        return;

    /**
     * Clamp to buffer size
     */
    if (fifo_length > FIFO_BUFFER_SIZE)
        fifo_length = FIFO_BUFFER_SIZE;

    /**
     * Prepare FIFO frame struct
     */
    bmi3_fifo_frame fifo = {0};
    fifo.data = fifo_buffer;
    fifo.length = fifo_length;

    /**
     * Read FIFO data
     */
    if (!imu.getFifoData(fifo))
    {
        Serial.println("FIFO read failed");
        return;
    }

    /**
     * Parse FIFO data
     */
    parseFIFO_ALL(fifo.data, fifo.length);

    Serial.println("--------------------");

    delay(200);
}

/**
 * Parse FIFO data (ACC + GYR + TEMP + TIME)
 *
 * - Frame format assumed:
 *   ACC (6 bytes) + GYR (6 bytes) + TEMP (2 bytes) + TIME (2 bytes)
 *   → Total = 16 bytes per frame
 *
 * Important:
 * - Works only when FIFO header is disabled
 * - Assumes continuous frame structure
 */
void parseFIFO_ALL(const uint8_t *data, uint16_t length)
{
    const uint8_t FRAME_SIZE = 16;

    /**
     * Not enough data for even one frame
     */
    if (length < FRAME_SIZE)
        return;

    uint16_t total_frames = length / FRAME_SIZE;

    /**
     * Loop through frames
     */
    for (uint16_t f = 0; f < total_frames; f++)
    {
        uint16_t idx = f * FRAME_SIZE + 2;

        /**
         * -------- ACC --------
         */
        int16_t ax = (int16_t)((data[idx + 1] << 8) | data[idx + 0]);
        int16_t ay = (int16_t)((data[idx + 3] << 8) | data[idx + 2]);
        int16_t az = (int16_t)((data[idx + 5] << 8) | data[idx + 4]);

        /**
         * -------- GYR --------
         */
        int16_t gx = (int16_t)((data[idx + 7] << 8) | data[idx + 6]);
        int16_t gy = (int16_t)((data[idx + 9] << 8) | data[idx + 8]);
        int16_t gz = (int16_t)((data[idx + 11] << 8) | data[idx + 10]);

        /**
         * -------- TEMP --------
         */
        int16_t temp_raw = (int16_t)((data[idx + 13] << 8) | data[idx + 12]);

        /**
         * -------- TIME --------
         */
        uint16_t time = (uint16_t)((data[idx + 15] << 8) | data[idx + 14]);

        /**
         * -------- VALIDATION --------
         *
         * Skip invalid frames:
         * - All zero values
         * - Invalid accel marker (-32768)
         */
        if ((ax == 0 && ay == 0 && az == 0 &&
             gx == 0 && gy == 0 && gz == 0) ||
            ax == -32768)
        {
            continue;
        }

        /**
         * -------- TEMP CONVERSION --------
         */
        float temp_c = NAN;

        if (temp_raw != -32768)
            temp_c = (temp_raw / 512.0f) + 23.0f;

        /**
         * -------- PRINT --------
         */
        Serial.print("ACC: ");
        Serial.print(ax); Serial.print(", ");
        Serial.print(ay); Serial.print(", ");
        Serial.print(az);

        Serial.print(" | GYR: ");
        Serial.print(gx); Serial.print(", ");
        Serial.print(gy); Serial.print(", ");
        Serial.print(gz);

        if (!isnan(temp_c))
        {
            Serial.print(" | TEMP: ");
            Serial.print(temp_c);
        }

        if (time != 0)
        {
            Serial.print(" | TIME: ");
            Serial.print(time);
        }

        Serial.println();
    }
}