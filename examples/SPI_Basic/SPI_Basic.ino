#include <7Semi_BMI323.h>

/**
 * 7Semi BMI323 - SPI Basic Example
 *
 * --------------------------------------------------------
 * - Demonstrates basic SPI communication with BMI323
 * - Reads accelerometer, gyroscope, and temperature data
 *
 * Flow:
 * - Initialize SPI interface
 * - Configure accelerometer and gyroscope
 * - Periodically read sensor data
 *
 * --------------------------------------------------------
 * Hardware Connection (SPI):
 *
 * BMI323        →    MCU
 * -----------------------------------
 * VCC           →    3.3V
 * GND           →    GND
 * SCK           →    SCK
 * MISO          →    MISO
 * MOSI          →    MOSI
 * CS            →    GPIO (defined below)
 *
 * Notes:
 * - SPI mode: MODE0
 * - Ensure proper wiring and stable power supply
 *
 * --------------------------------------------------------
 */

#define BMI_CS 10

BMI323_7Semi imu;

void setup()
{
    /**
     * Initialize serial communication
     */
    Serial.begin(115200);

    /**
     * Initialize IMU over SPI
     *
     * - Uses default SPI pins
     * - CS pin defined above
     */
    if (!imu.beginSPI(BMI_CS))
    {
        Serial.println("IMU SPI init failed!");
        while (1);
    }

    Serial.println("IMU SPI initialized");

    /**
     * Configure accelerometer
     *
     * - ODR: 100 Hz
     * - Bandwidth: ODR/4
     * - Mode: Normal
     * - Range: ±2G
     * - Averaging: 1 sample
     */
    imu.setAccelConfig(
        BMI3_ACC_ODR_100HZ,
        BMI3_ACC_BW_ODR_QUARTER,
        BMI3_ACC_MODE_NORMAL,
        BMI3_ACC_RANGE_2G,
        BMI3_ACC_AVG1);

    /**
     * Configure gyroscope
     *
     * - ODR: 100 Hz
     * - Bandwidth: ODR/4
     * - Mode: Normal
     * - Range: ±2000 dps
     * - Averaging: 1 sample
     */
    imu.setGyroConfig(
        BMI3_GYR_ODR_100HZ,
        BMI3_GYR_BW_ODR_QUARTER,
        BMI3_GYR_MODE_NORMAL,
        BMI3_GYR_RANGE_2000DPS,
        BMI3_GYR_AVG1);
}

void loop()
{
  /**
   * Variables to store sensor data
   */
  float ax, ay, az;
  float gx, gy, gz;
  float temperature = 0;

  /**
   * Read accelerometer data (g)
   */
  if (imu.readAccel(ax, ay, az))
  {
    Serial.print("Accel (g): X=");
    Serial.print(ax, 2);
    Serial.print(" Y=");
    Serial.print(ay, 2);
    Serial.print(" Z=");
    Serial.println(az, 2);
  }
  else
  {
    Serial.println("Accel read failed");
  }

  /**
   * Read gyroscope data (dps)
   */
  if (imu.readGyro(gx, gy, gz))
  {
    Serial.print("Gyro (dps): X=");
    Serial.print(gx, 2);
    Serial.print(" Y=");
    Serial.print(gy, 2);
    Serial.print(" Z=");
    Serial.println(gz, 2);
  }
  else
  {
    Serial.println("Gyro read failed");
  }

  /**
   * Read temperature (°C)
   */
  if (imu.getTemperature(temperature))
  {
    Serial.print("Temperature (C): ");
    Serial.println(temperature, 2);
  }
  else
  {
    Serial.println("Temperature read failed");
  }

  Serial.println();
}