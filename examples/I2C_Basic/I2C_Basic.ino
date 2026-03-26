/**

* 7Semi BMI323 - I2C Basic Example
*
* ---
* Description:
* * Demonstrates I2C communication with BMI323 IMU
* * Reads Accelerometer, Gyroscope, and Temperature data
* * Outputs scaled values over Serial Monitor
*
* Features:
* * I2C initialization
* * Sensor configuration (Accel + Gyro)
* * Scaled output (g, dps, °C)
*
* ---
* Hardware Connection (I2C):
*
* BMI323        →    Arduino / ESP32
* ---
* VCC           →    3.3V
* GND           →    GND
* SDA           →    SDA A4 (ESP32: GPIO21)
* SCL           →    SCL A5 (ESP32: GPIO22)
*/

#include <Wire.h>
#include "7Semi_BMI323.h"

BMI323_7Semi imu;

void setup() {
  Serial.begin(115200);


  Serial.println();
  Serial.println("7Semi BMI323 I2C Example");
  Serial.println("------------------------");

  /**
  * Initialize IMU using I2C
  */
  if (!imu.beginI2C(0x68)) {
    Serial.println("IMU I2C init failed!");
    while (1)
      ;
  }

  Serial.println("IMU initialized successfully");

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

  Serial.println("Sensor configured");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float temperature;
  uint32_t sensor_time;

  Serial.println("---- Sensor Data ----");

  /**
     * Read accelerometer
     */
  if (imu.readAccel(ax, ay, az)) {
    Serial.print("Accel (g): X=");
    Serial.print(ax, 3);
    Serial.print(" Y=");
    Serial.print(ay, 3);
    Serial.print(" Z=");
    Serial.println(az, 3);
  } else {
    Serial.println("Accel read failed");
  }

  /**
     * Read gyroscope
     */
  if (imu.readGyro(gx, gy, gz)) {
    Serial.print("Gyro (dps): X=");
    Serial.print(gx, 3);
    Serial.print(" Y=");
    Serial.print(gy, 3);
    Serial.print(" Z=");
    Serial.println(gz, 3);
  } else {
    Serial.println("Gyro read failed");
  }

  /**
     * Read temperature
     */
  if (imu.getTemperature(temperature)) {
    Serial.print("Temperature (°C): ");
    Serial.println(temperature, 2);
  } else {
    Serial.println("Temperature read failed");
  }

  /**
     * Read sensor time (optional)
     */
  if (imu.getSensorTime(sensor_time)) {
    Serial.print("Sensor Time: ");
    Serial.println(sensor_time);
  }

  Serial.println();
  delay(500);
}
