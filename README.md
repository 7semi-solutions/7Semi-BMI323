# 7Semi BMI323 Arduino Library

![Arduino](https://img.shields.io/badge/platform-Arduino-blue.svg)
![Sensor](https://img.shields.io/badge/sensor-BMI323-green.svg)
![License](https://img.shields.io/badge/license-MIT-orange.svg)

Arduino driver for the **Bosch BMI323 6-axis IMU sensor**.

The **BMI323** is a high-performance inertial measurement unit (IMU) with an **accelerometer and gyroscope**, supporting both **I²C and SPI interfaces**. It is suitable for applications like motion tracking, robotics, wearable devices, and industrial sensing.

This library provides a clean and efficient interface for configuring the sensor and reading motion data in **g (acceleration)** and **degrees per second (dps)**.

---

# Features

* 3-axis accelerometer (X, Y, Z)
* 3-axis gyroscope (X, Y, Z)
* Raw and scaled data reading (g / dps)
* FIFO support (high-speed buffered data)
* Interrupt support (INT1 / INT2)
* Sensor time support
* Temperature measurement
* Gyroscope self-calibration
* Accelerometer FOC calibration
* I2C and SPI communication
* ESP32 custom I2C/SPI pin support

---

# Supported Platforms

* Arduino UNO / Mega
* ESP32
* Any board supporting **Wire (I²C)** or **SPI**

---

# Hardware

Supported sensor:

**7Semi 6-Axis IMU Breakout - BMI323**

---

# Connection

The **BMI323 supports both I²C and SPI communication**.

---

## I²C Connection

| BMI323 Pin | MCU Pin | Description  |
|------------|--------|-------------|
| VCC        | 3.3V   | Sensor power |
| GND        | GND    | Ground       |
| SDA        | SDA    | I²C data     |
| SCL        | SCL    | I²C clock    |

### I²C Notes

Default sensor address: 0x68


Recommended I²C speed: 100kHz - 400kHz


---

## SPI Connection

| BMI323 Pin | MCU Pin |
|------------|--------|
| VCC        | 3.3V   |
| GND        | GND    |
| SCK        | SCK    |
| MISO       | MISO   |
| MOSI       | MOSI   |
| CS         | Any GPIO |

---

# Installation

## Arduino Library Manager

1. Open **Arduino IDE**
2. Go to **Library Manager**
3. Search for **7Semi BMI323**
4. Click **Install**

---

## Manual Installation

1. Download this repository as ZIP  
2. Arduino IDE → **Sketch → Include Library → Add .ZIP Library**

---

# Example

```cpp
#include <7Semi_BMI323.h>

BMI323_7Semi imu;

void setup()
{
Serial.begin(115200);

if(!imu.beginI2C(0x68))
{
Serial.println("BMI323 not detected");
while(1);
}
}

void loop()
{
float ax, ay, az;
float gx, gy, gz;

if(imu.readAccel(ax, ay, az))
{
Serial.print("ACC: ");
Serial.print(ax);
Serial.print(", ");
Serial.print(ay);
Serial.print(", ");
Serial.println(az);
}

if(imu.readGyro(gx, gy, gz))
{
Serial.print("GYR: ");
Serial.print(gx);
Serial.print(", ");
Serial.print(gy);
Serial.print(", ");
Serial.println(gz);
}

delay(500);
}
```
---

# Library Overview

## Reading Accelerometer (g)

```cpp

float x, y, z;

imu.readAccel(x, y, z);

```

Returns acceleration in **g units**.

---

## Reading Gyroscope (dps)

```cpp

float x,y,z;

imu.readGyro(x,y,z);

```


Returns angular velocity in **degrees per second (dps)**.

---

## Reading Raw Data

```cpp
int16_t ax, ay, az;
int16_t gx, gy, gz;

imu.readAccelRaw(ax, ay, az);
imu.readGyroRaw(gx, gy, gz);
```

Returns **raw sensor values**.

---

# Sensor Configuration

## Accelerometer

```cpp
imu.setAccelConfig(
BMI3_ACC_ODR_100HZ,
BMI3_ACC_BW_ODR_QUARTER,
BMI3_ACC_MODE_NORMAL,
BMI3_ACC_RANGE_2G,
BMI3_ACC_AVG1);
```
Returns **raw sensor values**.

## Gyroscope

```cpp
imu.setGyroConfig(
BMI3_GYR_ODR_100HZ,
BMI3_GYR_BW_ODR_QUARTER,
BMI3_GYR_MODE_NORMAL,
BMI3_GYR_RANGE_2000DPS,
BMI3_GYR_AVG1);

```

---

# FIFO Usage

Enable FIFO:
```cpp
imu.setFifoConfig(BMI_FIFO_ALL_EN,true);
imu.setFifoWatermark(200);
```

Read FIFO:
```cpp
bmi3_fifo_frame fifo;

fifo.data = buffer;
fifo.length = buffer_size;

imu.getFifoData(fifo);
```

---

# Interrupts

Configure interrupt pin:

```cpp
imu.setInterruptConfig(BMI3_INT1, true, false);
imu.enableInterrupt(BMI3_INT1, true, false);

```

---

# Temperature

```cpp
float temp;

imu.getTemperature(temp);

```
Returns temperature in **°C**.

---

# Calibration

## Gyroscope Calibration

```cpp
imu.calibrateGyro(1, true);

```

---

## Accelerometer FOC
```cpp
imu.calibrateAccelFOC(true, false, false, false);
```

---

# Example Applications

Typical applications include:

* Motion tracking
* Robotics and drones
* Gesture detection
* Wearable devices
* Industrial monitoring
* Orientation sensing

---

# License

MIT License

---

# Author

7Semi

