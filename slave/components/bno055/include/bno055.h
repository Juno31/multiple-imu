#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/* ---- I2C addresses ------------------------------------------------------- */
#define BNO055_ADDR_LOW   0x28  /* ADDR pin tied LOW  (default) */
#define BNO055_ADDR_HIGH  0x29  /* ADDR pin tied HIGH           */

/* ---- Device handle -------------------------------------------------------- */
typedef struct {
    i2c_master_dev_handle_t dev_handle;
} bno055_handle_t;

/* ---- Sensor output (raw integer values from BNO055 registers) ------------ */
typedef struct {
    /* Euler angles — 1 LSB = 1/16 degree */
    int16_t euler_h;   /* Heading  0 … 5760 (0–360 deg × 16) */
    int16_t euler_r;   /* Roll    -1440 … +1440               */
    int16_t euler_p;   /* Pitch   -1440 … +1440               */

    /* Quaternion — 1 LSB = 1/16384 */
    int16_t quat_w;
    int16_t quat_x;
    int16_t quat_y;
    int16_t quat_z;

    /* Linear acceleration (gravity-compensated) — 1 LSB = 0.01 m/s² */
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    /* Angular velocity — 1 LSB = 1/16 dps */
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    /* Calibration status [0 = uncalibrated, 3 = fully calibrated] */
    uint8_t calib_sys;
    uint8_t calib_gyro;
    uint8_t calib_accel;
    uint8_t calib_mag;
} bno055_data_t;

/* ---- API ------------------------------------------------------------------ */

/**
 * @brief  Initialise the BNO055 and put it in NDOF fusion mode.
 *
 * Performs a soft reset, verifies the chip ID, sets SI units (degrees, m/s²,
 * dps) and switches to NDOF (9-DOF full-fusion) mode.
 *
 * @param  dev   Pointer to a populated bno055_handle_t.
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if chip ID mismatch,
 *         ESP_ERR_TIMEOUT if device doesn't come up after reset.
 */
esp_err_t bno055_init(bno055_handle_t *dev);

/**
 * @brief  Read all fused sensor outputs in a single burst I2C transaction.
 *
 * Reads gyro, Euler, quaternion, linear-accel, and calibration status in one
 * 34-byte burst from register 0x14 to 0x35.
 *
 * @param  dev   Pointer to an initialised bno055_handle_t.
 * @param  data  Output struct populated with raw integer values.
 * @return ESP_OK on success, I2C error codes otherwise.
 */
esp_err_t bno055_read_data(bno055_handle_t *dev, bno055_data_t *data);
