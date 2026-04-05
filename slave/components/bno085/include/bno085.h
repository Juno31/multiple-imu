#pragma once
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

/* Parsed output from one poll cycle.
 * Quaternion components: Q14 fixed-point (divide by 16384.0 for float).
 * Linear accel components: Q8 (divide by 256.0, units m/s²).
 * Gyro components: Q9 (divide by 512.0, units rad/s).
 * accuracy: radians (Q12, divide by 4096.0) for rotation vector. */
typedef struct {
    int16_t qi, qj, qk, qr;   /* rotation vector quaternion (Q14) */
    int16_t quat_acc;          /* heading accuracy estimate (Q12, rad) */
    int16_t ax, ay, az;        /* linear acceleration (Q8, m/s²) */
    int16_t gx, gy, gz;        /* calibrated gyro (Q9, rad/s) */
    uint8_t reports_received;  /* bitmask: bit0=quat, bit1=accel, bit2=gyro */
} bno085_data_t;

/* Wire packet sent via ESP-NOW — 50 bytes, packed.
 * Python struct format: '<BBIffffffff fff fff'  (little-endian)
 * Offsets: node_id(0) calib(1) ts_us(2) qi(6) qj(10) qk(14) qr(18)
 *          quat_acc(22) ax(26) ay(30) az(34) gx(38) gy(42) gz(46) */
typedef struct __attribute__((packed)) {
    uint8_t  node_id;        /*  1 byte  — slave ID (1-8) */
    uint8_t  calib;          /*  1 byte  — reports_received bitmask */
    uint32_t timestamp_us;   /*  4 bytes — esp_timer_get_time() low 32 bits */
    float    qi, qj, qk, qr; /* 16 bytes — rotation vector */
    float    quat_acc;       /*  4 bytes — heading accuracy (rad) */
    float    ax, ay, az;     /* 12 bytes — linear accel (m/s²) */
    float    gx, gy, gz;     /* 12 bytes — calibrated gyro (rad/s) */
} imu_packet_t;              /* total: 50 bytes */

/* Initialize: drain startup traffic, send Set Feature for ROT_VEC + LINEAR_ACCEL + GYRO.
 * Returns ESP_OK or error. */
esp_err_t bno085_init(i2c_master_dev_handle_t dev);

/* Change all three report rates to hz Hz (1-200).
 * Returns ESP_OK or error. */
esp_err_t bno085_set_rate(i2c_master_dev_handle_t dev, uint8_t hz);

/* Read one SHTP packet and parse any sub-reports found.
 * out->reports_received is ORed with bits for each decoded report.
 * Call this function in a loop; aggregate until reports_received has all bits.
 * Returns: 1 if data was received, 0 if bus was idle, -1 on I2C error. */
int bno085_read(i2c_master_dev_handle_t dev, bno085_data_t *out);
