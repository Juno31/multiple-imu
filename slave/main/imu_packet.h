#pragma once
#include <stdint.h>

/*
 * IMU packet definition — slave side.
 *
 * BNO055 raw-value scaling (native units, no float on-device):
 *   Euler angles : divide int16 by 16.0   → degrees
 *   Quaternion   : divide int16 by 16384.0
 *   Linear accel : divide int16 by 100.0  → m/s²
 *   Gyro         : divide int16 by 16.0   → dps
 */

typedef struct __attribute__((packed)) {
    uint8_t  slave_mac[6];    /* Slave MAC address — unique node identifier          */
    uint32_t timestamp_ms;    /* ms since slave boot (from esp_timer_get_time / 1000) */

    int16_t  euler_h;         /* Heading  (÷16 → deg, 0–360)                         */
    int16_t  euler_r;         /* Roll     (÷16 → deg)                                 */
    int16_t  euler_p;         /* Pitch    (÷16 → deg)                                 */

    int16_t  quat_w;          /* Quaternion W (÷16384)                                */
    int16_t  quat_x;          /* Quaternion X                                         */
    int16_t  quat_y;          /* Quaternion Y                                         */
    int16_t  quat_z;          /* Quaternion Z                                         */

    int16_t  accel_x;         /* Linear accel X (÷100 → m/s²)                        */
    int16_t  accel_y;
    int16_t  accel_z;

    int16_t  gyro_x;          /* Gyro X (÷16 → dps)                                  */
    int16_t  gyro_y;
    int16_t  gyro_z;

    uint8_t  calib_sys;       /* System calibration status [0=uncal … 3=fully cal]   */
    uint8_t  calib_gyro;
    uint8_t  calib_accel;
    uint8_t  calib_mag;
} imu_packet_t;               /* Total: 40 bytes — well within ESP-NOW 250-byte limit */
