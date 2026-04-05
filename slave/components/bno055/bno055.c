#include "bno055.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bno055";

/* ---- Register map (Page 0) ----------------------------------------------- */
#define REG_CHIP_ID      0x00   /* Fixed value: 0xA0                           */
#define REG_GYR_DATA_X   0x14   /* Start of 34-byte burst read block           */
#define REG_CALIB_STAT   0x35   /* End of burst read block                     */
#define REG_UNIT_SEL     0x3B
#define REG_OPR_MODE     0x3D
#define REG_PWR_MODE     0x3E
#define REG_SYS_TRIGGER  0x3F

#define CHIP_ID_EXPECTED 0xA0
#define OPR_MODE_CONFIG  0x00
#define OPR_MODE_NDOF    0x0C   /* 9-DOF full fusion                           */
#define PWR_MODE_NORMAL  0x00
#define SYS_TRIGGER_RST  0x20   /* Software reset bit                          */
#define UNIT_SEL_SI      0x00   /* Degrees, m/s², dps, Celsius                 */

#define I2C_TIMEOUT_MS   100

/* ---- Internal helpers ----------------------------------------------------- */

static esp_err_t reg_write(const bno055_handle_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev->dev_handle, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t reg_read(const bno055_handle_t *dev,
                           uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(dev->dev_handle,
                                       &reg, 1,
                                       out, len,
                                       I2C_TIMEOUT_MS);
}

static esp_err_t wait_for_chip_id(const bno055_handle_t *dev, int tries, int delay_ms)
{
    uint8_t id = 0;
    esp_err_t last_err = ESP_FAIL;
    for (int i = 0; i < tries; i++) {
        last_err = reg_read(dev, REG_CHIP_ID, &id, 1);
        if (last_err == ESP_OK && id == CHIP_ID_EXPECTED) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "try %d/%d: i2c=%s id=0x%02X",
                 i + 1, tries, esp_err_to_name(last_err), id);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    ESP_LOGE(TAG, "BNO055 not found — check wiring and I2C address");
    return (last_err != ESP_OK) ? last_err : ESP_ERR_NOT_FOUND;
}

/* ---- Public API ----------------------------------------------------------- */

esp_err_t bno055_init(bno055_handle_t *dev)
{
    esp_err_t ret;

    /* 1. Verify chip is present */
    ret = wait_for_chip_id(dev, 5, 100);
    if (ret != ESP_OK) return ret;

    /* 2. Enter CONFIG mode (required before changing any settings) */
    ESP_RETURN_ON_ERROR(reg_write(dev, REG_OPR_MODE, OPR_MODE_CONFIG), TAG, "set CONFIG");
    vTaskDelay(pdMS_TO_TICKS(25));

    /* 3. Software reset */
    ESP_RETURN_ON_ERROR(reg_write(dev, REG_SYS_TRIGGER, SYS_TRIGGER_RST), TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(700));   /* Datasheet: 650 ms boot time after reset */

    /* 4. Wait for chip to come back */
    ret = wait_for_chip_id(dev, 10, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 did not come up after reset");
        return ret;
    }

    /* 5. Normal power mode */
    ESP_RETURN_ON_ERROR(reg_write(dev, REG_PWR_MODE, PWR_MODE_NORMAL), TAG, "power");
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 6. SI units: degrees, m/s², dps */
    ESP_RETURN_ON_ERROR(reg_write(dev, REG_UNIT_SEL, UNIT_SEL_SI), TAG, "units");

    /* 7. Switch to NDOF fusion mode */
    ESP_RETURN_ON_ERROR(reg_write(dev, REG_OPR_MODE, OPR_MODE_NDOF), TAG, "NDOF");
    vTaskDelay(pdMS_TO_TICKS(20));   /* Mode-switch transition: 19 ms            */

    ESP_LOGI(TAG, "BNO055 ready (NDOF mode)");
    return ESP_OK;
}

esp_err_t bno055_read_data(bno055_handle_t *dev, bno055_data_t *data)
{
    /*
     * Single burst read: registers 0x14 – 0x35 (34 bytes).
     *
     * Offset  Register        Content
     * ------  --------------- ----------------------------------------
     *  0, 1   GYR_DATA_X      Gyro X (LSB, MSB)
     *  2, 3   GYR_DATA_Y
     *  4, 5   GYR_DATA_Z
     *  6, 7   EUL_DATA_X      Euler Heading
     *  8, 9   EUL_DATA_Y      Euler Roll
     * 10,11   EUL_DATA_Z      Euler Pitch
     * 12,13   QUA_Data_w
     * 14,15   QUA_Data_x
     * 16,17   QUA_Data_y
     * 18,19   QUA_Data_z
     * 20,21   LIA_Data_X      Linear accel X
     * 22,23   LIA_Data_Y
     * 24,25   LIA_Data_Z
     * 26–31   GRV_Data_{X,Y,Z} (gravity vector, not used)
     * 32      TEMP             (not used)
     * 33      CALIB_STAT
     */
    uint8_t buf[34];
    esp_err_t ret = reg_read(dev, REG_GYR_DATA_X, buf, sizeof(buf));
    if (ret != ESP_OK) return ret;

/* Little-endian int16 helper */
#define LE16(i) ((int16_t)((uint16_t)buf[(i)] | ((uint16_t)buf[(i) + 1] << 8)))

    data->gyro_x  = LE16(0);
    data->gyro_y  = LE16(2);
    data->gyro_z  = LE16(4);
    data->euler_h = LE16(6);
    data->euler_r = LE16(8);
    data->euler_p = LE16(10);
    data->quat_w  = LE16(12);
    data->quat_x  = LE16(14);
    data->quat_y  = LE16(16);
    data->quat_z  = LE16(18);
    data->accel_x = LE16(20);
    data->accel_y = LE16(22);
    data->accel_z = LE16(24);
    /* buf[26..32] — gravity + temp, skipped */

    uint8_t cs = buf[33];
    data->calib_mag   = (cs >> 0) & 0x03;
    data->calib_accel = (cs >> 2) & 0x03;
    data->calib_gyro  = (cs >> 4) & 0x03;
    data->calib_sys   = (cs >> 6) & 0x03;

#undef LE16

    return ESP_OK;
}
