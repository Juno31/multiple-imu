#include <string.h>
#include "bno085.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bno085";

#define SHTP_MAXBUF       128
#define SHTP_CHAN_CONTROL  2
#define SHTP_CHAN_INPUT    3

#define SH2_SET_FEATURE   0xFD
#define SH2_TIMESTAMP     0xFB
#define SH2_ROT_VEC       0x05
#define SH2_LINEAR_ACCEL  0x04
#define SH2_GYRO_CAL      0x02

#define RATE_100HZ_US     10000UL

static uint8_t g_seq = 0;  /* persistent SHTP sequence counter */

/* ---- Low-level SHTP ---------------------------------------------------- */

static esp_err_t shtp_write(i2c_master_dev_handle_t dev, uint8_t channel,
                             uint8_t seq, const uint8_t *payload, uint8_t plen)
{
    uint8_t buf[SHTP_MAXBUF];
    uint16_t total = 4 + plen;
    if (total > SHTP_MAXBUF) return ESP_ERR_INVALID_SIZE;
    buf[0] = total & 0xFF;
    buf[1] = (total >> 8) & 0x7F;
    buf[2] = channel;
    buf[3] = seq;
    memcpy(&buf[4], payload, plen);
    return i2c_master_transmit(dev, buf, total, 200);
}

/* Returns payload length (>=1), 0 for empty packet, -1 on I2C error. */
static int shtp_read(i2c_master_dev_handle_t dev, uint8_t *ch, uint8_t *seq,
                     uint8_t *payload, uint8_t max_payload)
{
    uint8_t buf[SHTP_MAXBUF];
    if (i2c_master_receive(dev, buf, 4, 200) != ESP_OK) return -1;

    uint16_t pkt_len = ((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) & 0x7FFF;
    *ch  = buf[2];
    *seq = buf[3];

    if (pkt_len <= 4) return 0;

    uint16_t total = (pkt_len < SHTP_MAXBUF) ? pkt_len : SHTP_MAXBUF;
    if (i2c_master_receive(dev, buf, total, 200) != ESP_OK) return -1;

    uint16_t payload_len = total - 4;
    if (payload_len > max_payload) payload_len = max_payload;
    memcpy(payload, buf + 4, payload_len);
    return (int)payload_len;
}

static void shtp_drain(i2c_master_dev_handle_t dev)
{
    uint8_t payload[SHTP_MAXBUF], ch, seq;
    int empty = 0;
    int64_t deadline = esp_timer_get_time() + 3000000LL;
    while (empty < 5 && esp_timer_get_time() < deadline) {
        int len = shtp_read(dev, &ch, &seq, payload, sizeof(payload));
        if (len <= 0) { empty++; vTaskDelay(pdMS_TO_TICKS(20)); }
        else          { empty = 0; }
    }
    ESP_LOGI(TAG, "Drain done (idle=%s)", empty >= 5 ? "yes" : "timeout");
}

static esp_err_t send_set_feature(i2c_master_dev_handle_t dev, uint8_t report_id,
                                   uint32_t period_us, uint8_t *seq)
{
    uint8_t cmd[17] = {0};
    cmd[0] = SH2_SET_FEATURE;
    cmd[1] = report_id;
    cmd[5] = (period_us      ) & 0xFF;
    cmd[6] = (period_us >>  8) & 0xFF;
    cmd[7] = (period_us >> 16) & 0xFF;
    cmd[8] = (period_us >> 24) & 0xFF;

    esp_err_t err = ESP_FAIL;
    for (int retry = 0; retry < 5 && err != ESP_OK; retry++) {
        err = shtp_write(dev, SHTP_CHAN_CONTROL, (*seq)++, cmd, sizeof(cmd));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Set Feature 0x%02X retry %d", report_id, retry + 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    return err;
}

/* ---- Public API --------------------------------------------------------- */

esp_err_t bno085_init(i2c_master_dev_handle_t dev)
{
    vTaskDelay(pdMS_TO_TICKS(200));
    shtp_drain(dev);

    g_seq = 0;
    esp_err_t err;

    err = send_set_feature(dev, SH2_ROT_VEC, RATE_100HZ_US, &g_seq);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Set Feature ROT_VEC failed"); return err; }

    err = send_set_feature(dev, SH2_LINEAR_ACCEL, RATE_100HZ_US, &g_seq);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Set Feature LINEAR_ACCEL failed"); return err; }

    err = send_set_feature(dev, SH2_GYRO_CAL, RATE_100HZ_US, &g_seq);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Set Feature GYRO_CAL failed"); return err; }

    ESP_LOGI(TAG, "Set Feature sent: ROT_VEC + LINEAR_ACCEL + GYRO_CAL at 100Hz");
    vTaskDelay(pdMS_TO_TICKS(500));
    return ESP_OK;
}

esp_err_t bno085_set_rate(i2c_master_dev_handle_t dev, uint8_t hz)
{
    if (hz == 0) hz = 1;
    uint32_t period_us = 1000000UL / hz;
    esp_err_t err;

    err = send_set_feature(dev, SH2_ROT_VEC,      period_us, &g_seq);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Set Feature ROT_VEC failed"); return err; }

    err = send_set_feature(dev, SH2_LINEAR_ACCEL, period_us, &g_seq);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Set Feature LINEAR_ACCEL failed"); return err; }

    err = send_set_feature(dev, SH2_GYRO_CAL,     period_us, &g_seq);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Set Feature GYRO_CAL failed"); return err; }

    ESP_LOGI(TAG, "Rate changed to %dHz (period=%luus)", hz, (unsigned long)period_us);
    return ESP_OK;
}

int bno085_read(i2c_master_dev_handle_t dev, bno085_data_t *out)
{
    uint8_t payload[SHTP_MAXBUF], ch, seq;
    int len = shtp_read(dev, &ch, &seq, payload, sizeof(payload));

    if (len < 0) return -1;
    if (len == 0 || ch != SHTP_CHAN_INPUT) return 0;

    /* Scan sub-reports in the payload */
    int offset = 0;
    while (offset < len) {
        uint8_t rid = payload[offset];

        if (rid == SH2_TIMESTAMP) {
            if (offset + 5 > len) break;
            offset += 5;
            continue;
        }
        if (rid == SH2_ROT_VEC) {
            if (offset + 14 > len) break;
            out->qi       = (int16_t)((uint16_t)payload[offset+4]  | ((uint16_t)payload[offset+5]  << 8));
            out->qj       = (int16_t)((uint16_t)payload[offset+6]  | ((uint16_t)payload[offset+7]  << 8));
            out->qk       = (int16_t)((uint16_t)payload[offset+8]  | ((uint16_t)payload[offset+9]  << 8));
            out->qr       = (int16_t)((uint16_t)payload[offset+10] | ((uint16_t)payload[offset+11] << 8));
            out->quat_acc = (int16_t)((uint16_t)payload[offset+12] | ((uint16_t)payload[offset+13] << 8));
            out->reports_received |= 0x01;
            offset += 14;
            continue;
        }
        if (rid == SH2_LINEAR_ACCEL) {
            if (offset + 10 > len) break;
            out->ax = (int16_t)((uint16_t)payload[offset+4] | ((uint16_t)payload[offset+5] << 8));
            out->ay = (int16_t)((uint16_t)payload[offset+6] | ((uint16_t)payload[offset+7] << 8));
            out->az = (int16_t)((uint16_t)payload[offset+8] | ((uint16_t)payload[offset+9] << 8));
            out->reports_received |= 0x02;
            offset += 10;
            continue;
        }
        if (rid == SH2_GYRO_CAL) {
            if (offset + 10 > len) break;
            out->gx = (int16_t)((uint16_t)payload[offset+4] | ((uint16_t)payload[offset+5] << 8));
            out->gy = (int16_t)((uint16_t)payload[offset+6] | ((uint16_t)payload[offset+7] << 8));
            out->gz = (int16_t)((uint16_t)payload[offset+8] | ((uint16_t)payload[offset+9] << 8));
            out->reports_received |= 0x04;
            offset += 10;
            continue;
        }
        /* Unknown report ID — stop scanning */
        break;
    }
    return 1;
}
