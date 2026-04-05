#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bno085.h"

static const char *TAG = "imu_slave";

#define I2C_SDA_GPIO  6
#define I2C_SCL_GPIO  7
#define BNO085_ADDR   0x4A

/* ---- Node states -------------------------------------------------------- */
#define NODE_IDLE       0   /* booted, BNO085 not initialized */
#define NODE_CONNECTED  1   /* BNO085 ready, not streaming */
#define NODE_STREAMING  2   /* actively sending IMU packets */

/* ---- Beacon packet (slave → master, 4 bytes) ---------------------------- */
#define PKT_BEACON  0xB0

typedef struct __attribute__((packed)) {
    uint8_t type;      /* PKT_BEACON */
    uint8_t node_id;
    uint8_t state;     /* NODE_* */
    uint8_t pad;
} beacon_packet_t;

_Static_assert(sizeof(beacon_packet_t) == 4, "beacon_packet_t must be 4 bytes");

/* ---- Command packet (master → slave via ESP-NOW, 8 bytes) --------------- */
#define CMD_RATE     0x01   /* param = rate Hz */
#define CMD_START    0x02   /* resume streaming */
#define CMD_STOP     0x03   /* pause streaming */
#define CMD_CALIB    0x04   /* reinitialize BNO085 */
#define CMD_SYNC     0x05   /* arg  = master timestamp_us (uint32) */
#define CMD_CONNECT     0x06   /* trigger BNO085 init, go CONNECTED */
#define CMD_DISCONNECT  0x07   /* stop streaming, go back to IDLE */

typedef struct __attribute__((packed)) {
    uint8_t  cmd;
    uint8_t  param;
    uint8_t  pad[2];
    uint32_t arg;         /* CMD_SYNC: master esp_timer_get_time() lower 32 bits */
} cmd_packet_t;

_Static_assert(sizeof(cmd_packet_t) == 8, "cmd_packet_t must be 8 bytes");

/* ---- Shared state ------------------------------------------------------- */
static volatile uint8_t  g_node_state        = NODE_IDLE;
static volatile int32_t  g_time_offset_us    = 0;
static volatile bool     g_connect_requested = false; /* set in callback, cleared in main */
static volatile bool     g_calib_requested   = false;
static volatile bool     g_streaming         = false;

/* ---- Connection loss detection ------------------------------------------ */
#define SEND_FAIL_THRESHOLD  20   /* consecutive failures before reset to IDLE */
static volatile uint32_t g_consec_send_fails = 0;

/* ---- Orientation tare (reset-to-zero) ----------------------------------- */
static volatile bool g_tare_requested = false;
static float g_ref_qi = 0, g_ref_qj = 0, g_ref_qk = 0, g_ref_qr = 1; /* identity */
static bool  g_ref_valid = false;

/*
 * Apply tare: q_out = q_ref_inv * q_in  (Hamilton product)
 * q_ref_inv = conjugate of unit quaternion = (-i, -j, -k, +r)
 */
static void quat_apply_tare(float qi, float qj, float qk, float qr,
                            float *oi, float *oj, float *ok, float *or_)
{
    /* inverse of reference */
    float ri = -g_ref_qi, rj = -g_ref_qj, rk = -g_ref_qk, rr = g_ref_qr;
    /* Hamilton product: ref_inv * q */
    *or_ = rr*qr - ri*qi - rj*qj - rk*qk;
    *oi  = rr*qi + ri*qr + rj*qk - rk*qj;
    *oj  = rr*qj - ri*qk + rj*qr + rk*qi;
    *ok  = rr*qk + ri*qj - rj*qi + rk*qr;
}

static i2c_master_dev_handle_t g_i2c_dev;
static uint8_t g_master_mac[6];
static uint8_t g_node_id;        /* last byte of own MAC — unique per board */

/* ---- Parse "XX:XX:XX:XX:XX:XX" into byte array ------------------------- */
static void parse_mac(const char *str, uint8_t out[6])
{
    unsigned v[6] = {0};
    sscanf(str, "%x:%x:%x:%x:%x:%x",
           &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]);
    for (int i = 0; i < 6; i++) out[i] = (uint8_t)v[i];
}

/* ---- Send beacon to master ---------------------------------------------- */
static void send_beacon(void)
{
    beacon_packet_t b = {
        .type    = PKT_BEACON,
        .node_id = g_node_id,
        .state   = g_node_state,
        .pad     = 0,
    };
    esp_now_send(g_master_mac, (uint8_t *)&b, sizeof(b));
}

/* ---- ESP-NOW recv callback (commands from master) ----------------------- */
static void on_cmd_recv(const esp_now_recv_info_t *info,
                        const uint8_t *data, int len)
{
    if (len != sizeof(cmd_packet_t)) return;
    const cmd_packet_t *cmd = (const cmd_packet_t *)data;

    switch (cmd->cmd) {
        case CMD_RATE:
            if (cmd->param >= 10 && cmd->param <= 200 &&
                g_node_state != NODE_IDLE) {
                bno085_set_rate(g_i2c_dev, cmd->param);
                ESP_LOGI(TAG, "Rate → %dHz", cmd->param);
            }
            break;

        case CMD_START:
            if (g_node_state != NODE_IDLE) {
                g_streaming  = true;
                g_node_state = NODE_STREAMING;
                ESP_LOGI(TAG, "Streaming started");
            }
            break;

        case CMD_STOP:
            g_streaming = false;
            if (g_node_state == NODE_STREAMING)
                g_node_state = NODE_CONNECTED;
            ESP_LOGI(TAG, "Streaming stopped");
            break;

        case CMD_CALIB:
            if (g_node_state == NODE_STREAMING || g_node_state == NODE_CONNECTED) {
                /* Tare: capture current orientation as zero reference */
                g_tare_requested = true;
                ESP_LOGI(TAG, "Tare requested — next reading becomes zero");
            } else {
                /* IDLE: full reinit */
                g_calib_requested = true;
            }
            break;

        case CMD_SYNC: {
            uint32_t local_us = (uint32_t)esp_timer_get_time();
            g_time_offset_us  = (int32_t)(cmd->arg - local_us);
            ESP_LOGI(TAG, "Time sync: master=%"PRIu32" local=%"PRIu32" offset=%"PRId32"us",
                     cmd->arg, local_us, g_time_offset_us);
            break;
        }

        case CMD_CONNECT:
            /* Defer to main task — bno085_init must not run in Wi-Fi task */
            if (g_node_state == NODE_IDLE) {
                g_connect_requested = true;
            }
            break;

        case CMD_DISCONNECT:
            g_streaming  = false;
            g_node_state = NODE_IDLE;
            ESP_LOGI(TAG, "Disconnected — back to IDLE");
            break;
    }
}

/* ---- ESP-NOW send callback (connection loss detection) ------------------ */
static void on_send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        g_consec_send_fails = 0;
    } else {
        g_consec_send_fails++;
        if (g_consec_send_fails >= SEND_FAIL_THRESHOLD &&
            g_node_state != NODE_IDLE) {
            ESP_LOGW(TAG, "Connection lost (%"PRIu32" consecutive send failures) — resetting to IDLE",
                     g_consec_send_fails);
            g_streaming  = false;
            g_node_state = NODE_IDLE;
            g_consec_send_fails = 0;
        }
    }
}

/* ---- ESP-NOW init ------------------------------------------------------- */
static void espnow_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_SLAVE_ESPNOW_CHANNEL,
                                         WIFI_SECOND_CHAN_NONE));

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    g_node_id = mac[5];   /* unique per board, no config needed */
    ESP_LOGI(TAG, "Slave MAC: %02X:%02X:%02X:%02X:%02X:%02X  node_id=0x%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], g_node_id);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_cmd_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_send_cb));

    parse_mac(CONFIG_SLAVE_MASTER_MAC, g_master_mac);
    ESP_LOGI(TAG, "Master MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             g_master_mac[0], g_master_mac[1], g_master_mac[2],
             g_master_mac[3], g_master_mac[4], g_master_mac[5]);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, g_master_mac, 6);
    peer.channel = CONFIG_SLAVE_ESPNOW_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void app_main(void)
{
    /* ---- I2C bus + device (no BNO085 init yet) ---- */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port                     = I2C_NUM_0,
        .sda_io_num                   = I2C_SDA_GPIO,
        .scl_io_num                   = I2C_SCL_GPIO,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BNO085_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &g_i2c_dev));

    /* ---- ESP-NOW ---- */
    espnow_init();

    ESP_LOGI(TAG, "Node %d idle — waiting for Connect command from master",
             g_node_id);

    /* ---- Main loop ---- */
    bno085_data_t raw    = {0};
    int64_t last_log_us  = 0;
    int64_t last_bcn_us  = 0;
    uint32_t sent_ok = 0, sent_fail = 0;

    while (1) {
        int64_t now = esp_timer_get_time();

        /* ---- Handle deferred connect ---- */
        if (g_connect_requested) {
            g_connect_requested = false;
            ESP_LOGI(TAG, "Connecting — initializing BNO085...");
            if (bno085_init(g_i2c_dev) == ESP_OK) {
                g_node_state = NODE_CONNECTED;
                g_streaming  = false;
                send_beacon();
                last_bcn_us = now;
                ESP_LOGI(TAG, "BNO085 ready — state: CONNECTED");
            } else {
                ESP_LOGE(TAG, "BNO085 init failed — staying IDLE");
            }
        }

        /* ---- Handle deferred calibration ---- */
        if (g_calib_requested) {
            g_calib_requested = false;
            ESP_LOGI(TAG, "Recalibrating BNO085...");
            if (bno085_init(g_i2c_dev) == ESP_OK) {
                g_node_state = NODE_CONNECTED;
                g_streaming  = false;
                send_beacon();
                last_bcn_us = now;
                ESP_LOGI(TAG, "Recalibration done — state: CONNECTED");
            }
        }

        /* ---- IDLE: beacon every 1 s, wait for CMD_CONNECT ---- */
        if (g_node_state == NODE_IDLE) {
            if (now - last_bcn_us >= 1000000LL) {
                send_beacon();
                last_bcn_us = now;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* ---- CONNECTED / STREAMING: poll BNO085 ---- */
        int rc = bno085_read(g_i2c_dev, &raw);
        if (rc < 0) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
        if (rc == 0) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }

        if ((raw.reports_received & 0x07) != 0x07) continue;

        if (!g_streaming) {
            raw.reports_received = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* ---- Scale raw to float ---- */
        float qi_raw = raw.qi / 16384.0f;
        float qj_raw = raw.qj / 16384.0f;
        float qk_raw = raw.qk / 16384.0f;
        float qr_raw = raw.qr / 16384.0f;

        /* ---- Tare: capture current orientation as zero ---- */
        if (g_tare_requested) {
            g_tare_requested = false;
            g_ref_qi = qi_raw;
            g_ref_qj = qj_raw;
            g_ref_qk = qk_raw;
            g_ref_qr = qr_raw;
            g_ref_valid = true;
            ESP_LOGI(TAG, "Tare set: ref=(%6.3f,%6.3f,%6.3f,%6.3f)",
                     g_ref_qi, g_ref_qj, g_ref_qk, g_ref_qr);
        }

        /* ---- Apply tare if active ---- */
        float qi_out = qi_raw, qj_out = qj_raw, qk_out = qk_raw, qr_out = qr_raw;
        if (g_ref_valid) {
            quat_apply_tare(qi_raw, qj_raw, qk_raw, qr_raw,
                            &qi_out, &qj_out, &qk_out, &qr_out);
        }

        /* ---- Pack ---- */
        imu_packet_t pkt = {
            .node_id      = g_node_id,
            .calib        = raw.reports_received,
            .timestamp_us = (uint32_t)esp_timer_get_time() + (uint32_t)g_time_offset_us,
            .qi  = qi_out,
            .qj  = qj_out,
            .qk  = qk_out,
            .qr  = qr_out,
            .quat_acc = raw.quat_acc / 4096.0f,
            .ax  = raw.ax  / 256.0f,
            .ay  = raw.ay  / 256.0f,
            .az  = raw.az  / 256.0f,
            .gx  = raw.gx  / 512.0f,
            .gy  = raw.gy  / 512.0f,
            .gz  = raw.gz  / 512.0f,
        };

        /* ---- Send ---- */
        esp_err_t err = esp_now_send(g_master_mac, (uint8_t *)&pkt, sizeof(pkt));
        if (err == ESP_OK) sent_ok++; else sent_fail++;

        raw.reports_received = 0;

        /* Beacon heartbeat every 5 s while streaming (keeps master state fresh) */
        if (now - last_bcn_us >= 5000000LL) {
            send_beacon();
            last_bcn_us = now;
        }

        /* 2 Hz status log */
        if (now - last_log_us >= 500000LL) {
            ESP_LOGI(TAG,
                "quat i=%6.3f j=%6.3f k=%6.3f r=%6.3f | "
                "accel x=%5.2f y=%5.2f z=%5.2f | "
                "gyro x=%5.2f y=%5.2f z=%5.2f | "
                "sent=%"PRIu32" fail=%"PRIu32,
                pkt.qi, pkt.qj, pkt.qk, pkt.qr,
                pkt.ax, pkt.ay, pkt.az,
                pkt.gx, pkt.gy, pkt.gz,
                sent_ok, sent_fail);
            last_log_us = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
