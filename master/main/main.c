#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "imu_master";

#define ESPNOW_CHANNEL  1

/* ---- imu_packet_t (must match slave definition exactly, 50 bytes) ---- */
typedef struct __attribute__((packed)) {
    uint8_t  node_id;
    uint8_t  calib;
    uint32_t timestamp_us;
    float    qi, qj, qk, qr;
    float    quat_acc;
    float    ax, ay, az;
    float    gx, gy, gz;
} imu_packet_t;

_Static_assert(sizeof(imu_packet_t) == 50, "imu_packet_t must be 50 bytes");

/* ---- Frame: [0xAA][0x55][50 bytes][uint32_t timestamp_ms][CRC8] = 57 bytes ---- */
#define FRAME_MAGIC0  0xAA
#define FRAME_MAGIC1  0x55
#define FRAME_SIZE    57

/* ---- Node states ------------------------------------------------------- */
#define NODE_IDLE       0
#define NODE_CONNECTED  1
#define NODE_STREAMING  2

/* ---- Beacon from slave (ESP-NOW, 4 bytes) ------------------------------ */
#define PKT_BEACON  0xB0

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t node_id;
    uint8_t state;
    uint8_t pad;
} beacon_packet_t;

_Static_assert(sizeof(beacon_packet_t) == 4, "beacon_packet_t must be 4 bytes");

/* ---- Node-event frame to PC (11 bytes) --------------------------------- */
/* [0xAA][0xBB][node_id(1)][state(1)][mac(6)][CRC8(1)] */
#define NODE_EVT_MAGIC0  0xAA
#define NODE_EVT_MAGIC1  0xBB
#define NODE_EVT_SIZE    11

/* ---- Command packet (master → slave via ESP-NOW, 8 bytes) ------------ */
#define CMD_RATE     0x01   /* param = rate Hz */
#define CMD_START    0x02
#define CMD_STOP     0x03
#define CMD_CALIB    0x04
#define CMD_SYNC     0x05   /* arg = master timestamp_us */
#define CMD_CONNECT     0x06   /* trigger BNO085 init on slave */
#define CMD_DISCONNECT  0x07   /* slave goes back to IDLE */

typedef struct __attribute__((packed)) {
    uint8_t  cmd;
    uint8_t  param;
    uint8_t  pad[2];
    uint32_t arg;         /* CMD_SYNC: esp_timer_get_time() lower 32 bits */
} cmd_packet_t;

_Static_assert(sizeof(cmd_packet_t) == 8, "cmd_packet_t must be 8 bytes");

/* ---- Slave MAC table -------------------------------------------------- */
#define MAX_SLAVES  8

typedef struct {
    uint8_t node_id;
    uint8_t mac[6];
    uint8_t state;      /* NODE_* */
} slave_entry_t;

static slave_entry_t     slave_table[MAX_SLAVES];
static int               slave_count = 0;
static SemaphoreHandle_t slave_mutex;

/* ---- Time sync (PC epoch → master local clock) ----------------------- */
static volatile uint64_t g_sync_epoch_ms  = 0;   /* PC wall-clock at sync moment */
static volatile int64_t  g_sync_master_us = 0;   /* esp_timer_get_time() at sync */
static volatile bool     g_synced         = false;

/* ---- Stats ------------------------------------------------------------ */
static volatile uint32_t rx_total = 0;
static volatile uint32_t rx_bad   = 0;

/* ---- CRC8 ------------------------------------------------------------- */
static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

/* ---- Send node-event frame to PC -------------------------------------- */
static void send_node_event(uint8_t node_id, uint8_t state, const uint8_t mac[6])
{
    uint8_t frame[NODE_EVT_SIZE];
    frame[0] = NODE_EVT_MAGIC0;
    frame[1] = NODE_EVT_MAGIC1;
    frame[2] = node_id;
    frame[3] = state;
    memcpy(&frame[4], mac, 6);
    frame[10] = crc8(&frame[2], 8);   /* CRC over node_id+state+mac (8 bytes) */
    fwrite(frame, 1, NODE_EVT_SIZE, stdout);
    fflush(stdout);
}

/* ---- Send a command packet to one slave (node_id) or all (0xFF) ------ */
/* ---- Broadcast time-sync to all slaves -------------------------------- */
static void send_sync(void)
{
    cmd_packet_t pkt = {
        .cmd   = CMD_SYNC,
        .param = 0,
        .pad   = {0, 0},
        .arg   = (uint32_t)esp_timer_get_time(),
    };
    xSemaphoreTake(slave_mutex, portMAX_DELAY);
    for (int i = 0; i < slave_count; i++)
        esp_now_send(slave_table[i].mac, (uint8_t *)&pkt, sizeof(pkt));
    xSemaphoreGive(slave_mutex);
    ESP_LOGI(TAG, "CMD SYNC → all slaves (t=%"PRIu32"us)", pkt.arg);
}

static void send_cmd(uint8_t node_id, uint8_t cmd, uint8_t param)
{
    cmd_packet_t pkt = { .cmd = cmd, .param = param, .pad = {0, 0}, .arg = 0 };

    xSemaphoreTake(slave_mutex, portMAX_DELAY);
    if (node_id == 0xFF) {
        for (int i = 0; i < slave_count; i++)
            esp_now_send(slave_table[i].mac, (uint8_t *)&pkt, sizeof(pkt));
    } else {
        for (int i = 0; i < slave_count; i++) {
            if (slave_table[i].node_id == node_id) {
                esp_now_send(slave_table[i].mac, (uint8_t *)&pkt, sizeof(pkt));
                break;
            }
        }
    }
    xSemaphoreGive(slave_mutex);
}

/* ---- ESP-NOW receive callback (Wi-Fi task context) ------------------- */
static void on_recv(const esp_now_recv_info_t *info,
                    const uint8_t *data, int len)
{
    /* ---- Beacon from slave (4 bytes) ---------------------------------- */
    if (len == sizeof(beacon_packet_t) && data[0] == PKT_BEACON) {
        const beacon_packet_t *b = (const beacon_packet_t *)data;

        uint8_t  mac_copy[6];
        uint8_t  old_state = 0xFF;
        bool     is_new    = false;

        xSemaphoreTake(slave_mutex, portMAX_DELAY);
        int idx = -1;
        for (int i = 0; i < slave_count; i++) {
            if (slave_table[i].node_id == b->node_id) { idx = i; break; }
        }
        if (idx == -1 && slave_count < MAX_SLAVES) {
            idx = slave_count++;
            slave_table[idx].node_id = b->node_id;
            memcpy(slave_table[idx].mac, info->src_addr, 6);
            slave_table[idx].state = NODE_IDLE;

            esp_now_peer_info_t peer = {};
            memcpy(peer.peer_addr, info->src_addr, 6);
            peer.channel = ESPNOW_CHANNEL;
            peer.ifidx   = WIFI_IF_STA;
            peer.encrypt = false;
            esp_now_add_peer(&peer);

            is_new = true;
            ESP_LOGI(TAG, "Discovered node=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X",
                     b->node_id,
                     info->src_addr[0], info->src_addr[1], info->src_addr[2],
                     info->src_addr[3], info->src_addr[4], info->src_addr[5]);
        }
        if (idx != -1) {
            old_state = slave_table[idx].state;
            slave_table[idx].state = b->state;
            memcpy(mac_copy, slave_table[idx].mac, 6);
        }
        xSemaphoreGive(slave_mutex);

        /* Always emit node_event on every beacon so bridge.py stays in sync
         * even if it connected after initial discovery */
        if (idx != -1) {
            send_node_event(b->node_id, b->state, mac_copy);
        }
        return;
    }

    /* ---- IMU data from slave (50 bytes) ------------------------------- */
    if (len == sizeof(imu_packet_t)) {
        const imu_packet_t *pkt = (const imu_packet_t *)data;

        /* Update to STREAMING state if changed */
        uint8_t  mac_copy[6];
        bool     emit = false;

        xSemaphoreTake(slave_mutex, portMAX_DELAY);
        for (int i = 0; i < slave_count; i++) {
            if (slave_table[i].node_id == pkt->node_id) {
                if (slave_table[i].state != NODE_STREAMING) {
                    slave_table[i].state = NODE_STREAMING;
                    memcpy(mac_copy, slave_table[i].mac, 6);
                    emit = true;
                }
                break;
            }
        }
        xSemaphoreGive(slave_mutex);

        if (emit)
            send_node_event(pkt->node_id, NODE_STREAMING, mac_copy);

        /* Forward framed binary to PC: [0xAA][0x55][50B imu][4B ts_ms][CRC8] */
        uint8_t frame[FRAME_SIZE];
        frame[0] = FRAME_MAGIC0;
        frame[1] = FRAME_MAGIC1;
        memcpy(&frame[2], data, sizeof(imu_packet_t));

        /* M4: compute ms elapsed since sync (fits uint32_t for ~49 days) */
        uint32_t ts_ms = 0;
        if (g_synced) {
            int64_t now_us = esp_timer_get_time();
            ts_ms = (uint32_t)((now_us - g_sync_master_us) / 1000);
        }
        memcpy(&frame[2 + sizeof(imu_packet_t)], &ts_ms, sizeof(ts_ms));

        frame[FRAME_SIZE - 1] = crc8(&frame[2], sizeof(imu_packet_t) + sizeof(ts_ms));
        fwrite(frame, 1, FRAME_SIZE, stdout);
        fflush(stdout);
        rx_total++;
        return;
    }

    rx_bad++;
}

/* ---- Serial command reader task --------------------------------------- */
static void serial_cmd_task(void *arg)
{
    char buf[64];
    int  pos = 0;

    while (1) {
        int c = fgetc(stdin);
        if (c == EOF || c < 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        if (c == '\n' || c == '\r') {
            buf[pos] = '\0';
            if (pos > 0) {
                unsigned node_id, param;
                if (sscanf(buf, "RATE:%u", &param) == 1) {
                    send_cmd(0xFF, CMD_RATE, (uint8_t)param);
                    ESP_LOGI(TAG, "CMD RATE %uHz → all slaves", param);
                } else if (sscanf(buf, "START:%u", &node_id) == 1) {
                    send_cmd((uint8_t)node_id, CMD_START, 0);
                    ESP_LOGI(TAG, "CMD START → node %u", node_id);
                } else if (sscanf(buf, "STOP:%u", &node_id) == 1) {
                    send_cmd((uint8_t)node_id, CMD_STOP, 0);
                    ESP_LOGI(TAG, "CMD STOP → node %u", node_id);
                } else if (sscanf(buf, "CALIB:%u", &node_id) == 1) {
                    send_cmd((uint8_t)node_id, CMD_CALIB, 0);
                    ESP_LOGI(TAG, "CMD CALIB → node %u", node_id);
                } else if (strcmp(buf, "SYNC") == 0) {
                    send_sync();
                } else if (buf[0] == 'S' && buf[1] >= '0' && buf[1] <= '9') {
                    /* M1/M2: S<epoch_ms>\n — PC time sync */
                    uint64_t epoch_ms = 0;
                    for (int k = 1; buf[k] >= '0' && buf[k] <= '9'; k++)
                        epoch_ms = epoch_ms * 10 + (buf[k] - '0');
                    g_sync_epoch_ms  = epoch_ms;
                    g_sync_master_us = esp_timer_get_time();
                    g_synced         = true;
                    ESP_LOGI(TAG, "SYNC: epoch_ms=%llu master_us=%lld",
                             (unsigned long long)epoch_ms, (long long)g_sync_master_us);
                } else if (sscanf(buf, "CONNECT:%u", &node_id) == 1) {
                    send_cmd((uint8_t)node_id, CMD_CONNECT, 0);
                    ESP_LOGI(TAG, "CMD CONNECT → node %u", node_id);
                } else if (sscanf(buf, "DISCONNECT:%u", &node_id) == 1) {
                    send_cmd((uint8_t)node_id, CMD_DISCONNECT, 0);
                    ESP_LOGI(TAG, "CMD DISCONNECT → node %u", node_id);
                }
            }
            pos = 0;
        } else if (pos < (int)sizeof(buf) - 1) {
            buf[pos++] = (char)c;
        }
    }
}

/* ---- ESP-NOW init ----------------------------------------------------- */
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
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    ESP_LOGI(TAG, "Master MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
    ESP_LOGI(TAG, "ESP-NOW ready, channel %d", ESPNOW_CHANNEL);
}

void app_main(void)
{
    slave_mutex = xSemaphoreCreateMutex();

    espnow_init();

    xTaskCreate(serial_cmd_task, "serial_cmd", 4096, NULL, 5, NULL);

    int64_t last_log = esp_timer_get_time();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        int64_t now = esp_timer_get_time();
        if (now - last_log >= 5000000) {
            float sync_age_s = g_synced
                ? (float)(esp_timer_get_time() - g_sync_master_us) / 1e6f
                : -1.0f;
            xSemaphoreTake(slave_mutex, portMAX_DELAY);
            ESP_LOGI(TAG, "rx total=%"PRIu32" bad=%"PRIu32" slaves=%d synced=%d sync_age=%.1fs",
                     rx_total, rx_bad, slave_count, (int)g_synced, sync_age_s);
            xSemaphoreGive(slave_mutex);
            last_log = now;
        }
    }
}
