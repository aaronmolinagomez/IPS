#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

/* ================= CONFIG ================= */
#define WIFI_SSID      "ZTE_2.4G_ehTuuE"
#define WIFI_PASSWORD  "xxxxxx"
#define UDP_PORT       12345
#define WINDOW_SIZE_MS 500
#define PYTHON_CLOCK_IP "192.168.1.4" // solo para referencia, no se envía

static const uint8_t TARGET_UUID[16] = {
    0xAB,0x90,0x78,0x56,0x34,0x12,0x34,0x12,
    0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12
};

/* ================= GLOBALS ================= */
static bool process_scan_results = true;
static bool found_beacon = false;
static uint8_t beacon_mac[6];
static uint8_t esp32_mac[6];
static int last_avg_rssi = -999;

static int rssi_sum = 0;
static int rssi_count = 0;
static int64_t current_window_id = -1;

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/* ================= UI ================= */
static int line_count = 0;
static bool display_initialized = false;

static void clear_screen(void) {
    printf("\033[2J\033[H");
    line_count = 0;
}

static void print_line(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    printf("\033[2K\r");
    vprintf(fmt, args);
    printf("\n");
    va_end(args);
    line_count++;
    fflush(stdout);
}

static void draw_layout(void) {
    clear_screen();
    printf("==============================================\n");
    printf("              ESP32 BLE ANCHOR\n");
    printf("==============================================\n");
    line_count += 3;

    print_line(" ESP32 MAC : --:--:--:--:--:--");
    print_line(" BEACON MAC: --:--:--:--:--:--");
    print_line(" RSSI AVG  : --- dBm");
    print_line(" IP        : ...");
    printf("==============================================\n");
    line_count++;
    display_initialized = true;
}

static void update_display_line(int line, const char *fmt, ...) {
    if (!display_initialized) return;
    int target = 3 + line;
    printf("\033[%d;0H\033[2K\r", target);

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    printf("\033[%d;0H", line_count);
    fflush(stdout);
}

static void refresh_display(void) {
    static bool mac_shown = false;
    if (!mac_shown) {
        update_display_line(0, " ESP32 MAC : %02X:%02X:%02X:%02X:%02X:%02X",
                            esp32_mac[0], esp32_mac[1], esp32_mac[2],
                            esp32_mac[3], esp32_mac[4], esp32_mac[5]);
        mac_shown = true;
    }

    if (!process_scan_results) {
        update_display_line(1, " BEACON MAC: --:--:--:--:--:--");
        update_display_line(2, " RSSI AVG  : PAUSED");
        return;
    }

    if (found_beacon) {
        update_display_line(1, " BEACON MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                            beacon_mac[0], beacon_mac[1], beacon_mac[2],
                            beacon_mac[3], beacon_mac[4], beacon_mac[5]);
        update_display_line(2, " RSSI AVG  : %d dBm", last_avg_rssi);
    } else {
        update_display_line(1, " BEACON MAC: --:--:--:--:--:--");
        update_display_line(2, " RSSI AVG  : --- dBm");
    }
}

/* ================= BLE ================= */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
    if (!process_scan_results) return;
    if (event != ESP_GAP_BLE_SCAN_RESULT_EVT) return;
    if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) return;

    uint8_t *adv = param->scan_rst.ble_adv;
    uint8_t len = param->scan_rst.adv_data_len;
    uint8_t pos = 0;
    bool uuid_found = false;

    while (pos < len) {
        uint8_t l = adv[pos];
        if (l == 0 || pos + l >= len) break;
        if (l >= 17 && memcmp(&adv[pos + 2], TARGET_UUID, 16) == 0) {
            uuid_found = true;
            break;
        }
        pos += l + 1;
    }

    if (!uuid_found) return;

    memcpy(beacon_mac, param->scan_rst.bda, 6);
    found_beacon = true;

    int rssi = param->scan_rst.rssi;
    rssi_sum += rssi;
    rssi_count++;
}

/* ================= WIFI ================= */
static void wifi_event_handler(void* arg, esp_event_base_t base,
                               int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
        update_display_line(3, " IP        : " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/* ================= UDP SYNC (solo al inicio) ================= */
static void udp_initial_sync(void) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) return;

    struct sockaddr_in local_addr;
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(UDP_PORT);
    local_addr.sin_addr.s_addr = INADDR_ANY;
    bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr));

    char buf[16];
    socklen_t addr_len = sizeof(local_addr);

    int len = recvfrom(sockfd, buf, sizeof(buf)-1, 0,
                       (struct sockaddr *)&local_addr, &addr_len);
    if (len > 0) {
        buf[len] = 0;
        current_window_id = atoll(buf);
    }
    close(sockfd);
}

/* ================= HTTP ================= */
static esp_err_t http_get_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[8];
        if (httpd_query_key_value(buf, "scan", val, sizeof(val)) == ESP_OK) {
            if (!strcmp(val, "on")) process_scan_results = true;
            if (!strcmp(val, "off")) process_scan_results = false;
            refresh_display();
        }
    }

    char resp[128];
    snprintf(resp, sizeof(resp),
             "{\"scanning\":%s,\"rssi_avg\":%d}",
             process_scan_results ? "true" : "false",
             last_avg_rssi);

    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_uri_t uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = http_get_handler
};

static void http_server_task(void* pvParameters) {
    httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    httpd_start(&server, &http_cfg);
    httpd_register_uri_handler(server, &uri);
    vTaskDelete(NULL);
}

/* ================= MAIN ================= */
void app_main(void) {
    nvs_flash_init();
    esp_read_mac(esp32_mac, ESP_MAC_WIFI_STA);

    // Dibuja layout después de boot logs
    draw_layout();

    // Wi-Fi
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_ip_info_t ip_info;
    inet_pton(AF_INET, "192.168.1.42", &ip_info.ip);
    inet_pton(AF_INET, "255.255.255.0", &ip_info.netmask);
    inet_pton(AF_INET, "192.168.1.1", &ip_info.gw);
    esp_netif_dhcpc_stop(sta_netif);
    esp_netif_set_ip_info(sta_netif, &ip_info);

    wifi_init_config_t cfg_wifi = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg_wifi);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                       wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                       wifi_event_handler, NULL, NULL);

    wifi_config_t sta_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD
        }
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    esp_wifi_start();

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // BLE
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_BLE;
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_ble_gap_register_callback(gap_event_handler);

    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x30,
        .scan_window = 0x20,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };
    esp_ble_gap_set_scan_params(&scan_params);
    esp_ble_gap_start_scanning(0);

    // UDP sync inicial
    udp_initial_sync();

    // HTTP server
    xTaskCreate(http_server_task, "http_server_task", 4096, NULL, 5, NULL);

    // Loop principal de ventanas RSSI
    int64_t last_window_time = esp_timer_get_time() / 1000; // ms
    while (1) {
        int64_t now = esp_timer_get_time() / 1000;
        int64_t elapsed = now - last_window_time;

        if (elapsed >= WINDOW_SIZE_MS) {
            if (rssi_count > 0) {
                last_avg_rssi = rssi_sum / rssi_count;
                refresh_display();
            }
            rssi_sum = 0;
            rssi_count = 0;
            current_window_id++;
            last_window_time += WINDOW_SIZE_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
