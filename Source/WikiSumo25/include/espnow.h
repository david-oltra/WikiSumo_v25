#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <nvs_flash.h>

#define ESP_CHANNEL 1

/* WIKISUMO MAC */
// static uint8_t peer_mac [ESP_NOW_ETH_ALEN] = {0x3c, 0x84, 0x27, 0xad, 0x5b, 0x44};

/* REMOTE MAC */
static uint8_t peer_mac [ESP_NOW_ETH_ALEN] = {0xdc, 0x06, 0x75, 0x67, 0xf1, 0x48};

typedef struct remote_data_t {
    uint8_t status;
    uint16_t joystick_Y;
    uint16_t joystick_X;
    uint8_t start = 0;
    uint8_t strategy;
    uint8_t disable_vl53l0x_error = 0;
    uint8_t disable_qre1113_error = 0;
    uint8_t calibrate_qre1113 = 0;
} remote_data_t;
remote_data_t remote_data;

typedef struct led_struct_t {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} led_struct_t;

typedef struct robot_data_t {
    uint8_t status;
    uint8_t start;
    uint8_t strategy;
    uint8_t vl53l0x_error = 0;
    uint8_t qre1113_error = 0;
    uint8_t qre1113_calibrate = 0;
    led_struct_t led_0;
    led_struct_t led_1;
    led_struct_t led_2;
} robot_data_t;
robot_data_t robot_data;

static const char * TAG_ESP_NOW = "ESPNOW";

static esp_err_t init_wifi(void)
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    
    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_wifi_init(&wifi_init_config);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start();

    ESP_LOGI(TAG_ESP_NOW, "wifi init completed");
    return ESP_OK;
}

void recv_cb(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len)
{
    // ESP_LOGI(TAG_ESP_NOW, "Data received " MACSTR "%s", MAC2STR(esp_now_info->src_addr), data);

    memcpy(&remote_data, data, sizeof(remote_data));
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if(status == ESP_NOW_SEND_SUCCESS)
    {
        // ESP_LOGI(TAG_ESP_NOW, "ESP_NOW_SEND_SUCCESS");
        remote_data.status = 1;
        robot_data.status = 1;
    }
    else
    {
        // ESP_LOGW(TAG_ESP_NOW, "ESP_NOW_SEND_FAIL");
        remote_data.status = 0;
        robot_data.status = 0;
    }
}

static esp_err_t init_esp_now(void)
{
    init_wifi();
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);

    ESP_LOGI(TAG_ESP_NOW, "esp now init completed");
    return ESP_OK;
}

static esp_err_t register_peer(uint8_t *peer_addr)
{
    esp_now_peer_info_t esp_now_peer_info;
    memset(&esp_now_peer_info, 0, sizeof(esp_now_peer_info_t));
    esp_now_peer_info.channel = ESP_CHANNEL;
    esp_now_peer_info.ifidx = WIFI_IF_STA;
    esp_now_peer_info.encrypt = false;
    memcpy(esp_now_peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    esp_now_add_peer(&esp_now_peer_info);

    return ESP_OK;
}

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    esp_now_send(peer_addr, data, len);

    return ESP_OK;
}
