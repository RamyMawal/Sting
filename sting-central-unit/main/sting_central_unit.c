#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_crc.h"
#include <string.h>

#include "sdkconfig.h"


#include "esp_basic_config.h"


// static const char *TAG = "MASTER_UNIT";

QueueHandle_t test_queue;


#define CONFIG_FREERTOS_HZ 100
#define STACK_SIZE 2048
#define TASK_PRIORITY 3



static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI("ESP-NOW", "Send Status to MAC %02x:%02x:%02x:%02x:%02x:%02x: %s",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5],
             status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");

}

static void on_data_recv(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int len) {
    ESP_LOGI("ESP-NOW", "Received data from MAC %02x:%02x:%02x:%02x:%02x:%02x, Length: %d",
             esp_now_info->des_addr[0], esp_now_info->des_addr[1], esp_now_info->des_addr[2],
             esp_now_info->des_addr[3], esp_now_info->des_addr[4], esp_now_info->des_addr[5],
             len);
    

    // Example: Print received data
    ESP_LOGI("ESP-NOW", "Data: %.*s", len, data);
}

static void app_wifi_init(void)
{
   ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    assert(netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

}

static void app_espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN); // Replace <peer_mac> with actual MAC address
    peer_info.channel = 0;  // Default channel
    peer_info.encrypt = false;

    if (!esp_now_is_peer_exist(peer_info.peer_addr)) {
        ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    }
}

void task_send(void *pvParameters)
{
    example_payload_t *payload = malloc(sizeof(example_payload_t));
    
    printf("Initializing task_send \n");

    while (1)
    {
        printf("Send iteration started\n");
        
        payload->motor_on = true;

        for (size_t i = 10; i <= 100; i += 5)
        {
            payload->x_value = i * 0.01;
            esp_now_send(s_broadcast_mac, (uint8_t *)payload, sizeof(example_payload_t));
            printf("Sending speed up ... \n");
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  

        for (size_t i = 100; i >= 10; i -= 5)
        {
            payload->x_value = i * 0.01;
            esp_now_send(s_broadcast_mac, (uint8_t *)payload, sizeof(example_payload_t));
            printf("Sending slow down... \n");
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  
    }
}

void app_main()
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    app_wifi_init();

    app_espnow_init();

    xTaskCreate(task_send,
                "Speed send task",
                STACK_SIZE,
                NULL,
                TASK_PRIORITY,
                NULL);
}


