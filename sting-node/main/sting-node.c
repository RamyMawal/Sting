#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_gpio.h"
#include "nvs_flash.h"
#include "motor.h"

#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_crc.h"
#include <string.h>

#include "sdkconfig.h"
#include "esp_node_config.h"

#define QUEUE_SIZE 8
#define STACK_SIZE 2048
#define TASK_PRIORITY_DEFAULT 5
#define CONFIG_FREERTOS_HZ 100

QueueHandle_t s_esp_now_queue;

static const char *TAG = "NODE_UNIT";

void process_queue_task(void *pvParameters)
{
    example_payload_t message;

    while (1)
    {
        if (xQueueReceive(s_esp_now_queue, &message, portMAX_DELAY) == pdTRUE)
        {
            // ESP_LOGI("ESP-NOW", "Received data from %02x:%02x:%02x:%02x:%02x:%02x, Length: %d",
            //          message.mac_addr[0], message.mac_addr[1], message.mac_addr[2],
            //          message.mac_addr[3], message.mac_addr[4], message.mac_addr[5],
            //          message.len);

            ESP_LOGI(TAG, "Received data with motor state on, and speed value: %d", (int)message.speed_value);

            printf("Motor Speed: %u", (int)message.speed_value);
            printf("Motor On: %s", message.motor_on ? "Yes" : "No");

            if (message.motor_on == false)
            {
                control_motor_stop();
                continue;
            }

            switch (message.event_id)
            {
            case MOVE_FORWARD_CB:
                move_forward();
                update_speed((float)message.speed_value);
                ESP_LOGI("ESP-NOW", "Motor Direction: MOVE_FORWARD");
                break;
            case MOVE_BACKWARD_CB:
                move_backward();
                update_speed((float)message.speed_value);
                ESP_LOGI("ESP-NOW", "Motor Direction: MOVE_BACKWARD");
                break;
            default:
                ESP_LOGW("ESP-NOW", "Unknown Motor Direction!");
                break;
            }
        }
    }
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

static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI("ESP-NOW", "Send Status to MAC %02x:%02x:%02x:%02x:%02x:%02x: %s",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5],
             status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

static void on_data_recv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    ESP_LOGI("ESP-NOW", "Received data from MAC %02x:%02x:%02x:%02x:%02x:%02x, Length: %d",
             esp_now_info->des_addr[0], esp_now_info->des_addr[1], esp_now_info->des_addr[2],
             esp_now_info->des_addr[3], esp_now_info->des_addr[4], esp_now_info->des_addr[5],
             len);

    example_payload_t *payload = malloc(sizeof(example_payload_t));

    memcpy((uint8_t *)payload, data, len);

    if (xQueueSend(s_esp_now_queue, &payload, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Queue is full, dropping message");
    }

    // Example: Print received data
    ESP_LOGI("ESP-NOW", "Data: %.*s", len, data);
}

static esp_err_t app_espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_esp_now_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    return ESP_OK;

    // Set primary peer (for unicast communication)
    // esp_now_peer_info_t peer_info = {};
    // memcpy(peer_info.peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN); // Replace <peer_mac> with actual MAC address
    // peer_info.channel = 0;  // Default channel
    // peer_info.encrypt = false;

    // if (!esp_now_is_peer_exist(peer_info.peer_addr)) {
    //     ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    // }
}

void movement_test(void *pvParameters)
{
    move_forward();
    vTaskDelay(pdMS_TO_TICKS(1000));

    move_backward();
    vTaskDelay(pdMS_TO_TICKS(1000));

    control_motor_stop();
    vTaskDelay(pdMS_TO_TICKS(100));

    move_right();
    vTaskDelay(pdMS_TO_TICKS(1000));

    control_motor_stop();
    vTaskDelay(pdMS_TO_TICKS(100));

    move_left();
    vTaskDelay(pdMS_TO_TICKS(1000));
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

    s_esp_now_queue = xQueueCreate(QUEUE_SIZE, sizeof(example_payload_t));
    if (s_esp_now_queue == NULL)
    {
        ESP_LOGE("ESP-NOW", "Failed to create queue!");
        return;
    }

    app_wifi_init();
    app_espnow_init();

    setup_motor_gpio();
    setup_mcpwm();

    // xTaskCreate(movement_test,
    //             "Movement Test",
    //             STACK_SIZE,
    //             NULL,
    //             TASK_PRIORITY_DEFAULT,
    //             NULL);

    xTaskCreate(process_queue_task,
                "Process Queue Task",
                STACK_SIZE,
                NULL,
                TASK_PRIORITY_DEFAULT,
                NULL);
}