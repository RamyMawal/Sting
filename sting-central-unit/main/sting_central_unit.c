#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include <string.h>
#include "sdkconfig.h"
#include "esp_basic_config.h"

#define UART_PORT        UART_NUM_0
#define BUF_SIZE         265     // Enough for a line of text

static const char *TAG = "MASTER_UNIT";

QueueHandle_t test_queue;


#define CONFIG_FREERTOS_HZ 100
#define STACK_SIZE 4096
#define TASK_PRIORITY_ESP 3
#define TASK_PRIORITY_UART 3


static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Send Status to MAC %02x:%02x:%02x:%02x:%02x:%02x: %s",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5],
             status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");

}

static void on_data_recv(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int len) {
    ESP_LOGI(TAG, "Received data from MAC %02x:%02x:%02x:%02x:%02x:%02x, Length: %d",
             esp_now_info->des_addr[0], esp_now_info->des_addr[1], esp_now_info->des_addr[2],
             esp_now_info->des_addr[3], esp_now_info->des_addr[4], esp_now_info->des_addr[5],
             len);
    

    // Example: Print received data
    ESP_LOGI(TAG, "Data: %.*s", len, data);
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
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
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

void uart_receive_task(void *arg)
{
    char line[BUF_SIZE];
    int idx = 0;
    uint8_t byte;

    while (1) {
        // Block until we get a single byte
        int len = uart_read_bytes(UART_PORT, &byte, 1, portMAX_DELAY);
        if (len <= 0) {
            ESP_LOGI(TAG, "No data received");
            continue;
        }

        // If it's not newline, append (if space)
        if (byte != '\n' && idx < (BUF_SIZE - 1)) {
            line[idx++] = (char)byte;
        } else {
            // End of message: terminate and parse
            line[idx] = '\0';
            //Data format: "12.34,56.78,1.57,20.00,30.00\n"
            float x, y, yaw, xt, yt;
            int id;
            int ret = sscanf(line, "%d,%f,%f,%f,%f,%f",&id, &x, &y, &yaw, &xt, &yt);
            if (ret == 6) {
                ESP_LOGI(TAG,
                         "Received block:id=%d x=%.3f, y=%.3f, yaw=%.3f, xt=%.3f, yt=%.3f",
                         id, x, y, yaw, xt, yt);

                // Send data to ESP-NOW
                payload_node_t payload;
                payload.id = id;
                payload.x_value = x;
                payload.y_value = y;
                payload.yaw_value = yaw;
                payload.xt_value = xt;
                payload.yt_value = yt;
                ESP_LOGI(TAG, "Sending data to ESP-NOW");
                esp_now_send(s_broadcast_mac, (uint8_t *)&payload, sizeof(payload_node_t));
            } else {
                ESP_LOGW(TAG, "Failed to parse line (%d fields): \"%s\"", ret, line);
            }

            // Reset for next message
            idx = 0;
        }
    }
}

void task_send(void *pvParameters)
{
    payload_node_t *payload = malloc(sizeof(payload_node_t));
    if (payload == NULL) {
        ESP_LOGE(TAG, "Failed to allocate payload memory");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Initializing task_send \n");

    while (1)
    {
        ESP_LOGI(TAG, "Send iteration started\n");

        payload->x_value = 0;
        payload->y_value = 0;

        for (size_t i = 0; i <= 100; i += 5)
        {
            payload->x_value = i * 0.01;
            esp_now_send(s_broadcast_mac, (uint8_t *)payload, sizeof(payload_node_t));
            ESP_LOGI(TAG, "Sending speed up ...");
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  

        for (size_t i = 0; i <= 100; i += 5)
        {
            payload->y_value = i * 0.01;
            esp_now_send(s_broadcast_mac, (uint8_t *)payload, sizeof(payload_node_t));
            ESP_LOGI(TAG, "Sending speed up ...");
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  

        for (size_t i = 100; i > 0; i -= 5)
        {
            payload->y_value = i * 0.01;
            esp_now_send(s_broadcast_mac, (uint8_t *)payload, sizeof(payload_node_t));
            ESP_LOGI(TAG, "Sending slow down ...");
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        for (size_t i = 100; i > 0; i -= 5)
        {
            payload->x_value = i * 0.01;
            esp_now_send(s_broadcast_mac, (uint8_t *)payload, sizeof(payload_node_t));
            ESP_LOGI(TAG, "Sending slow down ...");
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void app_uart_init()
{
    // 1. Configure UART0 at 115200
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));

    // 2. Use default pins for USB-UART
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE));

    // 3. Install driver: RX buffer only
    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT,
        BUF_SIZE * 2,   // RX buffer size
        0,              // TX buffer (unused here)
        0, NULL, 0));

    // 4. Start the receive task
}

void app_main()
{
    ESP_LOGI(TAG, "Starting main application...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    app_uart_init();
    app_wifi_init();
    app_espnow_init();

    // xTaskCreate(task_send,
    //             "Speed send task",
    //             STACK_SIZE,
    //             NULL,
    //             TASK_PRIORITY_ESP,
    //             NULL);

    xTaskCreate(uart_receive_task,
                "uart_receive_task",
                4096, // Increased stack size for UART task
                NULL, 
                TASK_PRIORITY_UART, 
                NULL);
}