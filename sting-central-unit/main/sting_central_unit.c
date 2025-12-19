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
#define UART_EVENT_QUEUE_SIZE 20

static const char *TAG = "MASTER_UNIT";

static QueueHandle_t uart_event_queue = NULL;


#define CONFIG_FREERTOS_HZ 100
#define STACK_SIZE 4096
#define TASK_PRIORITY_ESP 3
#define TASK_PRIORITY_UART 3

// MAC addresses for target nodes (ID 0–3)
static const uint8_t peer_macs[4][ESP_NOW_ETH_ALEN] = {
  {0xC0, 0x4E, 0x30, 0x37, 0x29, 0x94}, // Node 0
  {0xC0, 0x4E, 0x30, 0x3A, 0x0F, 0x34}, // Node 1
  {0xC0, 0x4E, 0x30, 0x37, 0x45, 0x2C}, // Node 2
  {0xC0, 0x4E, 0x30, 0x37, 0x19, 0x40}  // Node 3
};


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


  esp_now_peer_info_t peer_info = {
    .channel = 0,
    .encrypt = false
  };

  for (int i = 0; i < 4; i++) {
    memcpy(peer_info.peer_addr, peer_macs[i], ESP_NOW_ETH_ALEN);

    if (!esp_now_is_peer_exist(peer_info.peer_addr)) {
      ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
      ESP_LOGI(TAG, "Added peer %d -> %02X:%02X:%02X:%02X:%02X:%02X",
               i,
               peer_info.peer_addr[0], peer_info.peer_addr[1], peer_info.peer_addr[2],
               peer_info.peer_addr[3], peer_info.peer_addr[4], peer_info.peer_addr[5]);
    }
  }
}

void uart_receive_task(void *arg)
{
  char line[BUF_SIZE];
  int idx = 0;
  uart_event_t event;
  bool overflow_state = false;
  int overflow_count = 0;

  // Error statistics
  uint32_t frame_errors = 0;
  uint32_t parity_errors = 0;
  uint32_t buffer_overflows = 0;
  uint32_t fifo_overflows = 0;
  uint32_t event_counter = 0;

  while (1)
  {
    // Wait for UART event (blocking)
    if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) {

      switch (event.type) {
        case UART_DATA:
          // Normal case: data available to read
          {
            uint8_t byte;
            int len = uart_read_bytes(UART_PORT, &byte, 1, pdMS_TO_TICKS(100));

            if (len < 0) {
              ESP_LOGE(TAG, "UART read failed after DATA event");
              continue;
            } else if (len == 0) {
              // Timeout - event was spurious or data already consumed
              continue;
            }

            // Check for newline - this always resets state
            if (byte == '\n') {
              if (overflow_state) {
                // We were in overflow - report and reset
                ESP_LOGW(TAG, "Buffer overflow: Discarded %d bytes (max line length: %d)",
                         overflow_count, BUF_SIZE - 1);
                overflow_state = false;
                overflow_count = 0;
                idx = 0;
                continue;  // Don't process this corrupted message
              }

              // Normal newline - process the complete line
              line[idx] = '\0';

              // Parse the CSV format: move,id,x,y,yaw,xt,yt
              float x, y, yaw, xt, yt;
              int move, id;
              int ret = sscanf(line, "%d,%d,%f,%f,%f,%f,%f", &move, &id, &x, &y, &yaw, &xt, &yt);

              if (ret == 7) {
                ESP_LOGI(TAG, "Received: move=%d, id=%d, x=%.3f, y=%.3f, yaw=%.3f, xt=%.3f, yt=%.3f",
                         move, id, x, y, yaw, xt, yt);

                // Build payload and send
                payload_node_t payload = {
                  .move = move,
                  .id = id,
                  .x_value = x,
                  .y_value = y,
                  .yaw_value = yaw,
                  .xt_value = xt,
                  .yt_value = yt
                };

                if (id >= 0 && id < 4) {
                  ESP_LOGI(TAG, "Sending to Node %d (%02X:%02X:%02X:%02X:%02X:%02X)",
                           id,
                           peer_macs[id][0], peer_macs[id][1], peer_macs[id][2],
                           peer_macs[id][3], peer_macs[id][4], peer_macs[id][5]);

                  esp_err_t result = esp_now_send(peer_macs[id], (uint8_t *)&payload, sizeof(payload));
                  if (result != ESP_OK) {
                    ESP_LOGE(TAG, "ESP-NOW send failed to Node %d: %d", id, result);
                  }
                } else {
                  ESP_LOGW(TAG, "Invalid robot ID: %d (valid range: 0-3)", id);
                }
              } else {
                ESP_LOGW(TAG, "Parse failed (%d/7 fields): \"%s\"", ret, line);
              }

              // Reset for next message
              idx = 0;
              continue;
            }

            // Not a newline - check if we have space
            if (idx < (BUF_SIZE - 1)) {
              // Normal case: room in buffer
              line[idx++] = (char)byte;
            } else {
              // Buffer full - enter overflow state
              if (!overflow_state) {
                // First overflow byte
                ESP_LOGW(TAG, "Buffer overflow detected - discarding bytes until newline");
                overflow_state = true;
                overflow_count = 0;
              }
              overflow_count++;
            }
          }
          break;

        case UART_FIFO_OVF:
          // Hardware FIFO overflow - data arriving too fast
          fifo_overflows++;
          ESP_LOGW(TAG, "UART FIFO overflow (total: %lu) - flushing", fifo_overflows);
          uart_flush_input(UART_PORT);
          xQueueReset(uart_event_queue);
          idx = 0;
          overflow_state = false;
          break;

        case UART_BUFFER_FULL:
          // Ring buffer overflow - task not processing fast enough
          buffer_overflows++;
          ESP_LOGW(TAG, "UART ring buffer overflow (total: %lu) - flushing", buffer_overflows);
          uart_flush_input(UART_PORT);
          xQueueReset(uart_event_queue);
          idx = 0;
          overflow_state = false;
          break;

        case UART_BREAK:
          // Break signal detected
          ESP_LOGI(TAG, "UART break signal detected");
          idx = 0;
          overflow_state = false;
          break;

        case UART_PARITY_ERR:
          // Parity error - electrical noise or wrong config
          parity_errors++;
          ESP_LOGE(TAG, "UART parity error (total: %lu) - check wiring", parity_errors);
          break;

        case UART_FRAME_ERR:
          // Framing error - baud rate mismatch or noise
          frame_errors++;
          ESP_LOGE(TAG, "UART framing error (total: %lu) - check baud rate", frame_errors);
          break;

        case UART_PATTERN_DET:
          // Pattern detection (not used)
          ESP_LOGI(TAG, "UART pattern detected");
          break;

        default:
          ESP_LOGW(TAG, "Unknown UART event type: %d", event.type);
          break;
      }

      // Periodic diagnostics (every 1000 events)
      if (++event_counter % 1000 == 0) {
        ESP_LOGI(TAG, "UART Stats - Frame:%lu Parity:%lu BufferOvf:%lu FifoOvf:%lu",
                 frame_errors, parity_errors, buffer_overflows, fifo_overflows);
      }
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
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));

  // 2. Use default pins for USB-UART
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT,
                               UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));

  // 3. Install driver with event queue for error monitoring
  ESP_ERROR_CHECK(uart_driver_install(
    UART_PORT,
    BUF_SIZE * 2,           // RX buffer size (530 bytes)
    BUF_SIZE * 2,           // TX buffer size (enable for diagnostics)
    UART_EVENT_QUEUE_SIZE,  // Event queue size
    &uart_event_queue,      // Event queue handle
    0));

  ESP_LOGI(TAG, "UART initialized with event queue monitoring");
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

  xTaskCreate(uart_receive_task,
              "uart_receive_task",
              4096, // Increased stack size for UART task
              NULL, 
              TASK_PRIORITY_UART, 
              NULL);

}
