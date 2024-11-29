#ifndef ESPNOW_BASIC_CONFIG_H
#endif
#define ESPNOW_BASIC_CONFIG_H

#include <inttypes.h>
#include <stdbool.h>

#define TX_BROADCAST_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

#define ESPNOW_PMK "pmk123123"
#define ESPNOW_CHANNEL 1
#define ESPNOW_QUEUE_SIZE 8

#define NODE_DEEP_SLEEP_TIME_MS 10000

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

typedef enum {
    MOVE_FORWARD_CB,
    MOVE_BACKWARD_CB,

} event_motor_direction;


typedef struct __attribute__((packed))
{
    uint32_t speed_value;
    bool motor_on;
    event_motor_direction event_id;
} example_payload_t;

