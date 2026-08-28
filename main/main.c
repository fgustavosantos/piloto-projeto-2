#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#define TWAI_SENDER_TX_GPIO     19
#define TWAI_SENDER_RX_GPIO     20
#define TWAI_QUEUE_DEPTH        10
#define TWAI_BITRATE            100000

// Message IDs
#define TWAI_DATA_ID            0x100
#define TWAI_HEARTBEAT_ID       0x7FF
#define TWAI_DATA_LEN           100

#define TWAI_SENSOR_1		0x123
#define TWAI_SENSOR_2           0x456

static const char *TAG = "twai_sender";

typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} twai_sender_data_t;

// Transmission completion callback
static IRAM_ATTR bool twai_sender_tx_done_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    if (!edata->is_tx_success) {
        ESP_EARLY_LOGW(TAG, "Failed to transmit message, ID: 0x%" PRIx32, edata->done_tx_frame->header.id);
    }
    return false; // No task wake required
}

// Bus error callback
static IRAM_ATTR bool twai_sender_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    ESP_EARLY_LOGW(TAG, "TWAI node error: 0x%x", edata->err_flags.val);
    return false; // No task wake required
}

void app_main(void){
    twai_node_handle_t sender_node = NULL;
    printf("===================TWAI Sender Example Starting...===================\n");

    // Configure TWAI node
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = TWAI_SENDER_TX_GPIO,
            .rx = TWAI_SENDER_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = TWAI_BITRATE,
        },
        .fail_retry_cnt = 3,
        .tx_queue_depth = TWAI_QUEUE_DEPTH,
    };

    // Create TWAI node
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &sender_node));

    // Register transmission completion callback
    twai_event_callbacks_t callbacks = {
        .on_tx_done = twai_sender_tx_done_callback,
        .on_error = twai_sender_on_error_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(sender_node, &callbacks, NULL));

    // Enable TWAI node
    ESP_ERROR_CHECK(twai_node_enable(sender_node));
    ESP_LOGI(TAG, "TWAI Sender started successfully");
  
    while (1) {
        uint64_t timestamp = esp_timer_get_time();

	uint8_t data_buffer[] = {1, 2, 3, 4};

        twai_frame_t tx_frame = {
            .header.id = TWAI_SENSOR_1,
            .buffer = (uint8_t *) &data_buffer,
            .buffer_len = sizeof(data_buffer),
        };
        ESP_ERROR_CHECK(twai_node_transmit(sender_node, &tx_frame, 500));
        
        // Log via ESP_LOGI
        ESP_LOGI(TAG, "Sending Sensor 1 message: %" PRIu64, timestamp);
        
        // Log via printf para visualização clara
        printf("[TWAI TX] Heartbeat - ID: 0x%03X | Timestamp: %" PRIu64 " us\n", 
               TWAI_HEARTBEAT_ID, timestamp);
        
        ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(sender_node, -1)); // -1 means wait forever
        
        
        vTaskDelay(pdMS_TO_TICKS(100));
        twai_node_status_t status;
        twai_node_get_info(sender_node, &status, NULL);
        if (status.state == TWAI_ERROR_BUS_OFF) {
            ESP_LOGW(TAG, "Bus-off detected");
            printf("[TWAI TX] ERROR: Bus-off detected! Exiting...\n");
            return;
        }
    }

    ESP_ERROR_CHECK(twai_node_disable(sender_node));
    ESP_ERROR_CHECK(twai_node_delete(sender_node));
}
