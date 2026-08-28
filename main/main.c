/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#define TWAI_LISTENER_TX_GPIO   19  // Listen only node doesn't need TX pin
#define TWAI_LISTENER_RX_GPIO   20
#define TWAI_BITRATE            100000

// Message IDs (must match sender)
#define TWAI_DATA_ID            0x100
#define TWAI_HEARTBEAT_ID       0x7FF

// Buffer for burst data handling
#define POLL_DEPTH              200

static const char *TAG = "twai_listen";

typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} twai_listener_data_t;

typedef struct {
    twai_node_handle_t node_hdl;
    twai_listener_data_t *rx_pool;
    SemaphoreHandle_t free_pool_semaphore;
    SemaphoreHandle_t rx_result_semaphore;
    int write_idx;
    int read_idx;
} twai_listener_ctx_t;

// Error callback
static bool IRAM_ATTR twai_listener_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    ESP_EARLY_LOGW(TAG, "bus error: 0x%x", edata->err_flags.val);
    return false;
}

// Node state
static bool IRAM_ATTR twai_listener_on_state_change_callback(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx)
{
    const char *twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};
    ESP_EARLY_LOGI(TAG, "state changed: %s -> %s", twai_state_name[edata->old_sta], twai_state_name[edata->new_sta]);
    return false;
}

// TWAI receive callback - store data and signal
static bool IRAM_ATTR twai_listener_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t woken;
    twai_listener_ctx_t *ctx = (twai_listener_ctx_t *)user_ctx;

    if (xSemaphoreTakeFromISR(ctx->free_pool_semaphore, &woken) != pdTRUE) {
        ESP_EARLY_LOGI(TAG, "Pool full, dropping frame");
        return (woken == pdTRUE);
    }
    if (twai_node_receive_from_isr(handle, &ctx->rx_pool[ctx->write_idx].frame) == ESP_OK) {
        ctx->write_idx = (ctx->write_idx + 1) % POLL_DEPTH;
        xSemaphoreGiveFromISR(ctx->rx_result_semaphore, &woken);
    }
    return (woken == pdTRUE);
}

void app_main(void)
{
    printf("===================TWAI Listen Only Example Starting...===================\n");

    // Create semaphore for receive notification
    twai_listener_ctx_t twai_listener_ctx = {0};
    twai_listener_ctx.free_pool_semaphore = xSemaphoreCreateCounting(POLL_DEPTH, POLL_DEPTH);
    twai_listener_ctx.rx_result_semaphore = xSemaphoreCreateCounting(POLL_DEPTH, 0);
    assert(twai_listener_ctx.free_pool_semaphore != NULL);
    assert(twai_listener_ctx.rx_result_semaphore != NULL);

    twai_listener_ctx.rx_pool = calloc(POLL_DEPTH, sizeof(twai_listener_data_t));
    assert(twai_listener_ctx.rx_pool != NULL);
    for (int i = 0; i < POLL_DEPTH; i++) {
        twai_listener_ctx.rx_pool[i].frame.buffer = twai_listener_ctx.rx_pool[i].data;
        twai_listener_ctx.rx_pool[i].frame.buffer_len = sizeof(twai_listener_ctx.rx_pool[i].data);
    }
    ESP_LOGI(TAG, "Buffer initialized: %d slots for burst data", POLL_DEPTH);

    // Configure TWAI node
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = TWAI_LISTENER_TX_GPIO,
            .rx = TWAI_LISTENER_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing.bitrate = TWAI_BITRATE,
        .timestamp_resolution_hz = 1000000,
        .flags.enable_listen_only = true,
    };

    // Create TWAI node
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &twai_listener_ctx.node_hdl));
    ESP_LOGI(TAG, "TWAI node created");

    // Configure acceptance filter - ESP32-C6 aceita apenas 1 filtro
    // Usar máscara 0x000 para aceitar TODOS os IDs standard (0x000-0x7FF)
    // Filtragem específica de IDs será feita no software
    twai_mask_filter_config_t filter = {
        .id = 0x000,
        .mask = 0x000,          // Máscara 0x000 = Aceita TODOS os IDs
        .is_ext = false,        // Receive only standard ID
    };
    ESP_ERROR_CHECK(twai_node_config_mask_filter(twai_listener_ctx.node_hdl, 0, &filter));
    ESP_LOGI(TAG, "Filter enabled: Accepting ALL standard CAN IDs (0x000-0x7FF)");

    // Register callbacks
    twai_event_callbacks_t callbacks = {
        .on_rx_done = twai_listener_rx_callback,
        .on_error = twai_listener_on_error_callback,
        .on_state_change = twai_listener_on_state_change_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(twai_listener_ctx.node_hdl, &callbacks, &twai_listener_ctx));

    // Enable TWAI node
    ESP_ERROR_CHECK(twai_node_enable(twai_listener_ctx.node_hdl));
    ESP_LOGI(TAG, "TWAI start listening...");

    // Main loop - process all buffered data when signaled
    while (1) {
        if (xSemaphoreTake(twai_listener_ctx.rx_result_semaphore, portMAX_DELAY) == pdTRUE) {
            twai_frame_t *frame = &twai_listener_ctx.rx_pool[twai_listener_ctx.read_idx].frame;
            
            // Log usando ESP_LOGI
            ESP_LOGI(TAG, "RX: timestamp %" PRIu64 ", ID: 0x%03" PRIx32 " [DLC:%d]", \
                     frame->header.timestamp, frame->header.id, frame->header.dlc);
            
            // Exibir via printf com formatação clara
            printf("[TWAI RX] ");
            
            // Identificar tipo de mensagem
            if (frame->header.id == TWAI_DATA_ID) {
                printf("BURST DATA - ");
            } else if (frame->header.id == TWAI_HEARTBEAT_ID) {
                printf("HEARTBEAT  - ");
            } else {
                printf("UNKNOWN    - ");
            }
            
            printf("ID: 0x%03" PRIx32 " | Timestamp: %" PRIu64 " | DLC: %d | Data: ", 
                   frame->header.id, frame->header.timestamp, frame->header.dlc);
            
            // Exibir dados
            for (int i = 0; i < frame->header.dlc && i < TWAI_FRAME_MAX_LEN; i++) {
                printf("%02X ", frame->buffer[i]);
            }
            printf("\n");
            
            twai_listener_ctx.read_idx = (twai_listener_ctx.read_idx + 1) % POLL_DEPTH;
            xSemaphoreGive(twai_listener_ctx.free_pool_semaphore);
        }
    }

    // Cleanup
    vSemaphoreDelete(twai_listener_ctx.rx_result_semaphore);
    vSemaphoreDelete(twai_listener_ctx.free_pool_semaphore);
    free(twai_listener_ctx.rx_pool);
    ESP_ERROR_CHECK(twai_node_disable(twai_listener_ctx.node_hdl));
    ESP_ERROR_CHECK(twai_node_delete(twai_listener_ctx.node_hdl));
}
