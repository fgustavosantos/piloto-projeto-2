#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#define TWAI_LISTENER_TX_GPIO   4
#define TWAI_LISTENER_RX_GPIO   5
#define TWAI_QUEUE_DEPTH        10
#define TWAI_BITRATE            250000

#define CAN_ID_AGUA             0x0C1
#define CAN_ID_FLUXO            0x0C2

static const char *TAG = "aquaponia_listener";

typedef struct {
    float ph;
    float temperatura;
} sensores_agua_t;

typedef struct {
    float fluxo;
    float tds;
} sensores_fluxo_t;

typedef struct {
    uint32_t id;
    uint8_t data[8];
} rx_msg_t;

static IRAM_ATTR bool twai_listener_on_rx_done_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx) {
    QueueHandle_t rx_queue = (QueueHandle_t)user_ctx;
    bool need_yield = false;
    
    for (int i = 0; i < 10; i++) {
        rx_msg_t msg;
        twai_frame_t rx_frame = {
            .buffer = msg.data,
            .buffer_len = sizeof(msg.data)
        };

        if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
            break; 
        }
        
        msg.id = rx_frame.header.id;
        
        BaseType_t task_woken = pdFALSE;
        xQueueSendFromISR(rx_queue, &msg, &task_woken);
        if (task_woken == pdTRUE) {
            need_yield = true;
        }
    }
    return need_yield;
}

// Callbacks vazios obrigatórios
static IRAM_ATTR bool twai_listener_tx_done_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx) { return false; }
static IRAM_ATTR bool twai_listener_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx) { return false; }

void app_main(void)
{
    twai_node_handle_t listen_node = NULL;
    printf("=================== Iniciando Receptor de Aquaponia ===================\n");

    QueueHandle_t rx_queue = xQueueCreate(10, sizeof(rx_msg_t));

    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = TWAI_LISTENER_TX_GPIO, 
            .rx = TWAI_LISTENER_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = TWAI_BITRATE,
        },
        .fail_retry_cnt = 3,
        .tx_queue_depth = TWAI_QUEUE_DEPTH,
        .flags.enable_listen_only = false,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &listen_node));

    twai_mask_filter_config_t filter = {
        .id = 0x000,
        .mask = 0x000, 
        .is_ext = false,
    };
    ESP_ERROR_CHECK(twai_node_config_mask_filter(listen_node, 0, &filter));

    twai_event_callbacks_t callbacks = {
        .on_tx_done = twai_listener_tx_done_callback,
        .on_rx_done = twai_listener_on_rx_done_callback,
        .on_error = twai_listener_on_error_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(listen_node, &callbacks, rx_queue));
    
    ESP_ERROR_CHECK(twai_node_enable(listen_node));
    ESP_LOGI(TAG, "Receptor CAN iniciado (Listen Only) a 250 kbps. Aguardando dados...");

    while (1) {
        rx_msg_t msg_recebida;
        
        if (xQueueReceive(rx_queue, &msg_recebida, portMAX_DELAY) == pdTRUE) {
            
            if (msg_recebida.id == CAN_ID_AGUA) {
                sensores_agua_t dados_agua;
                // Converte os 8 bytes de volta para 2 Floats
                memcpy(&dados_agua, msg_recebida.data, sizeof(sensores_agua_t));
                printf("[TWAI RX] -> pH: %.2f | Temp: %.2f C\n", dados_agua.ph, dados_agua.temperatura);
            } 
            else if (msg_recebida.id == CAN_ID_FLUXO) {
                sensores_fluxo_t dados_fluxo;
                // Converte os 8 bytes de volta para 2 Floats
                memcpy(&dados_fluxo, msg_recebida.data, sizeof(sensores_fluxo_t));
                printf("[TWAI RX] -> Fluxo: %.2f L/m | TDS: %.0f ppm\n", dados_fluxo.fluxo, dados_fluxo.tds);
            }
            else {
                ESP_LOGW(TAG, "ID Desconhecido detectado na rede: 0x%03" PRIx32, msg_recebida.id);
            }
        }
    }
}
