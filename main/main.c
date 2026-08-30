#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#define TWAI_SENDER_TX_GPIO     4
#define TWAI_SENDER_RX_GPIO     5
#define TWAI_QUEUE_DEPTH        10
#define TWAI_BITRATE            250000 

#define CAN_ID_AGUA             0x0C1
#define CAN_ID_FLUXO            0x0C2

static const char *TAG = "aquaponia_sender";

typedef struct {
    float ph;
    float temperatura;
} sensores_agua_t;

typedef struct {
    float fluxo;
    float tds;
} sensores_fluxo_t;

float simular_sensor(float min, float max) {
    float random_normalizado = (float)esp_random() / (float)UINT32_MAX;
    return min + (random_normalizado * (max - min));
}

static IRAM_ATTR bool twai_sender_tx_done_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx) {
    if (!edata->is_tx_success) {
        ESP_EARLY_LOGW(TAG, "Falha ao enviar, ID: 0x%" PRIx32, edata->done_tx_frame->header.id);
    }
    return false;
}

static IRAM_ATTR bool twai_sender_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx) {
    ESP_EARLY_LOGW(TAG, "Erro no barramento: 0x%x", edata->err_flags.val);
    return false; 
}

void app_main(void)
{
    twai_node_handle_t sender_node = NULL;
    printf("=================== Iniciando Simulador de Aquaponia ===================\n");

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

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &sender_node));

    twai_event_callbacks_t callbacks = {
        .on_tx_done = twai_sender_tx_done_callback,
        .on_error = twai_sender_on_error_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(sender_node, &callbacks, NULL));
    ESP_ERROR_CHECK(twai_node_enable(sender_node));

    ESP_LOGI(TAG, "Transmissor CAN iniciado a 100 kbps");

    while (1) {
        uint64_t timestamp = esp_timer_get_time();

        //Gerar os dados dos sensores
        sensores_agua_t dados_agua = {
            .ph = simular_sensor(6.5, 7.5),
            .temperatura = simular_sensor(22.0, 26.0)
        };

        sensores_fluxo_t dados_fluxo = {
            .fluxo = simular_sensor(10.0, 15.0),
            .tds = simular_sensor(300.0, 500.0)
        };

        twai_frame_t frame_agua = {
            .header.id = CAN_ID_AGUA,
            .buffer = (uint8_t *) &dados_agua,
            .buffer_len = sizeof(sensores_agua_t),
        };
        ESP_ERROR_CHECK(twai_node_transmit(sender_node, &frame_agua, pdMS_TO_TICKS(100)));

        twai_frame_t frame_fluxo = {
            .header.id = CAN_ID_FLUXO,
            .buffer = (uint8_t *) &dados_fluxo,
            .buffer_len = sizeof(sensores_fluxo_t),
        };
        ESP_ERROR_CHECK(twai_node_transmit(sender_node, &frame_fluxo, pdMS_TO_TICKS(100)));

        printf("[%" PRIu64 " us] Enviado -> ID 0x%03X [pH: %.2f | Temp: %.2f C]\n", timestamp, CAN_ID_AGUA, dados_agua.ph, dados_agua.temperatura);
        printf("[%" PRIu64 " us] Enviado -> ID 0x%03X [Fluxo: %.2f L/m | TDS: %.0f ppm]\n", timestamp, CAN_ID_FLUXO, dados_fluxo.fluxo, dados_fluxo.tds);

        ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(sender_node, -1));

        // Verificação de segurança (Bus-off)
        twai_node_status_t status;
        twai_node_get_info(sender_node, &status, NULL);
        if (status.state == TWAI_ERROR_BUS_OFF) {
            ESP_LOGW(TAG, "Bus-off detectado! Verifique os cabos e o Listener.");
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
