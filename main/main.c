#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include <string.h>
#include <sys/param.h>
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

static const char *TAG = "AQUAPONIA_MAIN";

#define TWAI_SENDER_TX_GPIO     GPIO_NUM_21
#define TWAI_SENDER_RX_GPIO     GPIO_NUM_26
#define TWAI_QUEUE_DEPTH        10
#define TWAI_BITRATE            1000000

// Message IDs
#define TWAI_DATA_ID            0x100
#define TWAI_HEARTBEAT_ID       0x7FF
#define TWAI_EMERGENCY_ID       0x080
#define TWAI_DATA_LEN           1000

// --- 1. DEFINIÇÕES DE HARDWARE ---
#define FLOW_SENSOR_PIN GPIO_NUM_18 
#define LM35_ADC_CHANNEL ADC_CHANNEL_7 // GPIO 4
#define PH_ADC_CHANNEL   ADC_CHANNEL_5 // GPIO 5
#define TDS_ADC_CHANNEL  ADC_CHANNEL_6 // GPIO 6

// --- 2. VARIÁVEIS GLOBAIS E ESTRUTURAS ---
volatile uint32_t pulse_count = 0;
adc_oneshot_unit_handle_t adc1_handle; 

SemaphoreHandle_t mutex_dados; // Chave de proteção para as variáveis

typedef struct {
    float temperatura;
    float ph;
    float tds;
    float fluxo;
} AquaponiaDados;

AquaponiaDados dados_atuais = {0}; // Armazena a última leitura válida

// --- 3. INTERRUPÇÃO (ISR) ---
static void IRAM_ATTR flow_sensor_isr_handler(void* arg) {
    pulse_count++;
}

// --- 4. INICIALIZAÇÃO DE HARDWARE ---
void init_adc() {
    ESP_LOGI(TAG, "Inicializando ADC Oneshot...");
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12, 
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LM35_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PH_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, TDS_ADC_CHANNEL, &config));
}

void init_gpio_flow_sensor() {
    ESP_LOGI(TAG, "Inicializando GPIO do Sensor de Fluxo...");
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FLOW_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE 
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_install_isr_service(0);
    gpio_isr_handler_add(FLOW_SENSOR_PIN, flow_sensor_isr_handler, NULL);
}

// --- 5. TASKS DO FREERTOS ---
void task_leitura_temperatura(void *pvParameters) {
    int raw_val;
    float tensao_mV, temp_calc;

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LM35_ADC_CHANNEL, &raw_val));
        
        tensao_mV = ((float)raw_val / 4095.0) * 3300.0; 
        float tensao_sensor_mV = tensao_mV / 7.8; // Ganho do Op-Amp
        temp_calc = (tensao_sensor_mV - 59.483) / 6.983; // Regressão

        if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(100)) == pdTRUE) {
            dados_atuais.temperatura = temp_calc;
            xSemaphoreGive(mutex_dados);
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Lê a cada 5s
    }
}

void task_leitura_rapida(void *pvParameters) {
    int raw_ph, raw_tds;
    float tensao_ph_V, tensao_tds_V, ph_calc, tds_calc, fluxo_calc;

    while (1) {
        adc_oneshot_read(adc1_handle, PH_ADC_CHANNEL, &raw_ph);
        adc_oneshot_read(adc1_handle, TDS_ADC_CHANNEL, &raw_tds);

        tensao_ph_V = ((float)raw_ph / 4095.0) * 3.3;
        tensao_tds_V = ((float)raw_tds / 4095.0) * 3.3;

        ph_calc = 42.018 - (7.931 * tensao_ph_V); // Regressão pH
        tds_calc = 784.31 * tensao_tds_V;         // Regressão TDS

        uint32_t pulsos_atuais = pulse_count;
        pulse_count = 0; 
        fluxo_calc = ((float)pulsos_atuais / 7.5); // Conversão Fluxo

        if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(10)) == pdTRUE) {
            dados_atuais.ph = ph_calc;
            dados_atuais.tds = tds_calc;
            dados_atuais.fluxo = fluxo_calc;
            xSemaphoreGive(mutex_dados);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Lê a cada 1s
    }
}

void task_serial_debug(void *pvParameters) {
    AquaponiaDados dados_copia;

    while (1) {
        if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(100)) == pdTRUE) {
            dados_copia = dados_atuais;
            xSemaphoreGive(mutex_dados);
            
            ESP_LOGI(TAG, "TDS: %.1f ppm | pH: %.2f | Temp: %.1f C | Vazao: %.1f L/min", 
                     dados_copia.tds, dados_copia.ph, dados_copia.temperatura, dados_copia.fluxo);
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Imprime a cada 2s
    }
}

static const char *TAG = "twai_sender";

typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} twai_sender_data_t;

// Transmission completion callback
static IRAM_ATTR bool twai_sender_tx_done_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    if (!edata->is_tx_success) {
        ESP_EARLY_LOGW(TAG, "Failed to transmit message, ID: 0x%X", edata->done_tx_frame->header.id);
    }
    return false; // No task wake required
}

// Bus error callback
static IRAM_ATTR bool twai_sender_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    ESP_EARLY_LOGW(TAG, "TWAI node error: 0x%x", edata->err_flags.val);
    return false; // No task wake required
}


// --- 6. APP_MAIN (PONTO DE ENTRADA) ---
void app_main(void) {
    ESP_LOGI(TAG, "--- Iniciando Firmware da Aquaponia ---");

    // A. Inicializa o hardware primeiro
    init_adc();
    init_gpio_flow_sensor();

    // B. Cria o Mutex ANTES de criar as Tasks (MUITO IMPORTANTE)
    mutex_dados = xSemaphoreCreateMutex();
    if (mutex_dados == NULL) {
        ESP_LOGE(TAG, "Falha ao criar o Mutex!");
        return; 
    }

    // C. Dispara as Tasks, dividindo-as entre os núcleos
    // Core 1: Focado em aquisição de dados
    xTaskCreatePinnedToCore(task_leitura_temperatura, "TaskTemp", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(task_leitura_rapida, "TaskRapida", 4096, NULL, 4, NULL, 1);
    
    // Core 0: Focado em comunicação (Serial)
    xTaskCreatePinnedToCore(task_serial_debug, "TaskSerial", 4096, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "Tasks disparadas com sucesso!");

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
    ESP_LOGI(TAG, "Sending messages with IDs: 0x%03X (data), 0x%03X (heartbeat)",  TWAI_DATA_ID, TWAI_HEARTBEAT_ID);

    while (1) {
        AquaponiaDados dados_envio;

        // 1. Pega os dados mais recentes de forma segura
        if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(100)) == pdTRUE) {
            dados_envio = dados_atuais;
            xSemaphoreGive(mutex_dados);
        } else {
            ESP_LOGW(TAG, "Falha ao pegar o mutex para envio CAN");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 2. Monta o Frame 1 (Temperatura e pH)
        twai_frame_t frame1 = {
            .header.id = 0x101, // ID específico para Temp e pH
            .buffer_len = 8,    // 4 bytes do float Temp + 4 bytes do float pH
        };
        memcpy(&frame1.buffer[0], &dados_envio.temperatura, sizeof(float));
        memcpy(&frame1.buffer[4], &dados_envio.ph, sizeof(float));

        // 3. Monta o Frame 2 (TDS e Fluxo)
        twai_frame_t frame2 = {
            .header.id = 0x102, // ID específico para TDS e Fluxo
            .buffer_len = 8,
        };
        memcpy(&frame2.buffer[0], &dados_envio.tds, sizeof(float));
        memcpy(&frame2.buffer[4], &dados_envio.fluxo, sizeof(float));

        // 4. Transmite os frames na rede CAN
        esp_err_t err1 = twai_node_transmit(sender_node, &frame1, pdMS_TO_TICKS(500));
        esp_err_t err2 = twai_node_transmit(sender_node, &frame2, pdMS_TO_TICKS(500));

        if (err1 == ESP_OK && err2 == ESP_OK) {
            ESP_LOGI(TAG, "Dados enviados via CAN com sucesso!");
        } else {
            ESP_LOGW(TAG, "Falha ao enviar frames CAN. Err1: %d, Err2: %d", err1, err2);
        }

        // Verifica status da rede (Bus-Off)
        twai_node_status_t status;
        twai_node_get_info(sender_node, &status, NULL);
        if (status.state == TWAI_ERROR_BUS_OFF) {
            ESP_LOGE(TAG, "Bus-off detectado! Reiniciando nó...");
            twai_node_disable(sender_node);
            twai_node_enable(sender_node);
        }

        // Aguarda 2 segundos antes do próximo envio
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_ERROR_CHECK(twai_node_disable(sender_node));
    ESP_ERROR_CHECK(twai_node_delete(sender_node));
}