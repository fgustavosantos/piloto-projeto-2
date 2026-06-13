#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "AQUAPONIA_MAIN";

#define FLOW_SENSOR_PIN GPIO_NUM_18 
#define LM35_ADC_CHANNEL ADC_CHANNEL_3 // GPIO 4
#define PH_ADC_CHANNEL   ADC_CHANNEL_4 // GPIO 5
#define TDS_ADC_CHANNEL  ADC_CHANNEL_5 // GPIO 6

volatile uint32_t pulse_count = 0;
adc_oneshot_unit_handle_t adc1_handle; 
adc_cali_handle_t adc1_cali_handle = NULL; 
bool calib_calibrado = false;

SemaphoreHandle_t mutex_dados; 

typedef struct {
    float temperatura;
    float ph;
    float tds;
    float fluxo;
} AquaponiaDados;

AquaponiaDados dados_atuais = {0}; 

static void IRAM_ATTR flow_sensor_isr_handler(void* arg) {
    pulse_count++;
}

void init_adc() {
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

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);

}

void init_gpio_flow_sensor() {
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

int ler_adc_calibrado(adc_channel_t canal) {
    int raw_val;
    adc_oneshot_read(adc1_handle, canal, &raw_val);
    
    int tensao_mV = 0;

    adc_cali_raw_to_voltage(adc1_cali_handle, raw_val, &tensao_mV);

    return tensao_mV;
}

void task_leitura_qualidade(void *pvParameters) {
    float temp_calc, ph_calc, tds_calc;

    while (1) {
        int tensao_mV = ler_adc_calibrado(LM35_ADC_CHANNEL);

        int tensao_ph_mV = ler_adc_calibrado(PH_ADC_CHANNEL);
        int tensao_tds_mV = ler_adc_calibrado(TDS_ADC_CHANNEL);

        float tensao_ph_V = tensao_ph_mV / 1000.0;
        float tensao_tds_V = tensao_tds_mV / 1000.0;    

        float tensao_sensor_mV = (float)tensao_mV / 7.8; 
        temp_calc = (tensao_sensor_mV - 59.483) / 6.983;

        ph_calc = 42.018 - (7.931 * tensao_ph_V); 
        tds_calc = 784.31 * tensao_tds_V;     

        if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(100)) == pdTRUE) {
            dados_atuais.temperatura = temp_calc;
            dados_atuais.ph = ph_calc;
            dados_atuais.tds = tds_calc;
            xSemaphoreGive(mutex_dados);
        }
        vTaskDelay(pdMS_TO_TICKS(300000));
    }
}

void task_leitura_fluxo(void *pvParameters) {
    float fluxo_calc;

    while (1) {
        uint32_t pulsos_atuais = pulse_count;
        pulse_count = 0; 
        fluxo_calc = ((float)pulsos_atuais / 7.5);

        if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(10)) == pdTRUE) {
            dados_atuais.fluxo = fluxo_calc;
            xSemaphoreGive(mutex_dados);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

void task_serial(void *pvParameters) {
    AquaponiaDados dados_copia;

    while (1) {
        if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(100)) == pdTRUE) {
            dados_copia = dados_atuais;
            xSemaphoreGive(mutex_dados);
            
            ESP_LOGI(TAG, "TDS: %.1f ppm | pH: %.2f | Temp: %.1f C | Vazao: %.1f L/min", 
                     dados_copia.tds, dados_copia.ph, dados_copia.temperatura, dados_copia.fluxo);
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); 
    }
}

void app_main(void) {
    init_adc();
    init_gpio_flow_sensor();

    mutex_dados = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(task_leitura_qualidade, "TaskQualidade", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(task_leitura_fluxo, "TaskFluxo", 4096, NULL, 4, NULL, 1);
    
    xTaskCreatePinnedToCore(task_serial, "TaskSerial", 4096, NULL, 1, NULL, 0);
}