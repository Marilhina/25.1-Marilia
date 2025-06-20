
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "int_i2c.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include <time.h>
#include "esp_log.h"

#define LED_GPIO     GPIO_NUM_17
#define LED_GPIO_2   GPIO_NUM_18
#define LED_GPIO_3   GPIO_NUM_8
#define LED_GPIO_4   GPIO_NUM_3
#define BOTAO_1      GPIO_NUM_4
#define BOTAO_2      GPIO_NUM_5
#define BUZZER_GPIO  GPIO_NUM_47
#define NTC_ADC_CHANNEL ADC1_CHANNEL_5

#define ALARME_INICIAL 25
#define BETA            3950
#define RESISTOR_FIXO   10000.0
#define V_REF           3.3
#define ADC_RESOLUCAO   4095.0
#define T0              298.15

#define MOUNT_POINT "/sdcard"
#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

#define BUZZER_TIMER        LEDC_TIMER_0
#define BUZZER_MODE         LEDC_LOW_SPEED_MODE
#define BUZZER_CHANNEL      LEDC_CHANNEL_0
#define BUZZER_DUTY_RES     LEDC_TIMER_13_BIT
#define BUZZER_FREQUENCY    2000
#define BUZZER_DUTY         (4095)

#define TASK_TEMP_PRIORITY      3
#define TASK_LED_PRIORITY       2
#define TASK_DISPLAY_PRIORITY   2
#define TASK_SD_PRIORITY        1
#define TASK_BUZZER_PRIORITY    4

typedef struct {
  int temperatura;
  int temperatura_alarme;
  bool alarme_ativo;
} temp_data_t;

TaskHandle_t task_temperature_handle = NULL;
TaskHandle_t task_led_control_handle = NULL;
TaskHandle_t task_display_handle = NULL;
TaskHandle_t task_sd_handle = NULL;
TaskHandle_t task_buzzer_handle = NULL;

QueueHandle_t temp_queue;
QueueHandle_t led_queue;
QueueHandle_t display_queue;
QueueHandle_t sd_queue;
QueueHandle_t buzzer_queue;
SemaphoreHandle_t i2c_mutex;

int temperatura_alarme = ALARME_INICIAL;
char aprox_temperatura = '0';
volatile bool botao1_pendente = false;
volatile bool botao2_pendente = false;

esp_timer_handle_t debounce_timer_botao1;
esp_timer_handle_t debounce_timer_botao2;

lcd_i2c_handle_t lcd = {
  .address = 0x27,
  .num = I2C_NUM_0,
  .backlight = 1,
  .size = DISPLAY_16X02
};

static const char *TAG = "TEMP_MONITOR";

void init_hardware(void);
void init_sdcard(void);
void init_pwm_buzzer(void);
void configurar_gpio(void);
void i2c_master_init(void);
int ler_temperatura_ntc(void);
void calcular_aproximacao_temp(int temp_ntc);

void debounce_callback_botao1(void* arg);
void debounce_callback_botao2(void* arg);
void IRAM_ATTR gpio_isr_handler_botao1(void* arg);
void IRAM_ATTR gpio_isr_handler_botao2(void* arg);

void task_temperature_monitor(void *pvParameter);
void task_led_control(void *pvParameter);
void task_display_update(void *pvParameter);
void task_sd_logger(void *pvParameter);
void task_buzzer_control(void *pvParameter);

void init_pwm_buzzer(void) {
  ledc_timer_config_t ledc_timer = {
    .duty_resolution = BUZZER_DUTY_RES,
    .freq_hz = BUZZER_FREQUENCY,
    .speed_mode = BUZZER_MODE,
    .timer_num = BUZZER_TIMER,
    .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {
    .channel    = BUZZER_CHANNEL,
    .duty       = 0,
    .gpio_num   = BUZZER_GPIO,
    .speed_mode = BUZZER_MODE,
    .hpoint     = 0,
    .timer_sel  = BUZZER_TIMER
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void init_sdcard() {
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };

  sdmmc_card_t* card;
  const char mount_point[] = MOUNT_POINT;

  ESP_LOGI(TAG, "Inicializando SD Card via SPI");
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  spi_bus_config_t bus_cfg = {
    .mosi_io_num = PIN_NUM_MOSI,
    .miso_io_num = PIN_NUM_MISO,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA));

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = PIN_NUM_CS;
  slot_config.host_id = host.slot;

  esp_err_t ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Falha ao montar SD Card: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "SD Card montado com sucesso");
}

void debounce_callback_botao1(void* arg) {
  if (gpio_get_level(BOTAO_1) == 0) {
    temperatura_alarme += 5;
    ESP_LOGI(TAG, "Temperatura de alarme ajustada para: %d°C", temperatura_alarme);
  }
  botao1_pendente = false;
}

void debounce_callback_botao2(void* arg) {
  if (gpio_get_level(BOTAO_2) == 0) {
    temperatura_alarme -= 5;
    ESP_LOGI(TAG, "Temperatura de alarme ajustada para: %d°C", temperatura_alarme);
  }
  botao2_pendente = false;
}

void IRAM_ATTR gpio_isr_handler_botao1(void* arg) {
  if (!botao1_pendente) {
    botao1_pendente = true;
    esp_timer_start_once(debounce_timer_botao1, 50000);
  }
}

void IRAM_ATTR gpio_isr_handler_botao2(void* arg) {
  if (!botao2_pendente) {
    botao2_pendente = true;
    esp_timer_start_once(debounce_timer_botao2, 50000);
  }
}

void i2c_master_init() {
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = 19,
    .scl_io_num = 20,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
}

void configurar_gpio() {
  gpio_config_t out_conf = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ULL << LED_GPIO) | (1ULL << LED_GPIO_2) |
    (1ULL << LED_GPIO_3) | (1ULL << LED_GPIO_4),
  };
  ESP_ERROR_CHECK(gpio_config(&out_conf));

  gpio_config_t in_conf = {
    .intr_type = GPIO_INTR_NEGEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL << BOTAO_1) | (1ULL << BOTAO_2),
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&in_conf));

  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(BOTAO_1, gpio_isr_handler_botao1, NULL));
  ESP_ERROR_CHECK(gpio_isr_handler_add(BOTAO_2, gpio_isr_handler_botao2, NULL));
}

int ler_temperatura_ntc() {
  int valor_adc = adc1_get_raw(NTC_ADC_CHANNEL);
  if (valor_adc <= 0 || valor_adc >= ADC_RESOLUCAO) return -1000;

  float tensao = (valor_adc * V_REF) / ADC_RESOLUCAO;
  float resistencia_ntc = (tensao * RESISTOR_FIXO) / (V_REF - tensao);

  float temp_kelvin = 1.0 / (log(resistencia_ntc / 10000.0) / BETA + 1.0 / T0);
  return (int)(temp_kelvin - 273.15);
}

void calcular_aproximacao_temp(int temp_ntc) {
  int diff = temperatura_alarme - temp_ntc;

  if (diff <= 2)       aprox_temperatura = 'D';
  else if (diff <= 10) aprox_temperatura = '7';
  else if (diff <= 15) aprox_temperatura = '3';
  else if (diff <= 20) aprox_temperatura = '0';
  else                 aprox_temperatura = 'F';
}

// Tarefas FreeRTOS
void task_temperature_monitor(void *pvParameter) {
  temp_data_t temp_data;

  while (1) {
    int temp_atual = ler_temperatura_ntc();

    if (temp_atual != -1000) {
      temp_data.temperatura = temp_atual;
      temp_data.temperatura_alarme = temperatura_alarme;
      temp_data.alarme_ativo = (temp_atual >= temperatura_alarme);

      calcular_aproximacao_temp(temp_atual);

      xQueueSend(led_queue, &temp_data, 0);
      xQueueSend(display_queue, &temp_data, 0);
      xQueueSend(sd_queue, &temp_data, 0);
      xQueueSend(buzzer_queue, &temp_data, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void task_led_control(void *pvParameter) {
  temp_data_t temp_data;

  while (1) {
    if (xQueueReceive(led_queue, &temp_data, portMAX_DELAY)) {
      gpio_set_level(LED_GPIO, 0);
      gpio_set_level(LED_GPIO_2, 0);
      gpio_set_level(LED_GPIO_3, 0);
      gpio_set_level(LED_GPIO_4, 0);

      if (temp_data.alarme_ativo) {
        for (int i = 0; i < 3; i++) {
          gpio_set_level(LED_GPIO, 1);
          gpio_set_level(LED_GPIO_2, 1);
          gpio_set_level(LED_GPIO_3, 1);
          gpio_set_level(LED_GPIO_4, 1);
          vTaskDelay(pdMS_TO_TICKS(200));

          gpio_set_level(LED_GPIO, 0);
          gpio_set_level(LED_GPIO_2, 0);
          gpio_set_level(LED_GPIO_3, 0);
          gpio_set_level(LED_GPIO_4, 0);
          vTaskDelay(pdMS_TO_TICKS(200));
        }
      } else {
        int diff = temp_data.temperatura_alarme - temp_data.temperatura;

        if (diff <= 20) gpio_set_level(LED_GPIO, 1);
        if (diff <= 15) gpio_set_level(LED_GPIO_2, 1);
        if (diff <= 10) gpio_set_level(LED_GPIO_3, 1);
        if (diff <= 2)  gpio_set_level(LED_GPIO_4, 1);
      }
    }
  }
}

void task_display_update(void *pvParameter) {
  temp_data_t temp_data;

  while (1) {
    if (xQueueReceive(display_queue, &temp_data, portMAX_DELAY)) {
      if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        char linha1[17];
        char linha2[17];

        snprintf(linha1, sizeof(linha1), "NTC:%2dC Al:%c",
                 temp_data.temperatura, aprox_temperatura);
        snprintf(linha2, sizeof(linha2), "Alarme: %2dC",
                 temp_data.temperatura_alarme);

        lcd_i2c_cursor_set(&lcd, 0, 0);
        lcd_i2c_print(&lcd, linha1);
        lcd_i2c_cursor_set(&lcd, 0, 1);
        lcd_i2c_print(&lcd, linha2);

        xSemaphoreGive(i2c_mutex);
      }
    }
  }
}

void task_sd_logger(void *pvParameter) {
  temp_data_t temp_data;

  while (1) {
    if (xQueueReceive(sd_queue, &temp_data, portMAX_DELAY)) {
      char caminho_arquivo[64];
      snprintf(caminho_arquivo, sizeof(caminho_arquivo), MOUNT_POINT"/Temp.csv");

      FILE* f = fopen(caminho_arquivo, "a");
      if (f) {
        time_t now;
        time(&now);
        fprintf(f, "%lld,%d,%d,%s\n",
                now,
                temp_data.temperatura,
                temp_data.temperatura_alarme,
                temp_data.alarme_ativo ? "ALARME" : "OK");
        fclose(f);
      } else {
        ESP_LOGE(TAG, "Falha na escrita do arquivo SD");
      }
    }
  }
}

void task_buzzer_control(void *pvParameter) {
  temp_data_t temp_data;

  while (1) {
    if (xQueueReceive(buzzer_queue, &temp_data, portMAX_DELAY)) {
      if (temp_data.alarme_ativo) {
        for (int i = 0; i < 5; i++) {
          ledc_set_freq(BUZZER_MODE, BUZZER_TIMER, 2500);
          ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, BUZZER_DUTY);
          ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
          vTaskDelay(pdMS_TO_TICKS(200));

          ledc_set_freq(BUZZER_MODE, BUZZER_TIMER, 1500);
          vTaskDelay(pdMS_TO_TICKS(200));

          ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 0);
          ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
          vTaskDelay(pdMS_TO_TICKS(100));
        }
      } else {
        ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 0);
        ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
      }
    }
  }
}

void init_hardware(void) {
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
  ESP_ERROR_CHECK(adc1_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11));

  configurar_gpio();

  init_pwm_buzzer();

  i2c_master_init();

  lcd_i2c_init(&lcd);

  init_sdcard();

  esp_timer_create_args_t args1 = {
    .callback = &debounce_callback_botao1,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "debounce1"
  };
  ESP_ERROR_CHECK(esp_timer_create(&args1, &debounce_timer_botao1));

  esp_timer_create_args_t args2 = {
    .callback = &debounce_callback_botao2,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "debounce2"
  };
  ESP_ERROR_CHECK(esp_timer_create(&args2, &debounce_timer_botao2));
}

void app_main() {
  ESP_LOGI(TAG, "Iniciando sistema de monitoramento de temperatura");

  init_hardware();

  temp_queue = xQueueCreate(5, sizeof(temp_data_t));
  led_queue = xQueueCreate(10, sizeof(temp_data_t));
  display_queue = xQueueCreate(5, sizeof(temp_data_t));
  sd_queue = xQueueCreate(20, sizeof(temp_data_t));
  buzzer_queue = xQueueCreate(10, sizeof(temp_data_t));

  i2c_mutex = xSemaphoreCreateMutex();

  if (!temp_queue || !led_queue || !display_queue || !sd_queue || !buzzer_queue || !i2c_mutex) {
    ESP_LOGE(TAG, "Falha ao criar filas ou mutex!");
    return;
  }

  xTaskCreate(task_temperature_monitor, "temp_monitor", 4096, NULL, TASK_TEMP_PRIORITY, &task_temperature_handle);
  xTaskCreate(task_led_control, "led_control", 2048, NULL, TASK_LED_PRIORITY, &task_led_control_handle);
  xTaskCreate(task_display_update, "display_update", 4096, NULL, TASK_DISPLAY_PRIORITY, &task_display_handle);
  xTaskCreate(task_sd_logger, "sd_logger", 4096, NULL, TASK_SD_PRIORITY, &task_sd_handle);
  xTaskCreate(task_buzzer_control, "buzzer_control", 2048, NULL, TASK_BUZZER_PRIORITY, &task_buzzer_handle);

  ESP_LOGI(TAG, "Sistema iniciado com sucesso!");

}
