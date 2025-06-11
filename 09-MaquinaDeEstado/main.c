#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

#define LED_GPIO     GPIO_NUM_17
#define LED_GPIO_2   GPIO_NUM_18
#define LED_GPIO_3   GPIO_NUM_8
#define LED_GPIO_4   GPIO_NUM_3

#define BOTAO_1      GPIO_NUM_4
#define BOTAO_2      GPIO_NUM_5

#define BUZZER_GPIO GPIO_NUM_47

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

int temperatura_alarme = ALARME_INICIAL;
bool piscar_leds = false;
volatile bool atualizar_display_flag = false;
volatile bool botao1_pendente = false;
volatile bool botao2_pendente = false;

esp_timer_handle_t debounce_timer_botao1;
esp_timer_handle_t debounce_timer_botao2;
esp_timer_handle_t piscar_timer;

TickType_t ultima_atualizacao = 0;
TickType_t ultimo_microsd = 0;
int temp_ntc_global = 0;
bool leds_acesos = false;

enum MaquinaDeEstado {
  LER_TEMPERATURA,
  MUDA_ESTADO_LED,
  ATUALIZA_LCD,
  ATUALIZA_MICROSD,
  TEMPERATURA_ACIMA
};

int estado = LER_TEMPERATURA;

lcd_i2c_handle_t lcd = {
  .address = 0x27,
  .num = I2C_NUM_0,
  .backlight = 1,
  .size = DISPLAY_16X02
};

void buzzer_pwm_init() {
  ledc_timer_config_t timer_conf = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .timer_num        = LEDC_TIMER_0,
    .duty_resolution  = LEDC_TIMER_10_BIT,
    .freq_hz          = 2000, 
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
    .gpio_num       = BUZZER_GPIO,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_0,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&channel_conf);
}

void buzzer_on() {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512); 
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void buzzer_off() {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void piscar_callback(void* arg) {
  if (piscar_leds) {
    if (leds_acesos) {
      gpio_set_level(LED_GPIO, 0);
      gpio_set_level(LED_GPIO_2, 0);
      gpio_set_level(LED_GPIO_3, 0);
      gpio_set_level(LED_GPIO_4, 0);
      leds_acesos = false;
    } else {
      gpio_set_level(LED_GPIO, 1);
      gpio_set_level(LED_GPIO_2, 1);
      gpio_set_level(LED_GPIO_3, 1);
      gpio_set_level(LED_GPIO_4, 1);
      leds_acesos = true;
    }

    esp_timer_start_once(piscar_timer, 300000); 
  }
}

void init_sdcard() {
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };

  sdmmc_card_t* card;
  const char mount_point[] = MOUNT_POINT;

  ESP_LOGI("SD", "Using SPI peripheral");
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
    ESP_LOGE("SD", "Falha! (%s)", esp_err_to_name(ret));
    return;
  }

  sdmmc_card_print_info(stdout, card);
}

void debounce_callback_botao1(void* arg) {
  if (gpio_get_level(BOTAO_1) == 0) {
    temperatura_alarme += 5;
    if (temperatura_alarme > 100) temperatura_alarme = 100;
    atualizar_display_flag = true;
  }
  botao1_pendente = false;
}

void debounce_callback_botao2(void* arg) {
  if (gpio_get_level(BOTAO_2) == 0) {
    temperatura_alarme -= 5;
    if (temperatura_alarme < 0) temperatura_alarme = 0;
    atualizar_display_flag = true;
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
  i2c_param_config(I2C_NUM_0, &conf);
  i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void configurar_gpio() {
  gpio_config_t out_conf = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ULL << LED_GPIO) | (1ULL << LED_GPIO_2) |
    (1ULL << LED_GPIO_3) | (1ULL << LED_GPIO_4),
  };
  gpio_config(&out_conf);

  gpio_config_t in_conf = {
    .intr_type = GPIO_INTR_NEGEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL << BOTAO_1) | (1ULL << BOTAO_2),
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  gpio_config(&in_conf);

  gpio_install_isr_service(0);
  gpio_isr_handler_add(BOTAO_1, gpio_isr_handler_botao1, NULL);
  gpio_isr_handler_add(BOTAO_2, gpio_isr_handler_botao2, NULL);
}

int ler_temperatura_ntc() {
  int valor_adc = adc1_get_raw(NTC_ADC_CHANNEL);
  if (valor_adc <= 0 || valor_adc >= ADC_RESOLUCAO) return -1000;

  float tensao = (valor_adc * V_REF) / ADC_RESOLUCAO;
  float resistencia_ntc = (tensao * RESISTOR_FIXO) / (V_REF - tensao);

  float temp_kelvin = 1.0 / (log(resistencia_ntc / 10000.0) / BETA + 1.0 / T0);
  return (int)(temp_kelvin - 273.15);
}

void verificar_e_atualizar_leds(int temp_ntc) {
  int diff = temperatura_alarme - temp_ntc;

  piscar_leds = false;
  esp_timer_stop(piscar_timer);

  gpio_set_level(LED_GPIO, 0);
  gpio_set_level(LED_GPIO_2, 0);
  gpio_set_level(LED_GPIO_3, 0);
  gpio_set_level(LED_GPIO_4, 0);

  buzzer_off();

  if (temp_ntc >= temperatura_alarme) {
    piscar_leds = true;
    leds_acesos = false;
    esp_timer_start_once(piscar_timer, 100000);
    buzzer_on();
    return;
  }

  if (diff <= 20) gpio_set_level(LED_GPIO, 1);
  if (diff <= 15) gpio_set_level(LED_GPIO_2, 1);
  if (diff <= 10) gpio_set_level(LED_GPIO_3, 1);
  if (diff <= 2)  gpio_set_level(LED_GPIO_4, 1);
}

void atualizar_display(int temp_ntc) {
  char linha1[17];
  snprintf(linha1, sizeof(linha1), "NTC:%2dC Al:%2dC", temp_ntc, temperatura_alarme);

  lcd_i2c_cursor_set(&lcd, 0, 0);
  lcd_i2c_print(&lcd, linha1);
  lcd_i2c_cursor_set(&lcd, 0, 1);
  lcd_i2c_print(&lcd, "Sistema Ativo   ");
}

void atualizar_microsd(int temp_ntc) {
  char caminho_arquivo[64];
  snprintf(caminho_arquivo, sizeof(caminho_arquivo), MOUNT_POINT"/Temp.csv");

  FILE* f = fopen(caminho_arquivo, "a");
  if (f) {
    time_t now;
    time(&now);
    fprintf(f, "%lld, %d\n", now, temp_ntc);
    fclose(f);
  } else {
    ESP_LOGE("SD", "Falha na escrita do arquivo");
  }
}

void app_main() {
  i2c_master_init();
  lcd_i2c_init(&lcd);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11);

  configurar_gpio();
  buzzer_pwm_init();

  esp_timer_create_args_t args1 = {
    .callback = &debounce_callback_botao1,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "debounce1"
  };
  esp_timer_create(&args1, &debounce_timer_botao1);

  esp_timer_create_args_t args2 = {
    .callback = &debounce_callback_botao2,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "debounce2"
  };
  esp_timer_create(&args2, &debounce_timer_botao2);

  esp_timer_create_args_t args3 = {
    .callback = &piscar_callback,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "piscar"
  };
  esp_timer_create(&args3, &piscar_timer);

  init_sdcard();

  lcd_i2c_cursor_set(&lcd, 0, 0);
  lcd_i2c_print(&lcd, "Monitor Temp    ");
  lcd_i2c_cursor_set(&lcd, 0, 1);
  lcd_i2c_print(&lcd, "Inicializando...");
  vTaskDelay(pdMS_TO_TICKS(2000));

  while (true) {
    switch (estado) {
      case LER_TEMPERATURA:
        temp_ntc_global = ler_temperatura_ntc();
        if (temp_ntc_global == -1000) {
          ESP_LOGE("NTC", "Erro na leitura do sensor");
          temp_ntc_global = 0;
        }
        estado = MUDA_ESTADO_LED;
        break;

      case MUDA_ESTADO_LED:
        verificar_e_atualizar_leds(temp_ntc_global);
        estado = ATUALIZA_LCD;
        break;

      case ATUALIZA_LCD:
        if (atualizar_display_flag || (xTaskGetTickCount() - ultima_atualizacao) > pdMS_TO_TICKS(1000)) {
          atualizar_display(temp_ntc_global);
          ultima_atualizacao = xTaskGetTickCount();
          atualizar_display_flag = false;
        }
        estado = ATUALIZA_MICROSD;
        break;

      case ATUALIZA_MICROSD:
        if ((xTaskGetTickCount() - ultimo_microsd) > pdMS_TO_TICKS(10000)) {
          atualizar_microsd(temp_ntc_global);
          ultimo_microsd = xTaskGetTickCount();
        }
        estado = LER_TEMPERATURA;
        break;

      case TEMPERATURA_ACIMA:
        estado = LER_TEMPERATURA;
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
