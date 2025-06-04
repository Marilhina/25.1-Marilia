#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "int_i2c.h"

#define LED_GPIO     GPIO_NUM_10
#define LED_GPIO_2   GPIO_NUM_12
#define LED_GPIO_3   GPIO_NUM_13
#define LED_GPIO_4   GPIO_NUM_14
#define LED_GPIO_5   GPIO_NUM_3

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

int temperatura_alarme = ALARME_INICIAL;
bool piscar_leds = false;
volatile bool atualizar_display_flag = false;
volatile bool botao1_pendente = false;
volatile bool botao2_pendente = false;

esp_timer_handle_t debounce_timer_botao1;
esp_timer_handle_t debounce_timer_botao2;

TickType_t ultima_atualizacao = 0;

lcd_i2c_handle_t lcd = {
    .address = 0x27,
    .num = I2C_NUM_0,
    .backlight = 1,
    .size = DISPLAY_16X02
};

static ledc_channel_config_t ledc_channel;

void config_led_pwm() {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel.channel    = LEDC_CHANNEL_0;
    ledc_channel.duty       = 50;
    ledc_channel.gpio_num   = LED_GPIO_5;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.hpoint     = 0;
    ledc_channel.timer_sel  = LEDC_TIMER_0;

    ledc_channel_config(&ledc_channel);
}

void debounce_callback_botao1(void* arg) {
    if (gpio_get_level(BOTAO_1) == 0) {
        temperatura_alarme += 5;
        atualizar_display_flag = true;
    }
    botao1_pendente = false;
}

void debounce_callback_botao2(void* arg) {
    if (gpio_get_level(BOTAO_2) == 0) {
        temperatura_alarme -= 5;
        atualizar_display_flag = true;
    }
    botao2_pendente = false;
}

void IRAM_ATTR gpio_isr_handler_botao1(void* arg) {
    if (!botao1_pendente) {
        botao1_pendente = true;
        esp_timer_start_once(debounce_timer_botao1, 5000);
    }
}

void IRAM_ATTR gpio_isr_handler_botao2(void* arg) {
    if (!botao2_pendente) {
        botao2_pendente = true;
        esp_timer_start_once(debounce_timer_botao2, 5000);
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
                (1ULL << LED_GPIO_3) | (1ULL << LED_GPIO_4) |
                (1ULL << LED_GPIO_5) | (1ULL << BUZZER_GPIO),

    };
    gpio_config(&out_conf);

    config_led_pwm();

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

    gpio_set_level(LED_GPIO, 0);
    gpio_set_level(LED_GPIO_2, 0);
    gpio_set_level(LED_GPIO_3, 0);
    gpio_set_level(LED_GPIO_4, 0);
    gpio_set_level(BUZZER_GPIO, 0); 

    piscar_leds = false;

    if (temp_ntc >= temperatura_alarme) {
        piscar_leds = true;
        gpio_set_level(BUZZER_GPIO, 1); 
        return;
    }

    if (diff <= 20) gpio_set_level(LED_GPIO, 1);
    if (diff <= 15) gpio_set_level(LED_GPIO_2, 1);
    if (diff <= 10) gpio_set_level(LED_GPIO_3, 1);
    if (diff <= 2)  gpio_set_level(LED_GPIO_4, 1);

    gpio_set_level(BUZZER_GPIO, 0);
}


void atualizar_display(int temp_ntc) {
    char linha1[17];
    snprintf(linha1, sizeof(linha1), "NTC:%2dC Al:%2dC", temp_ntc, temperatura_alarme);

    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, linha1);
    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, "                ");  
}


void app_main() {
    i2c_master_init();
    lcd_i2c_init(&lcd);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11);

    configurar_gpio();

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

    while (true) {
        int temp_ntc = ler_temperatura_ntc();
        verificar_e_atualizar_leds(temp_ntc);

        if (piscar_leds) {
            gpio_set_level(LED_GPIO,    1);
            gpio_set_level(LED_GPIO_2,  1);
            gpio_set_level(LED_GPIO_3,  1);
            gpio_set_level(LED_GPIO_4,  1);
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_set_level(LED_GPIO,    0);
            gpio_set_level(LED_GPIO_2,  0);
            gpio_set_level(LED_GPIO_3,  0);
            gpio_set_level(LED_GPIO_4,  0);
        }

        if (atualizar_display_flag || (xTaskGetTickCount() - ultima_atualizacao) > pdMS_TO_TICKS(500)) {
            atualizar_display(temp_ntc);
            ultima_atualizacao = xTaskGetTickCount();
            atualizar_display_flag = false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

