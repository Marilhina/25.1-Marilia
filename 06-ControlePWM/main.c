#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_rom_sys.h" 
#include "driver/ledc.h"
#include "int_i2c.h"
#include "esp_vfs_dev.h"

#define LED_GPIO    GPIO_NUM_10
#define LED_GPIO_2  GPIO_NUM_12
#define LED_GPIO_3  GPIO_NUM_13
#define LED_GPIO_4  GPIO_NUM_14
#define LED_GPIO_5  GPIO_NUM_3

#define BOTAO_1     GPIO_NUM_4
#define BOTAO_2     GPIO_NUM_5

volatile int contador = 0;
volatile int contagem = 1;
volatile bool atualizar_display_flag = false;

lcd_i2c_handle_t lcd = {
    .address = 0x27,      
    .num = I2C_NUM_0,    
    .backlight = 1,
    .size = DISPLAY_16X02 
};

TickType_t ultima_atualizacao = 0;

static ledc_channel_config_t ledc_channel;

void atualizar_display() {
    static int contador_antigo = -1;

    if (contador == contador_antigo) return; 
    contador_antigo = contador;

    char linha1[17], linha2[17];
    snprintf(linha1, sizeof(linha1), "DEC: %3d       ", contador);
    snprintf(linha2, sizeof(linha2), "HEX: 0x%X      ", contador);

    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, linha1);

    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, linha2);
}



void config_led_pwm(){
  ledc_timer_config_t ledc_timer = {
      .duty_resolution = LEDC_TIMER_10_BIT,
      .freq_hz = 1000,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .clk_cfg = LEDC_AUTO_CLK,
  };

  ledc_timer_config(&ledc_timer);
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.duty = 50;
  ledc_channel.gpio_num = LED_GPIO_5;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.hpoint = 0;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel_config(&ledc_channel);
}

void atualizar_pwm() {
    uint32_t duty = (contador * 1023) / 15;
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}


void atualizar_leds() {
    gpio_set_level(LED_GPIO,    (contador >> 0) & 0x01);
    gpio_set_level(LED_GPIO_2,  (contador >> 1) & 0x01);
    gpio_set_level(LED_GPIO_3,  (contador >> 2) & 0x01);
    gpio_set_level(LED_GPIO_4,  (contador >> 3) & 0x01);
}

void IRAM_ATTR gpio_isr_handler_botao1(void* arg) {
    gpio_intr_disable(BOTAO_1);
    esp_rom_delay_us(5000);

    if (gpio_get_level(BOTAO_1) == 0) { 
        contador = (contador + contagem) ;
        atualizar_leds();
        atualizar_pwm();
        atualizar_display_flag = true;
    }

    gpio_intr_enable(BOTAO_1);
}

void IRAM_ATTR gpio_isr_handler_botao2(void* arg) {
    gpio_intr_disable(BOTAO_2);
    esp_rom_delay_us(5000);

    if (gpio_get_level(BOTAO_2) == 0) { 
        contador = (contador - contagem) ;
        atualizar_pwm();
        atualizar_leds();
        atualizar_display_flag = true;
    }

    gpio_intr_enable(BOTAO_2);
}

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 20,
        .scl_io_num = 19,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void configurar_gpio() {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GPIO_2);
    gpio_set_direction(LED_GPIO_2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GPIO_3);
    gpio_set_direction(LED_GPIO_3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GPIO_4);
    gpio_set_direction(LED_GPIO_4, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GPIO_5);
    gpio_set_direction(LED_GPIO_5, GPIO_MODE_OUTPUT);

    config_led_pwm();

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,      
        .mode = GPIO_MODE_INPUT,           
        .pin_bit_mask = (1ULL << BOTAO_1) | (1ULL << BOTAO_2), 
        .pull_up_en = GPIO_PULLUP_ENABLE,    
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO_1, gpio_isr_handler_botao1, NULL);
    gpio_isr_handler_add(BOTAO_2, gpio_isr_handler_botao2, NULL);
}

void app_main() {
    i2c_master_init();
    lcd_i2c_init(&lcd);

    configurar_gpio();
    atualizar_leds();
    atualizar_pwm();
    atualizar_display();

    while (true) {
        if (atualizar_display_flag &&
            (xTaskGetTickCount() - ultima_atualizacao) > pdMS_TO_TICKS(100)) {

            atualizar_display_flag = false;
            atualizar_display();
            ultima_atualizacao = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
