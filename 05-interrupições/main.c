#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_rom_sys.h" 

#define LED_GPIO    GPIO_NUM_10
#define LED_GPIO_2  GPIO_NUM_12
#define LED_GPIO_3  GPIO_NUM_13
#define LED_GPIO_4  GPIO_NUM_14

#define BOTAO_1     GPIO_NUM_4
#define BOTAO_2     GPIO_NUM_5

volatile int contador = 0;
volatile int contagem = 1;

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
        contador = (contador + contagem) & 0x0F;
        atualizar_leds();
    }
    gpio_intr_enable(BOTAO_1);
}

void IRAM_ATTR gpio_isr_handler_botao2(void* arg) {

    gpio_intr_disable(BOTAO_2);
    esp_rom_delay_us(5000);
    if (gpio_get_level(BOTAO_2) == 0) { 
        contagem = (contagem == 1) ? 2 : 1;
    }
    gpio_intr_enable(BOTAO_2);
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
    atualizar_leds();
    configurar_gpio();

    while (true) {
    }
}
