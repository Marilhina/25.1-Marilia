#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"


#define LED_GPIO GPIO_NUM_10
#define LED_GPIO_2 GPIO_NUM_12
#define LED_GPIO_3 GPIO_NUM_13
#define LED_GPIO_4 GPIO_NUM_14

#define DEBOUNCE 50000

#define Botao_1 GPIO_NUM_4
#define Botao_2 GPIO_NUM_5

void app_main() {

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_GPIO_2);
    gpio_set_direction(LED_GPIO_2, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_GPIO_3);
    gpio_set_direction(LED_GPIO_3, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_GPIO_4);
    gpio_set_direction(LED_GPIO_4, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(Botao_1);
    gpio_set_direction(Botao_1, GPIO_MODE_INPUT);

    gpio_reset_pin(Botao_2);
    gpio_set_direction(Botao_2, GPIO_MODE_INPUT);
   

    int contador = 0;
    int contagem = 1;
    int64_t esperaBotao1 = esp_timer_get_time();
    int64_t esperaBotao2 = esp_timer_get_time();

    while (true) {
      int64_t tempoAtual = esp_timer_get_time();

      if(gpio_get_level(Botao_2) == 0 && (tempoAtual-esperaBotao2 > DEBOUNCE)){
        esperaBotao2 = tempoAtual;
        if(contagem == 1)
          contagem = 2;
        else contagem = 1;
      }
      
      if (gpio_get_level(Botao_1) == 0 && (tempoAtual-esperaBotao1 > DEBOUNCE)) { 
        esperaBotao1 = tempoAtual;
        contador = (contador + contagem) & 0x0F; 

        gpio_set_level(LED_GPIO,   (contador >> 0) & 0x01);
        gpio_set_level(LED_GPIO_2, (contador >> 1) & 0x01);
        gpio_set_level(LED_GPIO_3, (contador >> 2) & 0x01);
        gpio_set_level(LED_GPIO_4, (contador >> 3) & 0x01);

      }

    }
}
