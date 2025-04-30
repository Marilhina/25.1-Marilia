#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_GPIO GPIO_NUM_18 
#define LED_GPIO_5 GPIO_NUM_5

void app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GPIO_5);
    gpio_set_direction(LED_GPIO_5, GPIO_MODE_OUTPUT);


    while (1) {
        gpio_set_level(LED_GPIO, 1); 
        
        for (int i=0; i<5;i++){
          gpio_set_level(LED_GPIO_5, 1); 
          vTaskDelay(200 / portTICK_PERIOD_MS);

          gpio_set_level(LED_GPIO_5, 0); 
          vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        gpio_set_level(LED_GPIO, 0); 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
