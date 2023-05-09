/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "input_iot.h"
#include "uart_iot.h"

#define BTN 0

void uart_event_handler(uint8_t *data, uint16_t size)
{
    printf("%s\n", data);
}


void gpio_event_handler(int pin)
{
    if(pin == GPIO_NUM_0 && input_io_get_level(BTN) == 0)
    {
        uart_push( (uint8_t*)"hello\n", 7);
    }
}


void app_main(void)
{
    input_io_create(BTN, HI_TO_LO);
    input_set_callback(gpio_event_handler);

    uart_init();
    uart_set_callback(uart_event_handler);
}

