/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "sdkconfig.h"

#define EXAMPLE_STD_BCLK_IO1 GPIO_NUM_0 // I2S bit clock io number
#define EXAMPLE_STD_WS_IO1 GPIO_NUM_1   // I2S word select io number
#define EXAMPLE_STD_DOUT_IO1 GPIO_NUM_2 // I2S data out io number
#define I2S_PORT I2S_NUM_0

#define EXAMPLE_BUFF_SIZE 64
int16_t sBuffer[EXAMPLE_BUFF_SIZE];

void i2s_install()
{
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 44100,
        .bits_per_sample = (i2s_bits_per_sample_t)(16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = EXAMPLE_BUFF_SIZE,
        .use_apll = false};

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin()
{
    const i2s_pin_config_t pin_config = {
        .bck_io_num = EXAMPLE_STD_BCLK_IO1,
        .ws_io_num = EXAMPLE_STD_WS_IO1,
        .data_out_num = -1,
        .data_in_num = EXAMPLE_STD_DOUT_IO1};

    i2s_set_pin(I2S_PORT, &pin_config);
}

static void i2s_example_read_task(void *args)
{
    while (1)
    {
        int rangelimit = 3000;
        printf("%d", rangelimit * -1);
        printf(" ");
        printf("%d", rangelimit);
        printf(" ");
        size_t bytesIn = 0;
        esp_err_t result = i2s_read(I2S_PORT, &sBuffer, EXAMPLE_BUFF_SIZE, &bytesIn, portMAX_DELAY);

        if (result == ESP_OK)
        {
            // Read I2S data buffer
            int16_t samples_read = bytesIn / 8;
            if (samples_read > 0)
            {
                float mean = 0;
                for (int16_t i = 0; i < samples_read; ++i)
                {
                    mean += (sBuffer[i]);
                }

                // Average the data reading
                mean /= samples_read;

                // Print to serial plotter
                printf("%.2f\r\n", mean);
            }
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    i2s_install();
    i2s_setpin();
    i2s_start(I2S_PORT);
    /* Step 3: Create writing and reading task, enable and start the channels */
    xTaskCreate(i2s_example_read_task, "i2s_example_read_task", 4096, NULL, 5, NULL);
}