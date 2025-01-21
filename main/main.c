/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_dsp.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_timer.h"

const char *TAG = "[MAIN]";

/* I2S defs */
#define EXAMPLE_STD_BCLK_IO1 GPIO_NUM_0 // I2S bit clock io number
#define EXAMPLE_STD_WS_IO1 GPIO_NUM_1   // I2S word select io number
#define EXAMPLE_STD_DOUT_IO1 GPIO_NUM_2 // I2S data out io number
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE 8192
#define I2S_BUFFER_SIZE 256
#define FFT_SIZE CONFIG_DSP_MAX_FFT_SIZE
#define MAIN_BUTTON GPIO_NUM_21
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_NUM_21)
#define I2S_BUFFER_32_TOTAL_SIZE I2S_SAMPLE_RATE
#define I2S_BUFFER_16_TOTAL_SIZE I2S_SAMPLE_RATE >> 2
#define I2S_TWO_PERIOD_BUFFER_SIZE I2S_SAMPLE_RATE << 1

int16_t i2s_buffer[I2S_TWO_PERIOD_BUFFER_SIZE] = {0};
uint8_t buffer32[I2S_BUFFER_32_TOTAL_SIZE] = {0};
int16_t buffer16[I2S_BUFFER_16_TOTAL_SIZE] = {0};

// Input test array
__attribute__((aligned(16))) float x1[FFT_SIZE];
// Window coefficients
__attribute__((aligned(16))) float wind[FFT_SIZE];

void i2s_install()
{
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_NMI,
        .dma_buf_count = 16,
        .dma_buf_len = I2S_BUFFER_SIZE * 4,
        .use_apll = true,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0};

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

static void mic_read_task(void *args)
{
    int pos = 0;
    uint32_t sample_count = 0;
    int64_t start_time = 0; // Variable to hold the start time
    int64_t end_time = 0;   // Variable to hold the end time
    int64_t elapsed_time = 0;
    esp_err_t ret;

    ret = dsps_fft2r_init_fc32(NULL, FFT_SIZE >> 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
        return;
    }
    // Generate Hann window
    dsps_wind_hann_f32(wind, FFT_SIZE);

    while (1)
    {
        uint8_t j = 0;
        while (gpio_get_level(MAIN_BUTTON))
        {
            if (sample_count == 0)
            {
                i2s_start(I2S_PORT);
                start_time = esp_timer_get_time(); // Get the start time in microseconds
            }
            size_t bytes_read = 0;
            esp_err_t err = i2s_read(I2S_PORT, &buffer32, sizeof(buffer32), &bytes_read, pdMS_TO_TICKS(1001));
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to read I2S data. Error = %i", err);
                break;
            }
            // Read I2S data buffer
            int16_t samples_read = bytes_read / 4;
            // for (size_t i = 0; i < samples_read / 2; ++i)
            for (size_t i = 0; i < samples_read; ++i)
            {
                uint8_t mid = buffer32[i * 4 + 2];
                uint8_t msb = buffer32[i * 4 + 3];
                // uint8_t mid = buffer32[2 * i * 4 + 2];
                // uint8_t msb = buffer32[2 * i * 4 + 3];
                uint16_t raw = (((uint32_t)msb) << 8) + ((uint32_t)mid);
                // memcpy(&buffer16[i], &raw, sizeof(raw));
                memcpy(&i2s_buffer[i + (j * (I2S_BUFFER_16_TOTAL_SIZE))], &raw, sizeof(raw));
                // memcpy(&i2s_buffer[i + (j * (I2S_BUFFER_16_TOTAL_SIZE >> 2))], &raw, sizeof(raw));
                // i2s_buffer[i + (j * I2S_BUFFER_16_TOTAL_SIZE)] = (buffer16[i]);
                // x1[i + (j * I2S_BUFFER_16_TOTAL_SIZE)] = (float)(buffer16[i] /* * wind[i + j * I2S_BUFFER_SIZE] */);
            }

            sample_count += samples_read;
            j++;
            printf("j = %d\r\n", j);
            if (j >= 8)
            {
                i2s_stop(I2S_PORT);
                end_time = esp_timer_get_time();      // Get the end time in microseconds
                elapsed_time = end_time - start_time; // Calculate elapsed time in microseconds
                break;
            }

            // if (j >= 2)
            // {
            //     j = 0;
            //     for (int i = 0; i < FFT_SIZE; i++)
            //     {
            //         printf("%f\r\n", x1[i]);
            //     }
            //     ESP_LOGI(TAG, "Writing file");
            //     ret = sd_write_file(file_data, data);
            //     if (ret != ESP_OK)
            //     {
            //         ESP_LOGE(TAG, "Failed to write file. Error = %i", ret);
            //         return;
            //     }
            //     // FFT Radix-2
            //     unsigned int start_r2 = dsp_get_cpu_cycle_count();
            //     dsps_fft2r_fc32(x1, FFT_SIZE >> 1);
            //     // Bit reverse
            //     dsps_bit_rev2r_fc32(x1, FFT_SIZE >> 1);
            //     // Convert one complex vector with length N/2 to one real spectrum vector with length N/2
            //     dsps_cplx2real_fc32(x1, FFT_SIZE >> 1);
            //     unsigned int end_r2 = dsp_get_cpu_cycle_count();

            //     // for (int i = 0; i < FFT_SIZE / 2; i++)
            //     // {
            //     //     // printf("%f\r\n", x1[i]);
            //     // }
            //     // Show power spectrum in 64x10 window from -100 to 0 dB from 0..N/4 samples
            //     /*                 ESP_LOGW(TAG, "Signal x1");
            //                     dsps_view(x1, FFT_SIZE / 2, 64, 20, 0, 500, '|');
            //                     ESP_LOGI(TAG, "FFT Radix 2 for %i complex points take %i cycles", FFT_SIZE / 2, end_r2 - start_r2); */
            // }
        }
        if (sample_count > 0)
        {
            ESP_LOGI(TAG, "Read %lu samples. Total elapsed time: %lld ms", sample_count, elapsed_time / 1000);
            if (elapsed_time / 1000 >= 2000)
            {
                for (size_t i = 0; i < sample_count; i++)
                {
                    printf("%d\r\n", i2s_buffer[i]);
                }
            }
            sample_count = 0; // Reset sample count

            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

void gpio_config_pin(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);
}

void app_main(void)
{
    i2s_install();
    i2s_setpin();
    gpio_config_pin();

    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreate(mic_read_task, "mic_read_task", 4096, NULL, 5, NULL);
}