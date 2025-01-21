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
#define FFT_SIZE (CONFIG_DSP_MAX_FFT_SIZE >> 1)
#define MAIN_BUTTON GPIO_NUM_21
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_NUM_21)
#define I2S_BUFFER_32_TOTAL_SIZE I2S_SAMPLE_RATE
#define I2S_BUFFER_16_TOTAL_SIZE (I2S_SAMPLE_RATE >> 2)
#define I2S_TWO_PERIOD_BUFFER_SIZE (I2S_SAMPLE_RATE << 1)

int16_t i2s_buffer[I2S_TWO_PERIOD_BUFFER_SIZE] = {0};
uint8_t buffer32[I2S_BUFFER_32_TOTAL_SIZE] = {0};

// Input test array
__attribute__((aligned(16))) float x1[FFT_SIZE];
// Window coefficients
__attribute__((aligned(16))) float wind[FFT_SIZE];
// working complex array
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
// Pointers to result arrays
float *y1_cf = &y_cf[0];

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

void get_fft_peaks(float *y1_cf, float *fft_max_vals, int *fft_max_freq)
{
    // Initialize the top three peaks and their frequencies
    fft_max_vals[0] = fft_max_vals[1] = fft_max_vals[2] = 0;
    fft_max_freq[0] = fft_max_freq[1] = fft_max_freq[2] = 0;

    // Process the FFT array
    for (size_t i = 0; i < FFT_SIZE / 2; i++)
    {
        // Calculate the logarithmic value for the FFT
        y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] +
                                y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) /
                               FFT_SIZE);

        // Skip the first 64 and the last element
        if (i > 64 && i < ((FFT_SIZE / 2) - 1))
        {
            // Check if it's a local maximum
            if (y1_cf[i] > y1_cf[i - 1] && y1_cf[i] > y1_cf[i + 1])
            {
                // Compare with current top peaks
                if (y1_cf[i] > fft_max_vals[0])
                {
                    // Shift down the peaks
                    fft_max_vals[2] = fft_max_vals[1];
                    fft_max_freq[2] = fft_max_freq[1];
                    fft_max_vals[1] = fft_max_vals[0];
                    fft_max_freq[1] = fft_max_freq[0];
                    fft_max_vals[0] = y1_cf[i];
                    fft_max_freq[0] = i;
                }
                else if (y1_cf[i] > fft_max_vals[1])
                {
                    fft_max_vals[2] = fft_max_vals[1];
                    fft_max_freq[2] = fft_max_freq[1];
                    fft_max_vals[1] = y1_cf[i];
                    fft_max_freq[1] = i;
                }
                else if (y1_cf[i] > fft_max_vals[2])
                {
                    fft_max_vals[2] = y1_cf[i];
                    fft_max_freq[2] = i;
                }
            }
        }
    }
}

void fft_calc(float *y_cf, float *fft_max_vals, int *fft_max_freq)
{
    // Record start time for performance measurement
    unsigned int start_r2 = dsp_get_cpu_cycle_count();
    // Perform FFT radix-2 transformation
    dsps_fft2r_fc32(y_cf, FFT_SIZE);
    // Perform bit reversal for the FFT
    dsps_bit_rev2r_fc32(y_cf, FFT_SIZE);
    // Convert complex vector to real spectrum
    dsps_cplx2real_fc32(y_cf, FFT_SIZE);
    // Record end time for performance measurement
    unsigned int end_r2 = dsp_get_cpu_cycle_count();
    printf("FFT calculation time: %u cycles\n", end_r2 - start_r2);
    // Get the top 3 FFT peaks
    get_fft_peaks(y_cf, fft_max_vals, fft_max_freq);
}

static void mic_read_task(void *args)
{
    int pos = 0;
    uint32_t sample_count = 0;
    int64_t start_time = 0; // Variable to hold the start time
    int64_t end_time = 0;   // Variable to hold the end time
    int64_t elapsed_time = 0;
    uint32_t fft_max_vals[3] = {0};
    uint32_t fft_max_freq[3] = {0};
    esp_err_t ret;

    ret = dsps_fft2r_init_fc32(NULL, 2 * FFT_SIZE);
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
            for (size_t i = 0; i < samples_read; ++i)
            {
                uint8_t mid = buffer32[i * 4 + 2];
                uint8_t msb = buffer32[i * 4 + 3];
                uint16_t raw = (((uint32_t)msb) << 8) + ((uint32_t)mid);
                memcpy(&i2s_buffer[i + (j * (I2S_BUFFER_16_TOTAL_SIZE))], &raw, sizeof(raw));
            }

            sample_count += samples_read;
            j++;
            printf("j = %d\r\n", j);
            if (j >= 8)
            {
                break;
            }
        }
        if (sample_count > 0)
        {
            end_time = esp_timer_get_time();      // Get the end time in microseconds
            elapsed_time = end_time - start_time; // Calculate elapsed time in microseconds
            ESP_LOGI(TAG, "Read %lu samples. Total elapsed time: %lld ms", sample_count, elapsed_time / 1000);
            if (elapsed_time / 1000 >= 2000)
            {
                for (size_t j = 0; j < sample_count / FFT_SIZE; j++)
                {
                    for (size_t i = 0; i < FFT_SIZE; i++)
                    {
                        x1[i] = (float)(i2s_buffer[i + (j * FFT_SIZE)] * wind[i]);
                        y1_cf[2 * i] = x1[i];
                        y1_cf[2 * i + 1] = 0;
                    }
                    if (j > 0)
                    {
                        fft_calc(y_cf, fft_max_vals, fft_max_freq);
                        printf("Max 1: %lu at freq %lu\n", fft_max_vals[0], (fft_max_freq[0] * I2S_SAMPLE_RATE) / FFT_SIZE);
                        printf("Max 2: %lu at freq %lu\n", fft_max_vals[1], (fft_max_freq[1] * I2S_SAMPLE_RATE) / FFT_SIZE);
                        printf("Max 3: %lu at freq %lu\n", fft_max_vals[2], (fft_max_freq[2] * I2S_SAMPLE_RATE) / FFT_SIZE);
                    }
                }
            }
            i2s_stop(I2S_PORT);
            sample_count = 0; // Reset sample count
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