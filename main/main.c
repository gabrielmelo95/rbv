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

/* SD card defs */
#define EXAMPLE_MAX_CHAR_SIZE 64
#define ONE_DATA_SIZE 8
#define MOUNT_POINT "/sdcard"
#define PIN_NUM_MISO GPIO_NUM_17
#define PIN_NUM_MOSI GPIO_NUM_16
#define PIN_NUM_CLK GPIO_NUM_23
#define PIN_NUM_CS GPIO_NUM_22

const char *file_data = MOUNT_POINT "/data_1.txt";
char data[ONE_DATA_SIZE * I2S_BUFFER_16_TOTAL_SIZE];
char one_data[ONE_DATA_SIZE];

uint8_t buffer32[I2S_BUFFER_32_TOTAL_SIZE] = {0};
int16_t buffer16[I2S_BUFFER_16_TOTAL_SIZE] = {0};

// Input test array
__attribute__((aligned(16))) float x1[FFT_SIZE];
// Window coefficients
__attribute__((aligned(16))) float wind[FFT_SIZE];

static esp_err_t sd_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t sd_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }

    char line[EXAMPLE_MAX_CHAR_SIZE];
    while (fgets(line, sizeof(line), f) != NULL) // Continue reading until EOF
    {
        // Strip newline
        char *pos = strchr(line, '\n');
        if (pos)
        {
            *pos = '\0';
        }
        printf("%s\n", line);
    }

    fclose(f);
    return ESP_OK;
}

esp_err_t sd_card_init(void)
{
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 1000;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return ret;
    }
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return ret;
    }
    ESP_LOGI(TAG, "Filesystem mounted");
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    return ret;
}

void i2s_install()
{
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_SHARED,
        .dma_buf_count = 8,
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
            size_t bytes_read = 0;
            esp_err_t err = i2s_read(I2S_PORT, &buffer32, sizeof(buffer32), &bytes_read, portMAX_DELAY);
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
                memcpy(&buffer16[i], &raw, sizeof(raw));
                sprintf(one_data, "%d\n", buffer16[i]);
                strcat(data, one_data);

                x1[i + (j * I2S_BUFFER_16_TOTAL_SIZE)] = (float)(buffer16[i] /* * wind[i + j * I2S_BUFFER_SIZE] */);
            }
            ret = sd_write_file(file_data, data);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to write file. Error = %i", ret);
                return;
            }
            sample_count += samples_read;
            // j++;
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
            ESP_LOGI(TAG, "read %lu samples", sample_count);
            sample_count = 0;
            ret = sd_read_file(file_data);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to read file. Error = %i", ret);
                return;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
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
    i2s_start(I2S_PORT);
    gpio_config_pin();
    sd_card_init();
    /* Step 3: Create writing and reading task, enable and start the channels */
    xTaskCreate(mic_read_task, "mic_read_task", 4096, NULL, 5, NULL);
}