/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/lock.h>
#include <sys/param.h>

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
#include "esp_lcd_gc9a01.h"
#include "lvgl.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"

const char *TAG = "[MAIN]";

/* I2S defs */
#define STD_BCLK_IO1 GPIO_NUM_0 // I2S bit clock io number
#define STD_WS_IO1 GPIO_NUM_1   // I2S word select io number
#define STD_DOUT_IO1 GPIO_NUM_2 // I2S data out io number
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE 8192
#define I2S_BUFFER_SIZE 256
#define FFT_SIZE (CONFIG_DSP_MAX_FFT_SIZE >> 1)
#define MAIN_BUTTON GPIO_NUM_21
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_NUM_21)
#define I2S_BUFFER_32_TOTAL_SIZE I2S_SAMPLE_RATE
#define I2S_BUFFER_16_TOTAL_SIZE (I2S_SAMPLE_RATE >> 2)
#define I2S_TWO_PERIOD_BUFFER_SIZE (I2S_SAMPLE_RATE << 1)

/* SPI config*/
#define LCD_HOST SPI2_HOST
#define PIN_NUM_SCLK GPIO_NUM_17
#define PIN_NUM_MOSI GPIO_NUM_16
#define PIN_NUM_MISO -1
#define PIN_NUM_LCD_DC GPIO_NUM_23
#define PIN_NUM_LCD_RST GPIO_NUM_22
#define PIN_NUM_LCD_CS GPIO_NUM_19
#define PIN_NUM_BK_LIGHT -1
#define PIN_NUM_TOUCH_CS -1

/* LCD defs */
#define LCD_PIXEL_CLOCK_HZ (1 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL 1
#define LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define LCD_H_RES 240
#define LCD_V_RES 240
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

/* LVGL defs */
#define LVGL_DRAW_BUF_LINES 20 // number of display lines in each draw buffer
#define LVGL_TICK_PERIOD_MS 10
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE (4 * 1024)
#define LVGL_TASK_PRIORITY 2

// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;
lv_display_t *display = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;
esp_lcd_panel_handle_t panel_handle = NULL;
static lv_obj_t *btn_register;
static lv_obj_t *btn_indentify;
static lv_display_rotation_t rotation = LV_DISP_ROTATION_0;

int16_t i2s_buffer[I2S_TWO_PERIOD_BUFFER_SIZE] = {0};
uint8_t buffer32[I2S_BUFFER_32_TOTAL_SIZE] = {0};
uint8_t click_button_pos = 0;

static QueueHandle_t gpio_evt_queue = NULL;
// Input test array
__attribute__((aligned(16))) float x1[FFT_SIZE];
// Window coefficients
__attribute__((aligned(16))) float wind[FFT_SIZE];
// working complex array
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
// Pointers to result arrays
float *y1_cf = &y_cf[0];

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void lvgl_port_update_callback(lv_display_t *disp)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);
    rotation = LV_DISPLAY_ROTATION_90;
    switch (rotation)
    {
    case LV_DISPLAY_ROTATION_0:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISPLAY_ROTATION_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISPLAY_ROTATION_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISPLAY_ROTATION_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    lvgl_port_update_callback(disp);
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    uint32_t time_threshold_ms = 1000 / CONFIG_FREERTOS_HZ;
    while (1)
    {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, time_threshold_ms);
        usleep(1000 * time_till_next_ms);
    }
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_get_level_task(void *arg)
{
    uint32_t io_num;
    bool gpio_high_level_flag = false;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            if (io_num == MAIN_BUTTON)
            {
                if (gpio_get_level(io_num) && gpio_high_level_flag == false)
                {
                    gpio_high_level_flag = true;
                    click_button_pos++;
                    lv_obj_send_event(btn_register, LV_EVENT_CLICKED, display);
                }
                else if (gpio_get_level(io_num) == 0)
                {
                    gpio_high_level_flag = false;
                }
            }
        }
    }
}

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
        .bck_io_num = STD_BCLK_IO1,
        .ws_io_num = STD_WS_IO1,
        .data_out_num = -1,
        .data_in_num = STD_DOUT_IO1};

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
        // y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] +
        //                         y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) /
        //                        FFT_SIZE);

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
    // printf("FFT calculation time: %u cycles\n", end_r2 - start_r2);
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
    float fft_max_vals[3] = {0};
    int fft_max_freq[3] = {0};
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
                        fft_max_vals[0] += fft_max_vals[0];
                        fft_max_vals[1] += fft_max_vals[1];
                        fft_max_vals[2] += fft_max_vals[2];
                        fft_max_freq[0] += fft_max_freq[0];
                        fft_max_freq[1] += fft_max_freq[1];
                        fft_max_freq[2] += fft_max_freq[2];
                    }
                }
                uint8_t fft_iterarion = (sample_count / FFT_SIZE) - 1;
                fft_max_vals[0] = fft_max_vals[0] / fft_iterarion;
                fft_max_vals[1] = fft_max_vals[1] / fft_iterarion;
                fft_max_vals[2] = fft_max_vals[2] / fft_iterarion;
                fft_max_freq[0] = fft_max_freq[0] / fft_iterarion;
                fft_max_freq[1] = fft_max_freq[1] / fft_iterarion;
                fft_max_freq[2] = fft_max_freq[2] / fft_iterarion;

                printf("Max 1: %f at freq %d\n", fft_max_vals[0], (fft_max_freq[0] * I2S_SAMPLE_RATE) / FFT_SIZE);
                printf("Max 2: %f at freq %d\n", fft_max_vals[1], (fft_max_freq[1] * I2S_SAMPLE_RATE) / FFT_SIZE);
                printf("Max 3: %f at freq %d\n", fft_max_vals[2], (fft_max_freq[2] * I2S_SAMPLE_RATE) / FFT_SIZE);
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
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MAIN_BUTTON, gpio_isr_handler, (void *)MAIN_BUTTON);
}

void spi_configuration()
{
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle)); /*  */
}

void display_config()
{
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}

void initialize_lvgl()
{
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // Create the LVGL display
    display = lv_display_create(LCD_H_RES, LCD_V_RES);

    // Allocate buffers for LVGL
    size_t draw_buffer_sz = LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);
    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf2);

    // Initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Associate the MIPI panel handle with the display
    lv_display_set_user_data(display, panel_handle);

    // Set color depth and format
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);

    // Set the callback to flush rendered images to the display
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    ESP_LOGI(TAG, "Install LVGL tick timer");

    // Tick interface for LVGL using esp_timer
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");

    // Register callback for flush ready notification
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));
}

static void btn_cb(lv_event_t *e)
{
    if (click_button_pos % 2)
    {
        lv_obj_set_style_bg_color(btn_register, lv_color_hex(0xFFA500), LV_PART_MAIN);
        lv_obj_set_style_bg_color(btn_indentify, lv_color_hex(0x0000FF), LV_PART_MAIN);
    }
    else
    {
        lv_obj_set_style_bg_color(btn_register, lv_color_hex(0x0000FF), LV_PART_MAIN);
        lv_obj_set_style_bg_color(btn_indentify, lv_color_hex(0xFFA500), LV_PART_MAIN);
    }
}

void example_lvgl_demo_ui(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);

    btn_register = lv_button_create(scr);
    lv_obj_t *lbl_register = lv_label_create(btn_register);
    lv_label_set_text_static(lbl_register, LV_SYMBOL_UPLOAD " CADASTRAR");
    lv_obj_align(btn_register, LV_ALIGN_TOP_MID, 0, 60);
    /*Button event*/
    lv_obj_add_event_cb(btn_register, btn_cb, LV_EVENT_CLICKED, disp);

    btn_indentify = lv_button_create(scr);
    lv_obj_t *lbl_identify = lv_label_create(btn_indentify);
    lv_label_set_text_static(lbl_identify, LV_SYMBOL_DOWNLOAD " IDENTIFICAR");
    lv_obj_align(btn_indentify, LV_ALIGN_BOTTOM_MID, 0, -60);
}
void app_main(void)
{
    i2s_install();
    i2s_setpin();
    gpio_config_pin();
    spi_configuration();
    display_config();
    initialize_lvgl();

    ESP_LOGI(TAG, "Create GPIO task");
    xTaskCreate(gpio_get_level_task, "gpio_get_level_task", 2048, NULL, 10, NULL);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    _lock_acquire(&lvgl_api_lock);
    example_lvgl_demo_ui(display);
    _lock_release(&lvgl_api_lock);
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreate(mic_read_task, "mic_read_task", 4096, NULL, 5, NULL);
}