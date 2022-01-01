/**
    @file    uc8179c.c
    @brief   Waveshare e-paper display with UltraChip UC8179C
    @version 1.0
    @date    2021-11-21
    @author  Krzysztof Markiewicz <obbo.pl>


    @section LICENSE

    MIT License

    Copyright (c) 2021 Krzysztof Markiewicz

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute,
    sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_err.h>
#include "disp_driver.h"
#include "uc8179c.h"


#define TAG "lv_uc8179c"

#define PIN_DC_BIT          (1ULL << PIN_DC)
#define PIN_RST_BIT         (1ULL << PIN_RST)
#define PIN_BUSY_BIT        (1ULL << PIN_BUSY)
#define EPD_ROW_LEN         ((size_t)(EPD_WIDTH / 8))

#define SET_BIT(value, bit) ((value) |= (uint8_t)(1 << (bit)))
#define CLR_BIT(value, bit) ((value) &= (uint8_t)~(1 << (bit)))
#define	HIBYTE(x)           ((uint8_t)((x) >> 8))
#define	LOBYTE(x)           ((uint8_t)(x))

void (*flush_ready_cb)() = NULL;
bool use_lut_from_register = false;
lv_color_t palette_black = LV_COLOR_BLACK;
lv_color_t palette_red = LV_COLOR_RED;
unsigned char *lut_vcom = NULL;
unsigned char *lut_ww = NULL;
unsigned char *lut_r = NULL;
unsigned char *lut_w = NULL;
unsigned char *lut_k = NULL;
unsigned char *lut_bd = NULL;


// function prototype
static void uc8179c_wait_busy(int timeout_sec);
static void uc8179c_spi_send_cmd(uint8_t cmd);
static void uc8179c_spi_send_data(uint8_t *data, size_t len);
static void uc8179c_spi_send_data_byte(uint8_t data);
#ifdef LV_DRIVER_SUPPORT_READ_DATA
static void uc8179c_spi_read_data(uint8_t *data, size_t len);
#endif
static void uc8179c_sleep();
static void uc8179c_panel_init();
static void uc8179c_full_update(uint8_t *buf);
static void uc8179c_write_lut();



static void uc8179c_wait_busy(int timeout_sec)
{
    if (timeout_sec >= 0) {
        uint16_t tic_to_sec = 20;
        uint32_t timeout_count = tic_to_sec * timeout_sec;
        if (timeout_sec == 0) timeout_count = 1;
        while ((gpio_get_level(PIN_BUSY) == UC8179C_BUSY_LEVEL) && timeout_count) {
            vTaskDelay((1000 / tic_to_sec) / portTICK_PERIOD_MS);
            if (timeout_sec > 0) timeout_count--;
        }
    }
    if (gpio_get_level(PIN_BUSY) == UC8179C_BUSY_LEVEL) ESP_LOGE( TAG, "Busy exceeded %ds", timeout_sec);
}

static void uc8179c_spi_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 0);	// DC = 0 for command
    disp_spi_send_data(&cmd, 1);
}

static void uc8179c_spi_send_data(uint8_t *data, size_t len)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 1);  // DC = 1 for data
    disp_spi_send_data(data, len);
}

static void uc8179c_spi_send_data_byte(uint8_t data)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 1);  // DC = 1 for data
    disp_spi_send_data(&data, 1);
}

#ifdef LV_DRIVER_SUPPORT_READ_DATA
static void uc8179c_spi_read_data(uint8_t *data, size_t len)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 1);  // DC = 1 for data
    disp_spi_read_data(data, len);
}

void uc8179c_get_revision(uint8_t *data)
{
    // 32-bit aligned buffer
    uint8_t temp[8];
    uc8179c_spi_send_cmd(UC8179C_CMD_REVISION);
    uc8179c_spi_read_data(temp, UC8179C_REVISION_DATA_LENGTH);
    memcpy(data, temp, UC8179C_REVISION_DATA_LENGTH);
}

int8_t uc8179c_get_temperature_raw()
{
    // 32-bit aligned buffer
    uint8_t data[4];
    uc8179c_spi_send_cmd(UC8179C_CMD_TEMPERATURE_SENSOR_CALIBRATION);
    uc8179c_spi_read_data(data, UC8179C_TEMPERATURE_SENSOR_DATA_LENGTH);
    return (int8_t)data[0];
}
#endif

static void uc8179c_sleep()
{
    uc8179c_spi_send_cmd(UC8179C_CMD_POWER_OFF);
    uc8179c_wait_busy(1);
    uc8179c_spi_send_cmd(UC8179C_CMD_DEEP_SLEEP);
    uc8179c_spi_send_data_byte(UC8179C_DEEP_SLEEP_CHECK_CODE);
}

static void uc8179c_panel_init()
{
    ESP_LOGD(TAG, "Panel init");
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    // Power up
    uc8179c_spi_send_cmd(UC8179C_CMD_POWER_ON);
    uc8179c_wait_busy(0);
    // Panel settings
    uc8179c_spi_send_cmd(UC8179C_CMD_POWER_SETTING);
    uc8179c_spi_send_data_byte(0x07);
    uc8179c_spi_send_data_byte(0x17); 		// VGH=20V,VGL=-20V,VCOM_SLEW
    uc8179c_spi_send_data_byte(0x3F); 		// VDH=15V
    uc8179c_spi_send_data_byte(0x3F); 		// VDL=-15V
    uc8179c_spi_send_data_byte(0x03); 		// default 3.0V
    uc8179c_write_lut();
    uc8179c_spi_send_cmd(UC8179C_CMD_RESOLUTION_SETTING);
    uc8179c_spi_send_data_byte(HIBYTE(EPD_WIDTH));
    uc8179c_spi_send_data_byte(LOBYTE(EPD_WIDTH));
    uc8179c_spi_send_data_byte(HIBYTE(EPD_HEIGHT));
    uc8179c_spi_send_data_byte(LOBYTE(EPD_HEIGHT));
    uc8179c_spi_send_cmd(UC8179C_CMD_VCOM_DC_SETTING);
    uc8179c_spi_send_data_byte(0x04); 		// -0.3V
    uc8179c_spi_send_cmd(UC8179C_CMD_VCOM_AND_DATA_INTERVAL_SETTING);
    uc8179c_spi_send_data_byte(0x11);
    uc8179c_spi_send_data_byte(0x07);
    uc8179c_spi_send_cmd(UC8179C_CMD_TCON_SETTING);
    uc8179c_spi_send_data_byte(0x22);
}

void uc8179c_clear_screen()
{
    // Use LUT from OTP
    bool _use_lut_from_register_temp = use_lut_from_register;
    use_lut_from_register = false;
    uc8179c_panel_init();
    use_lut_from_register = _use_lut_from_register_temp;

    uc8179c_spi_send_cmd(UC8179C_CMD_DATA_START_TRANSMISSION_1);
    for (uint16_t x = 0; x < EPD_HEIGHT; x++) {
    	for (uint16_t y = 0 ; y < EPD_WIDTH / 8; y++) {
            uc8179c_spi_send_data_byte(0xFF);
    	}
    }
    uc8179c_spi_send_cmd(UC8179C_CMD_DATA_START_TRANSMISSION_2);
    for (uint16_t x = 0; x < EPD_HEIGHT; x++) {
    	for (uint16_t y = 0 ; y < EPD_WIDTH / 8; y++) {
            uc8179c_spi_send_data_byte(0x00);
    	}
    }
    uc8179c_spi_send_cmd(UC8179C_CMD_DISPLAY_REFRESH);
    vTaskDelay(pdMS_TO_TICKS(10));
    uc8179c_wait_busy(150);
    uc8179c_sleep();
}

static void uc8179c_full_update(uint8_t *buf)
{
    uc8179c_panel_init();
    uint8_t *buf_ptr = buf;
    uc8179c_spi_send_cmd(UC8179C_CMD_DATA_START_TRANSMISSION_1);
    for (size_t h_idx = 0; h_idx < EPD_HEIGHT; h_idx++) {
        uc8179c_spi_send_data(buf_ptr, EPD_ROW_LEN);
        buf_ptr += EPD_ROW_LEN;
    }
    uc8179c_spi_send_cmd(UC8179C_CMD_DATA_START_TRANSMISSION_2);
#if LV_COLOR_DEPTH == 1
    for (uint16_t x = 0; x < EPD_HEIGHT; x++) {
    	for (uint16_t y = 0 ; y < EPD_WIDTH / 8; y++) {
            uc8179c_spi_send_data_byte(0x00);
        }
    }
#else
    for (size_t h_idx = 0; h_idx < EPD_HEIGHT; h_idx++) {
        uc8179c_spi_send_data(buf_ptr, EPD_ROW_LEN);
        buf_ptr += EPD_ROW_LEN;
    }
#endif
    uc8179c_spi_send_cmd(UC8179C_CMD_DISPLAY_REFRESH);
    vTaskDelay(pdMS_TO_TICKS(10));
    uc8179c_wait_busy(150);
    uc8179c_sleep();
}

void uc8179c_lv_fb_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    size_t len = ((area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1)) / 8;
    ESP_LOGD(TAG, "x1: 0x%x, x2: 0x%x, y1: 0x%x, y2: 0x%x", area->x1, area->x2, area->y1, area->y2);
    ESP_LOGD(TAG, "Writing LVGL fb with len: %u", len);
    uc8179c_full_update((uint8_t *)color_map);
    if (flush_ready_cb != NULL) flush_ready_cb();
    lv_disp_flush_ready(drv);
    ESP_LOGD(TAG, "Flush ready");
}

void uc8179c_lv_set_fb_cb(struct _disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                          lv_color_t color, lv_opa_t opa)
{
    uint32_t byte_index = (uint32_t)((x >> 3) + (y * EPD_ROW_LEN));
    uint8_t bit_index = 7 - (uint8_t)(x & 0x07);
#if LV_COLOR_DEPTH == 1
    if (color.full == palette_black.full) {
    	ESP_LOGD(TAG, "Set black at x: %u, y: %u", x, y);
        CLR_BIT(buf[byte_index], bit_index);
    } else {
    	ESP_LOGD(TAG, "Set white at x: %u, y: %u", x, y);
        SET_BIT(buf[byte_index], bit_index);
    }
#else
    uint32_t offset = EPD_ROW_LEN * EPD_HEIGHT;
    if (color.full == palette_black.full) {
    	ESP_LOGD(TAG, "Set black at x: %u, y: %u", x, y);
        CLR_BIT(buf[byte_index], bit_index);
        CLR_BIT(buf[byte_index + offset], bit_index);
    } else if (color.full == palette_red.full) {
    	ESP_LOGD(TAG, "Set red at x: %u, y: %u", x, y);
        CLR_BIT(buf[byte_index], bit_index);
    	SET_BIT(buf[byte_index + offset], bit_index);
    } else {
    	ESP_LOGD(TAG, "Set white at x: %u, y: %u", x, y);
        SET_BIT(buf[byte_index], bit_index);
        CLR_BIT(buf[byte_index + offset], bit_index);
    }
#endif
}

void uc8179c_lv_rounder_cb(struct _disp_drv_t *disp_drv, lv_area_t *area)
{
    area->x1 = 0;
    area->y1 = 0;
    area->x2 = EPD_WIDTH - 1;
    area->y2 = EPD_HEIGHT - 1;
}

void uc8179c_init()
{
    // Setup DC, RST pins
    gpio_config_t out_io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = PIN_DC_BIT | PIN_RST_BIT,
            .pull_down_en = 0,
            .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&out_io_conf));
    // Setup BUSY pin
    gpio_config_t in_io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = PIN_BUSY_BIT,
            .pull_down_en = 0,
            .pull_up_en = 1,
    };
    ESP_ERROR_CHECK(gpio_config(&in_io_conf));
    uc8179c_panel_init();
    ESP_LOGD(TAG, "Panel initialized");
}

static void uc8179c_write_lut()
{
    if (use_lut_from_register) {
        uc8179c_spi_send_cmd(UC8179C_CMD_PANEL_SETTING);
        uc8179c_spi_send_data_byte(0x2F); 	// LUT from register, KWR, Scan up, Shift right, Booster ON, No RST
        uc8179c_spi_send_cmd(UC8179C_CMD_PLL_CONTROL);
        uc8179c_spi_send_data_byte(UC8179C_PLL_CONTROL_80HZ);
        if (lut_vcom != NULL) {
            uc8179c_spi_send_cmd(UC8179C_CMD_LUTC);
            for (int i = 0; i < EPD_LUTC_SIZE; i++) {
                uc8179c_spi_send_data_byte(lut_vcom[i]);
            }
        }
        if (lut_ww != NULL) {
            uc8179c_spi_send_cmd(UC8179C_CMD_LUTWW);
            for (int i = 0; i < EPD_LUTWW_SIZE; i++) {
                uc8179c_spi_send_data_byte(lut_ww[i]);
            }
        }
        if (lut_r != NULL) {
            uc8179c_spi_send_cmd(UC8179C_CMD_LUTR);
            for (int i = 0; i < EPD_LUTR_SIZE; i++) {
                uc8179c_spi_send_data_byte(lut_r[i]);
            }
        }
        if (lut_w != NULL) {
            uc8179c_spi_send_cmd(UC8179C_CMD_LUTW);
            for (int i = 0; i < EPD_LUTW_SIZE; i++) {
                uc8179c_spi_send_data_byte(lut_w[i]);
            }
        }
        if (lut_k != NULL) {
            uc8179c_spi_send_cmd(UC8179C_CMD_LUTK);
            for (int i = 0; i < EPD_LUTK_SIZE; i++) {
                uc8179c_spi_send_data_byte(lut_k[i]);
            }
        }
        if (lut_bd != NULL) {
            uc8179c_spi_send_cmd(UC8179C_CMD_BORDER_LUT);
            for (int i = 0; i < EPD_LUTBD_SIZE; i++) {
                uc8179c_spi_send_data_byte(lut_bd[i]);
            }
        }
    } else {
        uc8179c_spi_send_cmd(UC8179C_CMD_PANEL_SETTING);
        uc8179c_spi_send_data_byte(0x0F); 	// LUT from OTP, KWR, Scan up, Shift right, Booster ON, No RST
        uc8179c_spi_send_cmd(UC8179C_CMD_PLL_CONTROL);
        uc8179c_spi_send_data_byte(UC8179C_PLL_CONTROL_50HZ);
    }
}

void uc8179c_set_lut_from_reg(const unsigned char *epd_lut_vcom,
                              const unsigned char *epd_lut_ww,
                              const unsigned char *epd_lut_r,
                              const unsigned char *epd_lut_w,
                              const unsigned char *epd_lut_k,
                              const unsigned char *epd_lut_bd)
{
    lut_vcom = (unsigned char *)(epd_lut_vcom);
    lut_ww = (unsigned char *)(epd_lut_ww);
    lut_r = (unsigned char *)(epd_lut_r);
    lut_w = (unsigned char *)(epd_lut_w);
    lut_k = (unsigned char *)(epd_lut_k);
    lut_bd = (unsigned char *)(epd_lut_bd);
    use_lut_from_register = true;
}

void uc8179c_set_lut_from_otp()
{
    use_lut_from_register = false;
    lut_vcom = NULL;
    lut_ww = NULL;
    lut_r = NULL;
    lut_w = NULL;
    lut_k = NULL;
    lut_bd = NULL;
}

void uc8179c_set_palette(lv_color_t black, lv_color_t red)
{
    palette_black = black;
    palette_red = red;
}

void uc8179c_set_flush_ready_cb(void (*ready_cb)())
{
    flush_ready_cb = ready_cb;
}

