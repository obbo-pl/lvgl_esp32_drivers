/**
    @file    uc8179c.h
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

#ifndef _LVGL_UC8179C_H_
#define _LVGL_UC8179C_H_

#include <lvgl.h>
#include "disp_spi.h"
#include "uc8179c_lut.h"
#include "sdkconfig.h"


#define PIN_DC              CONFIG_LV_DISP_PIN_DC
#define PIN_RST             CONFIG_LV_DISP_PIN_RST
#define PIN_BUSY            CONFIG_LV_DISP_PIN_BUSY

#define EPD_WIDTH           LV_HOR_RES_MAX
#define EPD_HEIGHT          LV_VER_RES_MAX


#define UC8179C_CMD_PANEL_SETTING                   0x00
#define UC8179C_CMD_POWER_SETTING                   0x01
#define UC8179C_CMD_POWER_OFF                       0x02
#define UC8179C_CMD_POWER_OFF_SEQUENCE_SETTING      0x03
#define UC8179C_CMD_POWER_ON                        0x04
#define UC8179C_CMD_BOOSTER_SOFT_START              0x06
#define UC8179C_CMD_DEEP_SLEEP                      0x07
#define UC8179C_CMD_DATA_START_TRANSMISSION_1       0x10
#define UC8179C_CMD_DISPLAY_REFRESH                 0x12
#define UC8179C_CMD_DATA_START_TRANSMISSION_2       0x13
#define UC8179C_CMD_LUTC                            0x20
#define UC8179C_CMD_LUTWW                           0x21
#define UC8179C_CMD_LUTKW                           0x22
#define UC8179C_CMD_LUTR                            0x22
#define UC8179C_CMD_LUTWK                           0x23
#define UC8179C_CMD_LUTW                            0x23
#define UC8179C_CMD_LUTKK                           0x24
#define UC8179C_CMD_LUTK                            0x24
#define UC8179C_CMD_BORDER_LUT                      0x25
#define UC8179C_CMD_LUT_OPTION                      0x2A
#define UC8179C_CMD_PLL_CONTROL                     0x30
#define UC8179C_CMD_TEMPERATURE_SENSOR_CALIBRATION  0x40
#define UC8179C_CMD_VCOM_AND_DATA_INTERVAL_SETTING  0x50
#define UC8179C_CMD_TCON_SETTING                    0x60
#define UC8179C_CMD_RESOLUTION_SETTING              0x61
#define UC8179C_CMD_GATE_SOURCE_START_SETTING       0x65
#define UC8179C_CMD_REVISION                        0x70
#define UC8179C_CMD_VCOM_DC_SETTING                 0x82
#define UC8179C_CMD_PROGRAM_MODE                    0xA0
#define UC8179C_CMD_ACTIVE_PROGRAM                  0xA1
#define UC8179C_CMD_READ_OTP_DATA                   0xA2

#define UC8179C_DEEP_SLEEP_CHECK_CODE               0xA5
#define UC8179C_PLL_CONTROL_5HZ                     0x00
#define UC8179C_PLL_CONTROL_10HZ                    0x01
#define UC8179C_PLL_CONTROL_15HZ                    0x02
#define UC8179C_PLL_CONTROL_20HZ                    0x03
#define UC8179C_PLL_CONTROL_30HZ                    0x04
#define UC8179C_PLL_CONTROL_40HZ                    0x05
#define UC8179C_PLL_CONTROL_50HZ                    0x06
#define UC8179C_PLL_CONTROL_60HZ                    0x07
#define UC8179C_PLL_CONTROL_70HZ                    0x08
#define UC8179C_PLL_CONTROL_80HZ                    0x09
#define UC8179C_PLL_CONTROL_90HZ                    0x0A
#define UC8179C_PLL_CONTROL_100HZ                   0x0B
#define UC8179C_PLL_CONTROL_110HZ                   0x0C
#define UC8179C_PLL_CONTROL_130HZ                   0x0D
#define UC8179C_PLL_CONTROL_150HZ                   0x0E
#define UC8179C_PLL_CONTROL_200HZ                   0x0F
#define UC8179C_REVISION_DATA_LENGTH                7
#define UC8179C_TEMPERATURE_SENSOR_DATA_LENGTH      2

#define UC8179C_BUSY_LEVEL                          0


void uc8179c_init();
void uc8179c_lv_set_fb_cb(struct _disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                          lv_color_t color, lv_opa_t opa);

void uc8179c_lv_rounder_cb(struct _disp_drv_t *disp_drv, lv_area_t *area);
void uc8179c_lv_fb_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);


// Several functions extending the capabilities of the driver, not required by LVGL.
// The following functions should be used before starting a GUI thread or while the GUI is idle.
void uc8179c_set_lut_from_otp();
void uc8179c_set_lut_from_reg(const unsigned char *epd_lut_vcom,
                              const unsigned char *epd_lut_ww,
                              const unsigned char *epd_lut_r,
                              const unsigned char *epd_lut_w,
                              const unsigned char *epd_lut_k,
                              const unsigned char *epd_lut_bd);
void uc8179c_clear_screen();
// With 1-bit color depth, LV_COLOR_BLACK will be displayed as black, other colors will be displayed as white.
// For 8-bit depth and greater, by default black is defined as LV_COLOR_BLACK and red as LV_COLOR_RED, other
// colors will be displayed as white background.
// Here you can define which color should be displayed as black and which should be displayed as red.
void uc8179c_set_palette(lv_color_t black, lv_color_t red);

void uc8179c_set_flush_ready_cb(void (*ready_cb)());

#ifdef LV_SPI_DRIVER_SUPPORT_READ_DATA
#if defined (CONFIG_LV_DISP_SPI_MISO) && (CONFIG_LV_DISP_SPI_MISO >= 0)
// Using the read operation may require lowering the SPI clock frequency and adding a delay before reading.
#define LV_DRIVER_SUPPORT_READ_DATA
void uc8179c_get_revision(uint8_t *data);
int8_t uc8179c_get_temperature_raw();
#endif
#endif


#endif // _LVGL_UC8179C_H_
