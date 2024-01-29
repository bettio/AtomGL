/*
 * This file is part of AtomGL.
 *
 * Copyright 2024 Davide Bettio <davide@uninstall.it>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/task.h>

#include <context.h>
#include <interop.h>

#include <i2c_driver.h>

#define TAG "SSD1306"

#define DISPLAY_WIDTH 128
#define CHAR_WIDTH 8

// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define OLED_I2C_ADDRESS 0x3C

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE 0x80
#define OLED_CONTROL_BYTE_CMD_STREAM 0x00
#define OLED_CONTROL_BYTE_DATA_STREAM 0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST 0x81 // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM 0xA4
#define OLED_CMD_DISPLAY_ALLON 0xA5
#define OLED_CMD_DISPLAY_NORMAL 0xA6
#define OLED_CMD_DISPLAY_INVERTED 0xA7
#define OLED_CMD_DISPLAY_OFF 0xAE
#define OLED_CMD_DISPLAY_ON 0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE 0x20 // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE 0x21 // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE 0x22 // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP 0xA1
#define OLED_CMD_SET_MUX_RATIO 0xA8 // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE 0xC8
#define OLED_CMD_SET_DISPLAY_OFFSET 0xD3 // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP 0xDA // follow with 0x12
#define OLED_CMD_NOP 0xE3 // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV 0xD5 // follow with 0x80
#define OLED_CMD_SET_PRECHARGE 0xD9 // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT 0xDB // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP 0x8D // follow with 0x14

// TODO: let's change name, since also non SPI display are supported now
struct SPI
{
    term i2c_host;
    Context *ctx;
};

static void do_update(Context *ctx, term display_list);

#include "font.c"
#include "display_items.h"
#include "draw_common.h"
#include "monochrome.h"
#include "message_helpers.h"

static void do_update(Context *ctx, term display_list)
{
    int proper;
    int len = term_list_length(display_list, &proper);

    BaseDisplayItem *items = malloc(sizeof(BaseDisplayItem) * len);

    term t = display_list;
    for (int i = 0; i < len; i++) {
        init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }

    int screen_width = DISPLAY_WIDTH;
    int screen_height = 64;
    struct SPI *spi = ctx->platform_data;

    int memsize = (DISPLAY_WIDTH * (64 + 1)) / sizeof(uint8_t);
    uint8_t *buf = malloc(memsize);
    memset(buf, 0, memsize);

    i2c_port_t i2c_num;
    if (i2c_driver_acquire(spi->i2c_host, &i2c_num, ctx->global) != I2CAcquireOk) {
        fprintf(stderr, "Invalid I2C peripheral\n");
        return;
    }

    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(buf, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        uint8_t *out_buf = buf + (DISPLAY_WIDTH / 8);
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
            out_buf[i] |= ((~buf[i / 8] >> (i % 8)) & 1) << (ypos % 8);
        }

        if ((ypos % 8) == 7) {
            i2c_cmd_handle_t cmd;
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
            i2c_master_write_byte(cmd, 0xB0 | ypos / 8, true);
            i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
            for (uint8_t j = 0; j < DISPLAY_WIDTH; j++) {
                i2c_master_write_byte(cmd, out_buf[j], true);
            }
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);

            memset(buf, 0, memsize);
        }
    }

    i2c_driver_release(spi->i2c_host, ctx->global);

    free(buf);
    destroy_items(items, len);
}

static void display_init(Context *ctx, term opts)
{
    GlobalContext *glb = ctx->global;

    term i2c_host
        = interop_kv_get_value_default(opts, ATOM_STR("\x8", "i2c_host"), term_invalid_term(), glb);
    if (i2c_host == term_invalid_term()) {
        ESP_LOGE(TAG, "Missing i2c_host config option.");
        return;
    }

    display_messages_queue = xQueueCreate(32, sizeof(Message *));

    struct SPI *spi = malloc(sizeof(struct SPI));
    ctx->platform_data = spi;

    spi->ctx = ctx;

    gpio_set_direction(16, GPIO_MODE_OUTPUT);
    gpio_set_level(16, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(16, 1);

    i2c_port_t i2c_num;
    if (i2c_driver_acquire(i2c_host, &i2c_num, glb) != I2CAcquireOk) {
        fprintf(stderr, "Invalid I2C peripheral\n");
        return;
    }
    spi->i2c_host = i2c_host;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
    i2c_master_write_byte(cmd, 0x14, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true);

    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
    i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(i2c_num, cmd, 50 / portTICK_PERIOD_MS);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "ssd1306 OLED configuration failed. error: 0x%.2X", res);
    } else {
        xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
    }

    i2c_cmd_link_delete(cmd);
    i2c_driver_release(i2c_host, glb);
}

Context *ssd1306_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_init(ctx, opts);

    return ctx;
}
