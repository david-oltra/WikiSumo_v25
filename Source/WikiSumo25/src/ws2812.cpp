#include "ws2812.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "WS2812";

esp_err_t WS2812::init(ws2812Config_t config)
{
    num_leds = config.number_leds;
    rmt_ch = config.rmt_channel;

    rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX(config.gpio_num, config.rmt_channel);
    cfg.clk_div = 1;

    esp_err_t err = rmt_config(&cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error en rmt_config: %s", esp_err_to_name(err));
        return err;
    }

    err = rmt_driver_install(config.rmt_channel, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error en rmt_driver_install: %s", esp_err_to_name(err));
        return err;
    }

    leds = (uint8_t *)malloc(num_leds * 3);
    if (!leds)
    {
        ESP_LOGE(TAG, "Error al asignar memoria para los LEDs");
        return err;
    }
    memset(leds, 0, num_leds * 3);

    return err;
}


esp_err_t WS2812::setPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= num_leds)
    {
        return ESP_ERR_INVALID_ARG;
    }

    leds[index * 3] = green;
    leds[index * 3 + 1] = red;
    leds[index * 3 + 2] = blue;

    return ESP_OK;
}

esp_err_t WS2812::clear()
{
    memset(leds, 0, num_leds * 3);
    return ESP_OK;
}

esp_err_t WS2812::show()
{
    rmt_item32_t *items = (rmt_item32_t *)malloc(num_leds * 24 * sizeof(rmt_item32_t));
    if (!items)
    {
        ESP_LOGE(TAG, "Error al asignar memoria para los items RMT");
        return ESP_ERR_NO_MEM;
    }

    for (uint16_t i = 0; i < num_leds; ++i)
    {
        uint8_t *color = &leds[i * 3];
        for (uint8_t j = 0; j < 24; ++j)
        {
            uint8_t bit = (color[j / 8] >> (7 - (j % 8))) & 1;
            if (bit)
            {
                items[i * 24 + j].duration0 = 64; // T1H
                items[i * 24 + j].level0 = 1;
                items[i * 24 + j].duration1 = 36; // T1L
                items[i * 24 + j].level1 = 0;
            }
            else
            {
                items[i * 24 + j].duration0 = 32; // T0H
                items[i * 24 + j].level0 = 1;
                items[i * 24 + j].duration1 = 68; // T0L
                items[i * 24 + j].level1 = 0;
            }
        }
    }

    esp_err_t err = rmt_write_items(rmt_ch, items, num_leds * 24, true);
    free(items);
    return err;
}