#ifndef WS2812_H
#define WS2812_H

#include "driver/rmt.h"
#include "esp_err.h"
#include "driver/gpio.h"

typedef struct {
    gpio_num_t gpio_num;    // gpio_num
    uint8_t number_leds;        // number of LEDs
    rmt_channel_t rmt_channel = RMT_CHANNEL_0;
} ws2812Config_t;

class WS2812
{
public:
    esp_err_t init(ws2812Config_t config);
    esp_err_t setPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t clear();
    esp_err_t show();

private:
    // gpio_num_t gpio_num;
    uint16_t num_leds;
    rmt_channel_t rmt_ch;
    uint8_t *leds;
};

#endif // WS2812_H