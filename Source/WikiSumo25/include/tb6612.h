#ifndef __TB6612_H__
#define __TB6612_H__

#include <driver/gpio.h>
#include <driver/ledc.h>

typedef struct {
    gpio_num_t motorA1;
    gpio_num_t motorA2;
    gpio_num_t motorAPwm;
    ledc_channel_t motorAChannel;
    gpio_num_t motorB1;
    gpio_num_t motorB2;
    gpio_num_t motorBPwm;
    ledc_channel_t motorBChannel;
} tb6612Config_t;

class TB6612
{
    public:
        
        enum DIRECTION
        {
            STOP = 0,
            FWD,
            LEFT,
            RIGHT,
            BACK
        };

        void init(tb6612Config_t config);

        /*
        1 1 1 1 -> STOP
        0 1 0 1 -> FWD
        1 0 1 0 -> BACK
        1 0 0 1 -> IZQ
        0 1 1 0 -> DRCH
        */
        void setDirection(enum DIRECTION dir);
        
        void setSpeed(uint16_t speed);

    private:
        
        gpio_num_t motorA1, motorA2, motorAPwm, motorB1, motorB2, motorBPwm;
        ledc_channel_t motorAChannel, motorBChannel;
};

#endif