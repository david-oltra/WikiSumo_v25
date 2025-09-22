#include <tb6612.h>
#include <driver/gpio.h>
#include <driver/ledc.h>


void TB6612::init(tb6612Config_t config)
{
    motorA1 = config.motorA1;
    motorA2 = config.motorA2;
    motorAPwm = config.motorAPwm;
    motorAChannel = config.motorAChannel;
    motorB1 = config.motorB1;
    motorB2 = config.motorB2;
    motorBPwm = config.motorBPwm;
    motorBChannel = config.motorBChannel;

    gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << motorA1) | (1ULL << motorA2) | (1ULL << motorB1) | (1ULL << motorB2);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(motorA1, 0);
    gpio_set_level(motorA2, 0);
    gpio_set_level(motorB1, 0);
    gpio_set_level(motorB2, 0);

    // ledc_timer_config_t ledc_timer;
    //     ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    //     ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT;
    //     ledc_timer.timer_num        = LEDC_TIMER_0;
    //     ledc_timer.freq_hz          = 4000;
    //     ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    //     ledc_timer.deconfigure      = 0;
    // ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // ledc_channel_config_t ledc_channel;
    //     ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    //     ledc_channel.channel        = motorAChannel;
    //     ledc_channel.timer_sel      = LEDC_TIMER_0;
    //     ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    //     ledc_channel.gpio_num       = motorAPwm;
    //     ledc_channel.duty           = 0; // 0 -> 2^13 -1 = 8192 - 1
    //     ledc_channel.hpoint         = 0;
    //     ledc_channel.flags.output_invert  = 0;
    //     ledc_channel.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
    // ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    //     ledc_channel.channel        = motorBChannel;
    //     ledc_channel.gpio_num       = motorBPwm;
    // ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void TB6612::setDirection(enum DIRECTION dir)
{
    switch(dir)
    {
        case STOP:
            gpio_set_level(motorA1,1);
            gpio_set_level(motorA2,1);
            gpio_set_level(motorB1,1);
            gpio_set_level(motorB2,1);
            break;
        case FWD:
            gpio_set_level(motorA1,0);
            gpio_set_level(motorA2,1);
            gpio_set_level(motorB1,1);
            gpio_set_level(motorB2,0);
            break;
        case LEFT:
            gpio_set_level(motorA1,1);
            gpio_set_level(motorA2,0);
            gpio_set_level(motorB1,1);
            gpio_set_level(motorB2,0);
            break;
        case RIGHT:
            gpio_set_level(motorA1,0);
            gpio_set_level(motorA2,1);
            gpio_set_level(motorB1,0);
            gpio_set_level(motorB2,1);
            break;
        case BACK:
            gpio_set_level(motorA1,1);
            gpio_set_level(motorA2,0);
            gpio_set_level(motorB1,0);
            gpio_set_level(motorB2,1);
        default:
            break;
    }
}

void TB6612::setSpeed(uint16_t speed)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motorAChannel, abs(speed));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motorAChannel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motorBChannel, abs(speed));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motorBChannel);
}