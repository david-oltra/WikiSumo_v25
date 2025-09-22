/**
 *  TODO
 *  [✔] WS2812
 *  [✔] TB6612
 *  [✔] VL53L0X
 *  [✔] QRE1113
 *  [ ] ESPNOW
 *  [*] STRATEGIES
 */

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/adc.h>

#include "espnow.h"
#include "ws2812.h"
#include "tb6612.h"
#include "VL53L0X.h"

#define REMOTE_CONTROL

#define I2C_PORT I2C_NUM_0
#define SDA GPIO_NUM_20
#define SCL GPIO_NUM_21
#define CHANNEL_QRE_L ADC1_CHANNEL_1
#define CHANNEL_QRE_R ADC1_CHANNEL_2

#define timeToStart 5000 // [ms]
#define tofRangeDetect 100 // [mm]
#define timeMove 1000 // [ms]
#define strategyTime 500 // [ms]
#define brightness 10 // 0-255

uint8_t vl53l0xError = 0;
uint8_t qre1113Error = 0;

extern "C" void app_main();

WS2812 led;
TB6612 motors;
VL53L0X tofL(I2C_PORT, GPIO_NUM_7);
VL53L0X tofC(I2C_PORT, GPIO_NUM_9);
VL53L0X tofR(I2C_PORT, GPIO_NUM_10);

typedef struct {
    uint16_t qreSensorRRef = 0;
    uint16_t qreSensorLRef = 0;
} qreParameterToPass_t;
qreParameterToPass_t qreParameterToPass;

static char TAG[] = "MAIN";

void espnowInit()
{
    if(init_wifi() == ESP_OK)
    {
        ESP_LOGI(TAG, "WiFi Init Completed");
    }
    else{
        ESP_LOGE(TAG, "WiFi Init Error");
    }

    if(init_esp_now() == ESP_OK)
    {
        ESP_LOGI(TAG, "EspNow Init Completed");
    }
    else{
        ESP_LOGE(TAG, "EspNow Init Error");
    } 

    if(register_peer(peer_mac) == ESP_OK)
    {
        ESP_LOGI(TAG, "Register Peer Completed");
    }
    else{
        ESP_LOGE(TAG, "Register Peer Error");
    }
}

void buttonsInit()
{
    ESP_LOGW(TAG,"BUTTONS INIT");
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_12);
    ESP_LOGI(TAG,"ADC_WIDTH_BIT_12|\tADC1_CHANNEL_0,ADC_ATTEN_DB_12");
    ESP_LOGW(TAG,"BUTTONS INIT OK");
}

void ws2812Init()
{
    ESP_LOGW(TAG,"WS2812 INIT");
    ws2812Config_t ws2812_config;
        ws2812_config.gpio_num = GPIO_NUM_8;
        ws2812_config.number_leds = 3;
        ws2812_config.rmt_channel = RMT_CHANNEL_0;
        
    led.init(ws2812_config);

    vTaskDelay(pdMS_TO_TICKS(200));
    led.clear();
    led.setPixel(0,0,0,brightness);
    led.setPixel(1,0,brightness,0);
    led.setPixel(2,brightness,0,0);
    led.show();
    vTaskDelay(pdMS_TO_TICKS(1000));
    led.clear();
    led.setPixel(0,0,0,0);
    led.setPixel(1,0,0,0);
    led.setPixel(2,0,0,0);
    led.show();

    ESP_LOGW(TAG,"WS2812 INIT OK");
}

void tb6612Init()
{
    ESP_LOGW(TAG,"TB6612 INIT");
    tb6612Config_t tb6612_config;
        tb6612_config.motorA1 = GPIO_NUM_3;
        tb6612_config.motorA2 = GPIO_NUM_4;
        tb6612_config.motorB1 = GPIO_NUM_5;
        tb6612_config.motorB2 = GPIO_NUM_6;

    motors.init(tb6612_config);

    gpio_set_level(GPIO_NUM_3,0);
    gpio_set_level(GPIO_NUM_4,0);
    gpio_set_level(GPIO_NUM_5,0);
    gpio_set_level(GPIO_NUM_6,0);

    ESP_LOGW(TAG,"TB6612 INIT OK");
}

void vl53l0xInit()
{
    ESP_LOGW(TAG,"VL53L0X INIT");

    gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_7) | (1ULL<<GPIO_NUM_9) | (1ULL<<GPIO_NUM_10);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_7,0);
    gpio_set_level(GPIO_NUM_9,0);
    gpio_set_level(GPIO_NUM_10,0);

    vTaskDelay(pdMS_TO_TICKS(500));
    tofL.i2cMasterInit(SDA,SCL);
    vTaskDelay(pdMS_TO_TICKS(500));
    if (!tofL.init()) { 
        ESP_LOGE(TAG, "Failed to initialize VL53L0X L :("); 
        robot_data.vl53l0x_error = 1;
        led.clear();
        led.setPixel(2, brightness, 0, 0);
        led.show();
    }
    if(tofL.setDeviceAddress(0x31))
    {
        led.clear();
        led.setPixel(2, 0, brightness, 0);
        led.show();
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X L :("); 
        robot_data.vl53l0x_error = 1;
        led.clear();
        led.setPixel(2, brightness, 0, 0);
        led.show();
    }
    tofC.reset();
    tofR.reset();
    vTaskDelay(pdMS_TO_TICKS(250));
    if (!tofC.init()) { 
        ESP_LOGE(TAG, "Failed to initialize VL53L0X C :("); 
        robot_data.vl53l0x_error = 1;
        led.clear();
        led.setPixel(1, brightness, 0, 0);
        led.show();
    }
    if (tofC.setDeviceAddress(0x30))
    {
        led.setPixel(1, 0, brightness, 0);
        led.show();
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X C :("); 
        robot_data.vl53l0x_error = 1;
        led.clear();
        led.setPixel(1, brightness, 0, 0);
        led.show();
    }
    tofR.reset();
    if (!tofR.init()) { 
        ESP_LOGE(TAG, "Failed to initialize VL53L0X R :("); 
        robot_data.vl53l0x_error = 1;
        led.clear();
        led.setPixel(0, brightness, 0, 0);
        led.show();
    }
    led.setPixel(0, 0, brightness, 0);
    led.show();
    
    if(!robot_data.vl53l0x_error)
    {
        ESP_LOGW(TAG,"VL53L0X INIT OK");
    }
}

void vl53l0xTask( void * pvParameters )
{
    uint8_t dir = TB6612::STOP;
    uint16_t result_mm = 0;
    for(;;)
    {
        tofC.read(&result_mm);
        if (result_mm < tofRangeDetect)
        {
            if(dir!=TB6612::FWD){vTaskDelay(pdMS_TO_TICKS(100));};
            dir = TB6612::FWD;
            motors.setDirection(TB6612::FWD);
            led.clear();
            led.setPixel(1,0,brightness,0);
            led.show();
        }
        // ESP_LOGI("TOFC","%u",result_mm);
        else
        {
            tofL.read(&result_mm);
            if (result_mm < tofRangeDetect)
            {
                if(dir!=TB6612::LEFT){vTaskDelay(pdMS_TO_TICKS(100));};
                dir = TB6612::LEFT;
                motors.setDirection(TB6612::LEFT);
                led.clear();
                led.setPixel(2,0,brightness,0);
                led.show();
            }
            // ESP_LOGI("TOFL","%u",result_mm);
            tofR.read(&result_mm);
            if (result_mm < tofRangeDetect)
            {
                if(dir!=TB6612::RIGHT){vTaskDelay(pdMS_TO_TICKS(100));};
                dir = TB6612::RIGHT;
                motors.setDirection(TB6612::RIGHT);
                led.clear();
                led.setPixel(0,0,brightness,0);
                led.show();
            }
            // ESP_LOGI("TOFR","%u",result_mm);
        }

        if (adc1_get_raw(ADC1_CHANNEL_0)>2000 || remote_data.start)
        {
            led.clear();
            led.setPixel(0,0,0,0);
            led.setPixel(1,0,0,0);
            led.setPixel(2,0,0,0);
            led.show();
            motors.setDirection(TB6612::STOP);
            esp_restart();
        }
    }
}

uint8_t qre1113Calibrate(qreParameterToPass_t &qreParameterToPass)
{
    robot_data.qre1113_calibrate = 0;

    uint16_t qreSensorL = adc1_get_raw(CHANNEL_QRE_L);
    uint16_t qreSensorR = adc1_get_raw(CHANNEL_QRE_R);
    qreParameterToPass.qreSensorLRef = adc1_get_raw(CHANNEL_QRE_L);
    qreParameterToPass.qreSensorRRef = adc1_get_raw(CHANNEL_QRE_R);

    // CALIBRATE
    for (uint16_t i=0; i<50000; i++)
    {
        qreSensorL = adc1_get_raw(CHANNEL_QRE_L);
        qreSensorR= adc1_get_raw(CHANNEL_QRE_R);
        // ESP_LOGI("QRE1113","%u\t%u",qreSensorL,qreSensorR);
        if (qreSensorL < qreParameterToPass.qreSensorLRef){qreParameterToPass.qreSensorLRef = qreSensorL;}
        if (qreSensorR < qreParameterToPass.qreSensorRRef){qreParameterToPass.qreSensorRRef = qreSensorR;}
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
    qreParameterToPass.qreSensorLRef -= 100;
    qreParameterToPass.qreSensorRRef -= 100;

    ESP_LOGI("QRE1113","%u\t%u",qreParameterToPass.qreSensorLRef,qreParameterToPass.qreSensorRRef);

    if((qreParameterToPass.qreSensorLRef < 200) || (qreParameterToPass.qreSensorRRef < 200))
    {
        robot_data.qre1113_error = 1;
        led.clear();
        led.setPixel(0, brightness, 0, 0);
        led.setPixel(2, brightness, 0, 0);
        led.show();
        ESP_LOGE(TAG,"QRE1113 CALIBRATE FAIL");
    }
    else
    {
        robot_data.qre1113_error = 0;
        led.clear();
        led.setPixel(0, 0, 0, brightness);
        led.setPixel(2, 0, 0, brightness);
        led.show();
        ESP_LOGW(TAG,"QRE1113 CALIBRATE OK");
    }

    return 1;
}

void qre1113Init(qreParameterToPass_t &qreParameterToPass)
{
    ESP_LOGW(TAG,"QRE1113 INIT");
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(CHANNEL_QRE_L,ADC_ATTEN_DB_12);
    ESP_LOGI(TAG,"ADC_WIDTH_BIT_12|\tADC1_CHANNEL_1,ADC_ATTEN_DB_11");
    adc1_config_channel_atten(CHANNEL_QRE_R,ADC_ATTEN_DB_12);
    ESP_LOGI(TAG,"ADC_WIDTH_BIT_12|\tADC1_CHANNEL_2,ADC_ATTEN_DB_11");

    robot_data.qre1113_calibrate = qre1113Calibrate(qreParameterToPass);

    if((qreParameterToPass.qreSensorLRef < 200) || (qreParameterToPass.qreSensorRRef < 200))
    {
        ESP_LOGE(TAG,"QRE1113 INIT FAIL");
    }
    else
    {
        ESP_LOGW(TAG,"QRE1113 INIT OK");
    }
}

void qre1113Task( void * pvParameters )
{
    qreParameterToPass_t *params = (qreParameterToPass_t *)pvParameters;

    uint8_t dir = TB6612::STOP;
    uint64_t refTime = esp_timer_get_time(); 

    for(;;)
    {
        if (robot_data.qre1113_error)
        {
            if ((esp_timer_get_time() - refTime) > 5000000)
            {
                motors.setDirection(TB6612::LEFT);
                led.clear();
                led.setPixel(2,brightness,0,0);
                led.show();
                refTime = esp_timer_get_time(); 
            }
        }
        else
        {
            uint16_t qreSensorL = adc1_get_raw(CHANNEL_QRE_L);
            uint16_t qreSensorR = adc1_get_raw(CHANNEL_QRE_R);
            
            // ESP_LOGI("QRE", "%u - %u", qreSensorL, qreSensorR);
            if (qreSensorL <= params->qreSensorLRef)
            {
                if(dir!=TB6612::RIGHT){vTaskDelay(pdMS_TO_TICKS(100));};
                dir = TB6612::RIGHT;
                motors.setDirection(TB6612::RIGHT);
                led.clear();
                led.setPixel(2,0,0,brightness);
                led.show();
                vTaskDelay(pdMS_TO_TICKS(timeMove));
                motors.setDirection(TB6612::FWD);
            }
            else if (qreSensorR <= params->qreSensorRRef)
            {
                if(dir!=TB6612::LEFT){vTaskDelay(pdMS_TO_TICKS(100));};
                dir = TB6612::LEFT;
                motors.setDirection(TB6612::LEFT);
                led.clear();
                led.setPixel(0,0,0,brightness);
                led.show();
                vTaskDelay(pdMS_TO_TICKS(timeMove));
                motors.setDirection(TB6612::FWD);
            }
        }

        if (adc1_get_raw(ADC1_CHANNEL_0)>2000 || remote_data.start)
        {
            led.clear();
            led.setPixel(0,0,0,0);
            led.setPixel(1,0,0,0);
            led.setPixel(2,0,0,0);
            led.show();
            motors.setDirection(TB6612::STOP);
            esp_restart();
        }
    };
}

void stopTask(void * pvParameter )
{
    for(;;)
    {
        if (adc1_get_raw(ADC1_CHANNEL_0)>2000 || remote_data.start)
        {
            led.clear();
            led.setPixel(0,0,0,0);
            led.setPixel(1,0,0,0);
            led.setPixel(2,0,0,0);
            led.show();
            motors.setDirection(TB6612::STOP);
            esp_restart();
        }
    }
}

void espnowTask(void * pvParameter )
{
    for(;;)
    {
       esp_now_send_data(peer_mac, (uint8_t *) &robot_data, sizeof(robot_data));
    }
}


void strategies(uint8_t strategy, uint8_t waitStart)
{
    /**
     * 0 - modo normal (valido para frontal y para derecha)
     *      - retrocede a la derecha
     * 1 - modo izquierda
     *      - retrocede a la izquierda
     * 2 - modo fallo TOF(si fallan sensores TOF)
     *      - avanza a tirones pequeños
     * 3 - modo fallo QRE
     *      - gira a la izquierda sin avanzar
     * 4 - modo fallo QRE
     *      - gira a la derecha sin avanzar
     */
    switch (strategy)
    {
        case 0:
            if(!waitStart)
            {
                led.clear();
                led.setPixel(0,0,brightness,0);
                led.setPixel(1,0,brightness,0);
                led.show();
            }
            else
            {
                motors.setDirection(TB6612::RIGHT);
                vTaskDelay(pdMS_TO_TICKS(strategyTime));
                motors.setDirection(TB6612::BACK);
                vTaskDelay(pdMS_TO_TICKS(strategyTime));
                motors.setDirection(TB6612::STOP);
            }
            break;
        case 1:
            if(!waitStart)
            {
                led.clear();
                led.setPixel(2,0,brightness,0);
                led.show();
            }
            else
            {
                motors.setDirection(TB6612::LEFT);
                vTaskDelay(pdMS_TO_TICKS(strategyTime));
                motors.setDirection(TB6612::BACK);
                vTaskDelay(pdMS_TO_TICKS(strategyTime));
                motors.setDirection(TB6612::STOP);
            }
            break;
        case 2:
            if(!waitStart)
            {
                led.clear();
                led.setPixel(1,brightness,0,0);
                led.show();
            }
            else
                for(;;)
                {
                    motors.setDirection(TB6612::FWD);
                    vTaskDelay(pdMS_TO_TICKS(strategyTime));
                    motors.setDirection(TB6612::STOP);
                    vTaskDelay(pdMS_TO_TICKS(5000));
                }
            break;
        case 3:
            if(!waitStart)
            {
                led.clear();
                led.setPixel(2,brightness,0,0);
                led.show();
            }
            else
            {
                robot_data.qre1113_error = 1;
                motors.setDirection(TB6612::LEFT);
            }
            break;
        case 4:
            if(!waitStart)
            {
                led.clear();
                led.setPixel(0,brightness,0,0);
                led.show();
            }
            else
            {
                robot_data.qre1113_error = 1;
                motors.setDirection(TB6612::RIGHT);
            }
            break;
        default:
            break;
    }
}

void app_main()
{
    // vTaskDelay(pdMS_TO_TICKS(5000));

    #ifdef REMOTE_CONTROL
        init_esp_now();
        register_peer(peer_mac);
    #endif
    tb6612Init();
    buttonsInit();
    ws2812Init();
    vl53l0xInit();
    qre1113Init(qreParameterToPass);

    uint8_t strategy = 0;
    uint8_t waitStart = 0;
    // uint64_t refTime = 0;
    uint64_t buttonsStatus = 0;

    if(robot_data.vl53l0x_error){strategy = 2;}
    else if(robot_data.qre1113_error){strategy = 3;}
    while(!waitStart)
    {
        // refTime = esp_timer_get_time();
        // while(remote_data.swicth_1) { }
        // if (esp_timer_get_time()-refTime >= 3000000)
        // {
        //     while(remote_data.swicth_1)
        //     {
        //         led.clear();
        //         led.setPixel(0,0,brightness,0);
        //         led.setPixel(1,0,brightness,0);
        //         led.setPixel(2,0,brightness,0);
        //         led.show();
        //     }
        //     waitStart = 1;
        //     break;
        // }
        // else if ((esp_timer_get_time()-refTime > 500000) && (esp_timer_get_time()-refTime < 3000000))
        // {
        //     strategy++;
        //     if (strategy>4)
        //     {
        //         strategy = 0;
        //     }
        //     strategies(strategy, waitStart);
        // }

        buttonsStatus = adc1_get_raw(ADC1_CHANNEL_0);
        if (buttonsStatus > 2000)
        {
            buttonsStatus = 0;
            for (uint8_t i=0; i<200; i++)
            {
                buttonsStatus += adc1_get_raw(ADC1_CHANNEL_0);
            }
            buttonsStatus = buttonsStatus/200;
            if (buttonsStatus>3000)
            {
                while (adc1_get_raw(ADC1_CHANNEL_0)>2000)
                {
                    //Boton 3v3 pulsado
                }
                strategy++;
                if (strategy>4)
                {
                    strategy = 0;
                }
                if (robot_data.vl53l0x_error)
                {
                    if (!remote_data.disable_vl53l0x_error)
                    {
                        strategy = 2;
                        for (uint8_t i=0;i<2;i++)
                        {
                            led.clear();
                            led.setPixel(0, 0, 0, 0);
                            led.show();
                            vTaskDelay(pdMS_TO_TICKS(500));
                            led.clear();
                            led.setPixel(1, brightness, 0, 0);
                            led.show();
                            vTaskDelay(pdMS_TO_TICKS(500));
                        }
                    }
                }
                else if(robot_data.qre1113_error)
                {
                    if (!remote_data.disable_qre1113_error)
                    {
                        if ((strategy<3) || (strategy>4))
                        {
                            strategy = 3;
                        }
                        for (uint8_t i=0;i<2;i++)
                        {
                            led.clear();
                            led.setPixel(0, 0, 0, 0);
                            led.show();
                            vTaskDelay(pdMS_TO_TICKS(200));
                            led.clear();
                            if (strategy == 3)
                            {
                                led.setPixel(2, brightness, 0, 0);
                            }
                            else
                            {
                                led.setPixel(0, brightness, 0, 0);
                            }
                            led.show();
                            vTaskDelay(pdMS_TO_TICKS(200));
                        }
                    }
                }
                
                // vTaskDelay(pdMS_TO_TICKS(500));
            }
            else
            {
                while (adc1_get_raw(ADC1_CHANNEL_0)>2000)
                {
                    //Boton 1v65 pulsado
                    led.clear();
                    led.setPixel(0,0,brightness,0);
                    led.setPixel(1,0,brightness,0);
                    led.setPixel(2,0,brightness,0);
                    led.show();
                }
                waitStart = 1;
                break;
            }
            // ESP_LOGI("STRATEGY","%u %llu",strategy, buttonsStatus);
        }

        robot_data.strategy = strategy;

        esp_now_send_data(peer_mac, (uint8_t *) &robot_data, sizeof(robot_data));
        if(remote_data.status)
        {
            strategy = remote_data.strategy;
            waitStart = remote_data.start;
            if (waitStart){break;}
        }

        if (remote_data.calibrate_qre1113)
        {
            robot_data.qre1113_calibrate = qre1113Calibrate(qreParameterToPass);
        }
        
        // ESP_LOGE("STRATEGY","%u\t%u",strategy, robot_data.strategy);
        strategies(strategy, waitStart);
    };

    // blink leds (timeToStart) ms.
    for (uint8_t i=0; i<10; i++)
    {
        led.clear();
        if (robot_data.vl53l0x_error || robot_data.qre1113_error)
        {
            led.setPixel(0,brightness,0,0);
            led.setPixel(1,brightness,0,0);
            led.setPixel(2,brightness,0,0);
        }
        else
        {
            led.setPixel(0,0,brightness,0);
            led.setPixel(1,0,brightness,0);
            led.setPixel(2,0,brightness,0);
        }
        led.show();
        vTaskDelay(pdMS_TO_TICKS(timeToStart/20));
        led.clear();
        led.setPixel(0,0,0,0);
        led.setPixel(1,0,0,0);
        led.setPixel(2,0,0,0);
        led.show();
        vTaskDelay(pdMS_TO_TICKS(timeToStart/20));
    }

    strategies(strategy, waitStart);

    static uint8_t ucParameterToPass;
    if (!robot_data.vl53l0x_error)
    {
        TaskHandle_t vl53l0xTaskHandle = NULL;
        xTaskCreate( vl53l0xTask, "vl53l0xTask", 1024*2, &ucParameterToPass, 24, &vl53l0xTaskHandle ); 
    }
    if (!robot_data.qre1113_error)
    {
        TaskHandle_t qre1113TaskHandle = NULL;
        xTaskCreate( qre1113Task, "qre1113Task", 1024*2, &qreParameterToPass, 23, &qre1113TaskHandle ); 
    }
    TaskHandle_t stopTaskHandle = NULL;
    xTaskCreate( stopTask, "stopTask", 1024*2, &ucParameterToPass, 10, &stopTaskHandle ); 
    TaskHandle_t espnowTaskHandle = NULL;
    xTaskCreate( espnowTask, "espnowTask", 1024*2, &ucParameterToPass, 9, &espnowTaskHandle ); 

}