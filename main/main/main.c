#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wrapper/include/vl53l1x.h"

#define sdaPin 21
#define sclPin 22

#define xShutA 16
#define xShutB 17
#define xShutC 5

#define addrA 0x30
#define addrB 0x31
#define addrC 0x32

static const char *TAG = "VL53L1X";

static vl53l1x_i2c_handle_t i2cHandle = VL53L1X_I2C_INIT;
static vl53l1x_handle_t vl53 = VL53L1X_INIT;

static vl53l1x_device_handle_t SensorA = VL53L1X_DEVICE_INIT;
static vl53l1x_device_handle_t SensorB = VL53L1X_DEVICE_INIT;
static vl53l1x_device_handle_t SensorC = VL53L1X_DEVICE_INIT;


static void gpio_init(gpio_num_t pin)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

void init_sensors()
{
    ESP_LOGI(TAG,"Init sensors...");

    gpio_init(xShutA);
    gpio_init(xShutB);
    gpio_init(xShutC);

    gpio_set_level(xShutA,0);
    gpio_set_level(xShutB,0);
    gpio_set_level(xShutC,0);
    vTaskDelay(pdMS_TO_TICKS(50));

    i2cHandle.scl_gpio = sclPin;
    i2cHandle.sda_gpio = sdaPin;

    vl53.i2c_handle = &i2cHandle;
    if(!vl53l1x_init(&vl53))
    {
        ESP_LOGE(TAG,"Failed init VL53 core");
        return;
    }

    //Init SensorA
    gpio_set_level(xShutA,1);
    vTaskDelay(pdMS_TO_TICKS(20));

    SensorA.vl53l1x_handle = &vl53;
    SensorA.xshut_gpio = xShutA;

    if(!vl53l1x_add_device(&SensorA))
    {
        ESP_LOGE(TAG,"Add SensorA failed");
        return;
    }

    vl53l1x_update_device_address(&SensorA, addrA);
    vTaskDelay(pdMS_TO_TICKS(20));


    //Init SensorB
    gpio_set_level(xShutB,1);
    vTaskDelay(pdMS_TO_TICKS(20));

    SensorB.vl53l1x_handle = &vl53;
    SensorB.xshut_gpio = xShutB;

    if(!vl53l1x_add_device(&SensorB))
    {
        ESP_LOGE(TAG,"Add SensorB failed");
        return;
    }

    vl53l1x_update_device_address(&SensorB, addrB);
    vTaskDelay(pdMS_TO_TICKS(20));

    //Init SensorC
    gpio_set_level(xShutC,1);
    vTaskDelay(pdMS_TO_TICKS(20));

    SensorC.vl53l1x_handle = &vl53;
    SensorC.xshut_gpio = xShutC;

    if(!vl53l1x_add_device(&SensorC))
    {
        ESP_LOGE(TAG,"Add SensorC failed");
        return;
    }

    vl53l1x_update_device_address(&SensorC, addrC);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG,"Done init sensors");
}


void app_main(void)
{
    init_sensors();

    while(1)
    {
        uint16_t sA = vl53l1x_get_mm(&SensorA);
        uint16_t sB = vl53l1x_get_mm(&SensorB);
        uint16_t sC = vl53l1x_get_mm(&SensorC);

        ESP_LOGI(TAG,"A=%d mm | B=%d mm | C=%d mm", sA, sB, sC);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
