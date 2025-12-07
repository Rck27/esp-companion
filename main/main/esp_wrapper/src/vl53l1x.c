#include "esp_wrapper/include/vl53l1x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "core/include/VL53L1X_api.h"
#include "core/include/VL53L1X_calibration.h"
#include "esp_log.h"

#include "esp_wrapper/include/i2c_device_handler.h"

static const char *TAG = "VL53L1X";

void wait_boot(uint16_t dev);
bool vl53l1x_init(vl53l1x_handle_t *vl53l1x_handle)
{
    g_vl53l1x_write_multi_ptr = i2c_write_multi;
    g_vl53l1x_read_multi_ptr = i2c_read_multi;

    g_vl53l1x_write_byte_ptr = i2c_write_byte;
    g_vl53l1x_write_word_ptr = i2c_write_word;
    g_vl53l1x_write_dword_ptr = i2c_write_dword;
    g_vl53l1x_read_byte_ptr = i2c_read_byte;
    g_vl53l1x_read_word_ptr = i2c_read_word;
    g_vl53l1x_read_dword_ptr = i2c_read_dword;

    // check if i2c is initialized
    if (!i2c_master_init(vl53l1x_handle->i2c_handle))
    {
        return false;
    }
    
    vl53l1x_handle->initialized = true;
    return true;
}

bool vl53l1x_add_device(vl53l1x_device_handle_t *device)
{
    if (!device->vl53l1x_handle->initialized)
    {
        ESP_LOGE(TAG, "attempted to add device while vl53l1x not initialized, call vl53l1x_init first");
        return false;
    }

    if (!i2c_add_device(device))
    {
        return false;
    }
    ESP_LOGI(TAG, "device 0x%04X created", device->dev);

    wait_boot(device->dev);
    ESP_LOGI(TAG, "device booted");

    if (VL53L1X_SensorInit(device->dev) != 0)
    {
        ESP_LOGE(TAG, "device failled initialization");
    }

    // configuration
    VL53L1X_SetDistanceMode(device->dev, LONG);

    // calibraton
    VL53L1X_StartTemperatureUpdate(device->dev);
    // VL53L1X_CalibrateOffset(device->dev, 100, NULL);
    // VL53L1X_CalibrateXtalk(device->dev, 100, NULL);

    ESP_LOGI(TAG, "device ready");
    VL53L1X_StartRanging(device->dev);

    // log stuff after initialization success
    vl53l1x_log_sensor_id(device);
    vl53l1x_log_ambient_light(device);
    vl53l1x_log_signal_rate(device);
    return true;
}

uint16_t vl53l1x_get_mm(const vl53l1x_device_handle_t *device)
{
    uint16_t distance = 0;
    VL53L1X_GetDistance(device->dev, &distance);
    VL53L1X_ClearInterrupt(device->dev);
    return distance;
}

bool vl53l1x_update_device_address(vl53l1x_device_handle_t *device, uint8_t new_address)
{
    if (!i2c_update_address(device->dev, new_address))
    {
        return false;
    }

    ESP_LOGI(TAG, "device address updated: 0x%02X->0x%02X", device->i2c_address, new_address);
    device->dev = create_dev(get_port(device->dev), new_address);

    return true;
}

void vl53l1x_log_sensor_id(const vl53l1x_device_handle_t *device)
{
    uint16_t id = 0;
    VL53L1X_GetSensorId(device->dev, &id);

    ESP_LOGI(TAG, "Model ID: 0x%02X", id); // VL53L1X = 0xEEAC
}

void vl53l1x_log_ambient_light(const vl53l1x_device_handle_t *device)
{
    uint16_t ambRate;
    VL53L1X_GetAmbientRate(device->dev, &ambRate);

    ESP_LOGI(TAG, "Ambient rate: %dkcps", ambRate);
}

void vl53l1x_log_signal_rate(const vl53l1x_device_handle_t *device)
{
    uint16_t signal;
    VL53L1X_GetSignalRate(device->dev, &signal);
    ESP_LOGI(TAG, "Signal rate: %dkcps", signal);
}

void wait_boot(uint16_t dev)
{
    uint8_t boot_state = 0;
    while (boot_state == 0)
    {
        VL53L1X_BootState(dev, &boot_state);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}