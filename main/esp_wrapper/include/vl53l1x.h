#ifndef VL53L1X_H
#define VL53L1X_H

#include "driver/gpio.h"
#include "i2c_handler.h"
#include "vl53l1x_types.h"

typedef enum
{
    SHORT = 1, // <1.3m
    LONG = 2,  // <4m
} distance_mode;

bool vl53l1x_init(vl53l1x_handle_t *vl53l1x_handle);
bool vl53l1x_add_device(vl53l1x_device_handle_t *device);
bool vl53l1x_update_device_address(vl53l1x_device_handle_t *device, uint8_t new_address);

uint16_t vl53l1x_get_mm(const vl53l1x_device_handle_t *device);

void vl53l1x_log_sensor_id(const vl53l1x_device_handle_t *device);
void vl53l1x_log_ambient_light(const vl53l1x_device_handle_t *device);
void vl53l1x_log_signal_rate(const vl53l1x_device_handle_t *device);

#endif