#include <stdio.h>
#include <string.h>
#include <math.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wrapper/include/vl53l1x.h"


#define MSP_RX_PIN      (GPIO_NUM_19)
#define MSP_TX_PIN      (GPIO_NUM_18)
#define MSP_UART_NUM    (UART_NUM_2)
#define MSP_BAUD_RATE   (230400)
#define RX_BUF_SIZE     (1024)
#define MAX_CHANNELS    16 

#define MIN_DISTACE_MM  200
#define MAX_DISTANCE_MM 2000
#define MAX_PWM_DIFF 500
#define MIN_PWM_DIFF 0

#define sdaPin 21
#define sclPin 22

#define xShutA 16
#define xShutB 17
#define xShutC 5

enum sensor_pos {
    FRONT=0,
    LEFT,
    RIGHT
};

enum attitude_index {
    ROLL=0,
    PITCH,
    YAW
};

int8_t xShut_arr[3] = {17, 16, 4};
int8_t adress_arr[3] = {0x30, 0x31, 0x32};
vl53l1x_device_handle_t device_arr[3] = {VL53L1X_DEVICE_INIT, VL53L1X_DEVICE_INIT, VL53L1X_DEVICE_INIT};
volatile uint16_t distance[3] = {0, 0, 0};

static const char *TAG = "VL53L1X";

static vl53l1x_i2c_handle_t i2cHandle = VL53L1X_I2C_INIT;
static vl53l1x_handle_t vl53 = VL53L1X_INIT;

volatile int16_t attitude[3] = {0,0,0}; // roll, pitch, yaw


#define APF_DETECT_DIST_CM  450.0f  // Force starts here (2 meters)
#define APF_MIN_DIST_CM     50.0f   // Max force reached here (20 cm)
#define APF_MAX_MODIFIER    500     // Max PWM change (+/- 500)
#define APF_GAIN            15000.0f // Tuning knob (Higher = stronger reaction)

// Calculate Repulsive Force based on distance
// Returns: 0 to APF_MAX_MODIFIER
int16_t calculate_apf_force(float distance_cm) {
    // 1. If we are far away, no force
    if (distance_cm >= APF_DETECT_DIST_CM || distance_cm <= 0) {
        return 0;
    }

    // 2. Safety: Don't divide by zero or negative numbers
    if (distance_cm < 1.0f) distance_cm = 1.0f;

    // 3. The Formula: F = Gain * (1/dist - 1/limit)
    // This creates a curve: gentle at 200cm, extreme at 20cm
    float force = APF_GAIN * ( (1.0f / distance_cm) - (1.0f / APF_DETECT_DIST_CM) );

    // 4. Convert to Integer
    int16_t pwm_adjust = (int16_t)force;

    // 5. Clamp the result
    if (pwm_adjust > APF_MAX_MODIFIER) pwm_adjust = APF_MAX_MODIFIER;
    if (pwm_adjust < 0) pwm_adjust = 0;

    return pwm_adjust;
}



int map_range(int value, int in_min, int in_max, int out_min, int out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void gpio_init(gpio_num_t pin)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

int init_sensors()
{   
    int err_count = 0;
    ESP_LOGI(TAG,"Init sensors...");

    for(int i = 0; i < 3; i++)
    {
        gpio_init(xShut_arr[i]);
        gpio_set_level(xShut_arr[i],0);
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    i2cHandle.scl_gpio = sclPin;
    i2cHandle.sda_gpio = sdaPin;
    
    vl53.i2c_handle = &i2cHandle;
    
    if(!vl53l1x_init(&vl53))
    {
        ESP_LOGE(TAG,"Failed init VL53 core");
        return 0;
    }

    for(int i = 0; i < 3; i++)
    {
        gpio_set_level(xShut_arr[i],1);
        vTaskDelay(pdMS_TO_TICKS(20));

        device_arr[i].vl53l1x_handle = &vl53;
        device_arr[i].xshut_gpio = xShut_arr[i];

        if(!vl53l1x_add_device(&device_arr[i]))
        {
            ESP_LOGE(TAG,"Add Sensor %d failed", i);
            err_count++;
            continue;
        }

        vl53l1x_update_device_address(&device_arr[i], adress_arr[i]);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI(TAG,"Done init sensors");
    return err_count;
}



// --- CHANNEL MAPPING ---
// Array Indices (0-15) corresponding to Channels (1-16)
enum rc_map {
    // --- OUTPUTS (Controlled by ESP32) ---
    // Mask 127 covers indices 0 to 6
    OUT_ROLL=0, OUT_PITCH, OUT_YAW, OUT_THR,  // Ch 1-4
    OUT_AUX1,   OUT_AUX2,  OUT_AUX3,          // Ch 5-7 (Virtuals)
    
    // --- PASSTHROUGH (Controlled by Radio) ---
    // Mask skips index 7 (Ch 8). We write it, but FC ignores us.
    PASS_ARM,                                 // Ch 8
    
    // --- INPUTS (Read Only) ---
    IN_ROLL_MIRROR,    // Ch 9
    IN_PITCH_MIRROR,   // Ch 10
    IN_YAW_MIRROR,     // Ch 11
    IN_THR_MIRROR,     // Ch 12
    IN_PAGE_SW,        // Ch 13 (3-Pos Switch)
    IN_VAL_A,          // Ch 14 (Pot/Slider)
    IN_VAL_B,          // Ch 15 (Switch)
    IN_VAL_C           // Ch 16 (Switch)
};

#define MSP_CMD_RC          105 
#define MSP_CMD_SET_RAW_RC  200
#define MSP_CMD_ATTITUDE    108 

uint16_t global_rc[MAX_CHANNELS]; 
SemaphoreHandle_t xDroneStateMutex = NULL;

// --- UART ---
void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = MSP_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };


    uart_driver_install(MSP_UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(MSP_UART_NUM, &uart_config);
    uart_set_pin(MSP_UART_NUM, MSP_TX_PIN, MSP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void msp_send_packet(uint8_t cmd_id, void *payload, uint8_t size) {
    uint8_t header[6] = {'$', 'M', '<', size, cmd_id, 0};
    uint8_t checksum = size ^ cmd_id;
    uart_write_bytes(MSP_UART_NUM, (const char *)header, 5);
    uint8_t *payload_ptr = (uint8_t *)payload;
    if (size > 0 && payload != NULL) {
        for (int i = 0; i < size; i++) checksum ^= payload_ptr[i];
        uart_write_bytes(MSP_UART_NUM, (const char *)payload, size);
    }
    uart_write_bytes(MSP_UART_NUM, (const char *)&checksum, 1);
}

// --- PARSER ---
void process_msp_byte(uint8_t byte) {
    static int state = 0; 
    static uint8_t msg_len = 0;
    static uint8_t msg_cmd = 0;
    static uint8_t rx_buf[64];
    static uint8_t rx_idx = 0;
    static uint8_t calc_crc = 0;

    switch (state) {
        case 0: if (byte == '$') state = 1; break;
        case 1: if (byte == 'M') state = 2; else state = 0; break;
        case 2: if (byte == '>') state = 3; else state = 0; break;
        case 3: msg_len = byte; calc_crc = byte; state = 4; break;
        case 4: msg_cmd = byte; calc_crc ^= byte; rx_idx = 0; state = (msg_len > 0) ? 5 : 6; break;
        case 5: rx_buf[rx_idx++] = byte; calc_crc ^= byte; if (rx_idx == msg_len) state = 6; break;
        case 6: 
            if (calc_crc == byte && msg_cmd == MSP_CMD_RC) {
                if (xSemaphoreTake(xDroneStateMutex, portMAX_DELAY)) {
                    int num = msg_len / 2;
                    if (num > MAX_CHANNELS) num = MAX_CHANNELS;
                    for (int i = 0; i < num; i++) global_rc[i] = rx_buf[i*2] | (rx_buf[i*2+1] << 8);
                    xSemaphoreGive(xDroneStateMutex);
                }
            }
            else if(calc_crc == byte && msg_cmd == MSP_CMD_ATTITUDE){
                if (xSemaphoreTake(xDroneStateMutex, portMAX_DELAY)) {
                    if (msg_len >= 6) {
                        attitude[ROLL]  = (rx_buf[0] | (rx_buf[1] << 8));
                        attitude[PITCH] = (rx_buf[2] | (rx_buf[3] << 8));
                        attitude[YAW]   = (rx_buf[4] | (rx_buf[5] << 8));
                    }
                    else {
                        ESP_LOGE(TAG, "Attitude payload too short");
                    }
                    xSemaphoreGive(xDroneStateMutex);
                }
                else {
                    ESP_LOGE(TAG, "Failed to take mutex for attitude");
                }
            }
            state = 0; break;
        default: state = 0; break;
    }
}

void rx_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(RX_BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(MSP_UART_NUM, data, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
        if (len > 0) for (int i = 0; i < len; i++) process_msp_byte(data[i]);
    }
    free(data);
}


int get_page_number(uint16_t pwm) {
    if (pwm < 1300) return 0; // Low
    if (pwm > 1700) return 2; // High
    return 1;                 // Mid
}

void control_task(void *pvParameters) {
    uint16_t input_rc[MAX_CHANNELS];
    static uint16_t output_rc[MAX_CHANNELS]; 

    // Memory: 3 Pages, 3 Inputs
    static uint16_t memory_bank[3][3]; 

    // Init Defaults
    for(int i=0; i<MAX_CHANNELS; i++) output_rc[i] = 1500;
    output_rc[OUT_THR] = 1000;
    
    // Init Memory
    for(int p=0; p<3; p++) {
        for(int i=0; i<3; i++) memory_bank[p][i] = 1500;
    }

    while (1) {

        // 1. Request Data (Returns Physical Ch 8-16 + Echo Ch 1-7)
        msp_send_packet(MSP_CMD_RC, NULL, 0);

        // 2. Read Snapshot
        if (xSemaphoreTake(xDroneStateMutex, portMAX_DELAY)) {
            memcpy(input_rc, global_rc, sizeof(global_rc));
            xSemaphoreGive(xDroneStateMutex);
        }

        // --- 3. STICK LOGIC ---
        uint16_t force_from_left = calculate_apf_force(distance[LEFT]);
        uint16_t force_from_right = calculate_apf_force(distance[RIGHT]);


        // output_rc[OUT_ROLL]  = input_rc[IN_ROLL_MIRROR] + force_from_left - force_from_right;
        output_rc[OUT_ROLL]  = input_rc[IN_ROLL_MIRROR];
        output_rc[OUT_PITCH] = input_rc[IN_PITCH_MIRROR];
        output_rc[OUT_YAW]   = input_rc[IN_YAW_MIRROR];
        output_rc[OUT_THR]   = input_rc[IN_THR_MIRROR];

        // --- 4. PAGE LOGIC (Virtual Channels) ---
        int page = get_page_number(input_rc[IN_PAGE_SW]);

        // Save inputs to current page memory
        memory_bank[page][0] = input_rc[IN_VAL_A]; 
        memory_bank[page][1] = input_rc[IN_VAL_B]; 
        memory_bank[page][2] = input_rc[IN_VAL_C]; 

        // Map Page 0 -> AUX 1 (Ch 5)
        output_rc[OUT_AUX1] = memory_bank[0][0]; 
        
        // Map Page 1 -> AUX 2 (Ch 6)
        output_rc[OUT_AUX2] = memory_bank[1][0]; 
        
        // Map Page 2 -> AUX 3 (Ch 7)
        output_rc[OUT_AUX3] = memory_bank[2][0]; 

        output_rc[PASS_ARM] = 1000; 

        uint8_t payload[32]; 
        for (int i = 0; i < 16; i++) {
            payload[i*2]     = output_rc[i] & 0xFF;
            payload[i*2 + 1] = output_rc[i] >> 8;
        }
        msp_send_packet(MSP_CMD_SET_RAW_RC, payload, 32);

        msp_send_packet(MSP_CMD_ATTITUDE, NULL, 0);


    }
}


void sensor_task(void *pvParameters) {
    init_sensors();

    while(1)
    {
        for(int i = 0; i < 2; i++)
        {
            distance[i] = vl53l1x_get_mm(&device_arr[i]);
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }

    }
}


void log_task(){
    while(1){
        if (xSemaphoreTake(xDroneStateMutex, portMAX_DELAY)) {
            ESP_LOGI("ATT", "Roll: %d | Pitch: %d | Yaw: %d", attitude[ROLL] / 10, attitude[PITCH] / 10, attitude[YAW] / 10);
            ESP_LOGI(TAG,"A=%d mm | B=%d mm | C=%d mm", distance[FRONT], distance[LEFT], distance[RIGHT]);
            xSemaphoreGive(xDroneStateMutex);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    init_uart();
    xDroneStateMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(sensor_task, "SENSOR", 4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(rx_task, "RX", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(control_task, "CTRL", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(log_task, "LOG", 4096, NULL, 3, NULL, 1);
}