#include <stdio.h>
#include <string.h>
#include <math.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

// --- CONFIG ---
#define MSP_RX_PIN      (GPIO_NUM_18)
#define MSP_TX_PIN      (GPIO_NUM_19)
#define MSP_UART_NUM    (UART_NUM_2)
#define MSP_BAUD_RATE   (115200)
#define RX_BUF_SIZE     (1024)
#define MAX_CHANNELS    16 

static const char *TAG = "MSP_PAGE";

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

    int loop_counter = 0;

    while (1) {
        loop_counter++;

        // 1. Request Data (Returns Physical Ch 8-16 + Echo Ch 1-7)
        msp_send_packet(MSP_CMD_RC, NULL, 0);
        vTaskDelay(15 / portTICK_PERIOD_MS); 

        // 2. Read Snapshot
        if (xSemaphoreTake(xDroneStateMutex, portMAX_DELAY)) {
            memcpy(input_rc, global_rc, sizeof(global_rc));
            xSemaphoreGive(xDroneStateMutex);
        }

        // --- 3. STICK LOGIC ---
        int sensor_offset = (int)(sin(loop_counter * 0.1) * 200);
        
        output_rc[OUT_ROLL]  = input_rc[IN_ROLL_MIRROR] + sensor_offset;
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

        // Debug
        if (loop_counter % 50 == 0) {
             ESP_LOGI(TAG, "Page:%d | Aux1:%d | Aux2:%d | Aux3:%d", 
                page, output_rc[OUT_AUX1], output_rc[OUT_AUX2], output_rc[OUT_AUX3]);
        }

        vTaskDelay(20 / portTICK_PERIOD_MS); 
    }
}

void app_main(void) {
    init_uart();
    xDroneStateMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(rx_task, "RX", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(control_task, "CTRL", 4096, NULL, 4, NULL, 1);
}