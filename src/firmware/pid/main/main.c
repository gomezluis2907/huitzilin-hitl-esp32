#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT_NUM      UART_NUM_2
#define UART_TX_PIN        GPIO_NUM_17
#define UART_RX_PIN        GPIO_NUM_16
#define UART_BAUD_RATE     115200       
#define BUF_SIZE           256          

static const char *TAG = "HUITZILIN_UART";

// Payload blueprint
typedef struct __attribute__((packed)) {
    float pitch;
    float roll;
    float yaw;
} ImuPayload;

// Payload blueprint
typedef struct __attribute__((packed)){
    float pitch;
    float roll;
    float yaw;
    float throttle;
} KeysPayload;

// States
typedef enum {
    WAIT_FOR_HEADER1,
    WAIT_FOR_BB,
    WAIT_FOR_DD,
    READ_IMU_PAYLOAD,
    READ_KEYS_PAYLOAD
} SerialState;

// FreeRTOS Task
void uart_rx_task(void *pvParameters) 
{
    uint8_t raw_bytes[BUF_SIZE];
    SerialState current_state = WAIT_FOR_HEADER1;
    
    uint8_t payload[16];
    int payload_index = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, raw_bytes, BUF_SIZE, pdMS_TO_TICKS(10));

        for (int i = 0; i < len; i++) {
            uint8_t byte = raw_bytes[i];

            switch (current_state) {
                
                case WAIT_FOR_HEADER1:
                    if (byte == 0xAA) current_state = WAIT_FOR_BB;
                    else if (byte == 0xCC) current_state = WAIT_FOR_DD;
                    break;

                case WAIT_FOR_BB:
                    if (byte == 0xBB) {
                        current_state = READ_IMU_PAYLOAD;
                        payload_index = 0;
                    } else if (byte == 0xAA) {
                        current_state = WAIT_FOR_BB;
                    } else if (byte == 0xCC) {
                        current_state = WAIT_FOR_DD;
                    }else {
                        current_state = WAIT_FOR_HEADER1;
                    }
                    break;
                
                case WAIT_FOR_DD:
                    if (byte == 0xDD) {
                        current_state = READ_KEYS_PAYLOAD;
                        payload_index = 0;
                    } else if (byte == 0xAA) {
                        current_state = WAIT_FOR_BB;
                    } else if (byte == 0xCC) {
                        current_state = WAIT_FOR_DD;
                    } else {
                        current_state = WAIT_FOR_HEADER1;
                    }
                    break;
                
                case READ_IMU_PAYLOAD:
                    payload[payload_index] = byte;
                    payload_index++;

                    if (payload_index == 12) {

                        ImuPayload* imu_data = (ImuPayload*)payload;

                        // Print 
                        ESP_LOGI(TAG, "Pitch: %.2f | Roll: %.2f | Yaw: %.2f", 
                                 imu_data->pitch, imu_data->roll, imu_data->yaw);
                        
                        current_state = WAIT_FOR_HEADER1;

                    }
                    break;

                case READ_KEYS_PAYLOAD:
                    payload[payload_index] = byte;
                    payload_index++;

                    if (payload_index == 16) {

                        KeysPayload* keys_data = (KeysPayload*)payload;

                        // Print
                        ESP_LOGI(TAG, "Pitch: %.2f | Roll: %.2f | Yaw: %.2f | Throttle: %.2f", 
                                 keys_data->pitch, keys_data->roll, keys_data->yaw, keys_data->throttle);
                        
                        current_state = WAIT_FOR_HEADER1;
                    }
                    break;

            }
        }
    }
}

void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // 512 bytes allocation for RX
    uart_driver_install(UART_PORT_NUM, BUF_SIZE*2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Launch FreeRTOS task
    xTaskCreatePinnedToCore(
        uart_rx_task,       // Function
        "uart_rx_task",     // Name for debugging
        4096,               // Stack size 
        NULL,               
        10,                 // 10 priority
        NULL,               
        0                   // Pin to Core 0
    );
}