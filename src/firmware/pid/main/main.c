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

// 1. The Blueprint for the 12-byte payload
// __attribute__((packed)) does exactly what #pragma pack(push, 1) did in Ubuntu
typedef struct __attribute__((packed)) {
    float pitch;
    float roll;
    float yaw;
} ImuPayload;

// 2. The States
typedef enum {
    WAIT_FOR_AA,
    WAIT_FOR_BB,
    READ_PAYLOAD
} SerialState;

// 3. The FreeRTOS Task
void uart_rx_task(void *arg) 
{
    uint8_t raw_bytes[BUF_SIZE];
    SerialState current_state = WAIT_FOR_AA;
    
    uint8_t payload[12];
    int payload_index = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, raw_bytes, BUF_SIZE, pdMS_TO_TICKS(10));

        for (int i = 0; i < len; i++) {
            uint8_t byte = raw_bytes[i];

            switch (current_state) {
                case WAIT_FOR_AA:
                    if (byte == 0xAA) current_state = WAIT_FOR_BB;
                    break;

                case WAIT_FOR_BB:
                    if (byte == 0xBB) {
                        current_state = READ_PAYLOAD;
                        payload_index = 0;
                    } else if (byte == 0xAA) {
                        current_state = WAIT_FOR_BB;
                    } else {
                        current_state = WAIT_FOR_AA;
                    }
                    break;

                case READ_PAYLOAD:
                    payload[payload_index] = byte;
                    payload_index++;

                    if (payload_index == 12) {
                        
                        // --- THE RECONSTRUCTION ---
                        // Put on the "ImuPayload" glasses and look at the 12-byte array
                        ImuPayload* imu_data = (ImuPayload*)payload;
                        
                        // Print the pure floats to the ESP32 terminal!
                        ESP_LOGI(TAG, "Pitch: %.2f | Roll: %.2f | Yaw: %.2f", 
                                 imu_data->pitch, imu_data->roll, imu_data->yaw);
                        
                        current_state = WAIT_FOR_AA;
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

    // 4. Launch the FreeRTOS task!
    // This tells the operating system to start running your infinite loop in the background.
    xTaskCreatePinnedToCore(
        uart_rx_task,       // The function to run
        "uart_rx_task",     // A human-readable name for debugging
        4096,               // Stack size (How much RAM to give this task)
        NULL,               // The "void *arg" we discussed (We pass nothing)
        10,                 // Priority (10 is high, meaning this task is important)
        NULL,               // Task Handle (Not needed)
        0                   // Pin this exclusively to Core 0
    );
}