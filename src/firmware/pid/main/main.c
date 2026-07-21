#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/semphr.h"

// UART definitions
#define UART_PORT_NUM      UART_NUM_2
#define UART_TX_PIN        GPIO_NUM_17
#define UART_RX_PIN        GPIO_NUM_16
#define UART_BAUD_RATE     115200       
#define BUF_SIZE           256          

// Tuning constants
#define KP 1.0f
#define KI 0.0f
#define KD 0.0f
#define DT 0.01f

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

// Global instances
ImuPayload global_imu;
KeysPayload global_keys;

// Semaphore
SemaphoreHandle_t xMutex;

// FreeRTOS UART task
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
                        
                        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {

                            global_imu.pitch = imu_data->pitch;
                            global_imu.roll = imu_data->roll;
                            global_imu.yaw = imu_data->yaw;

                            // Give key
                            xSemaphoreGive(xMutex);
                        }
                        
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
                        
                        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {

                            global_keys.pitch = keys_data->pitch;
                            global_keys.roll = keys_data->roll;
                            global_keys.yaw = keys_data->yaw;
                            global_keys.throttle = keys_data->throttle;
                            
                            // Give key
                            xSemaphoreGive(xMutex);


                        }
                        
                        
                        current_state = WAIT_FOR_HEADER1;
                    }
                    break;

            }
        }
    }
}

// FreeRTOS PID task
void pid_task(void *pvParameters)
{
    // Integral and derivative history
    float prev_error_pitch = 0, integral_pitch = 0;
    float prev_error_roll = 0, integral_roll = 0;
    float prev_error_yaw = 0, integral_yaw = 0;


    while (1) {

        // Local copies
        ImuPayload local_imu;
        KeysPayload local_keys;

        // Mutex
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {

            local_imu = global_imu;
            local_keys = global_keys;
            xSemaphoreGive(xMutex);
        }

        // Current errors
        float error_pitch = local_keys.pitch - local_imu.pitch;
        float error_roll = local_keys.roll - local_imu.roll;
        float error_yaw = local_keys.yaw - local_imu.yaw;

        // Proportional
        float p_pitch = KP * error_pitch;
        float p_roll = KP * error_roll;
        float p_yaw = KP * error_yaw;

        // Integral
        // Sum terms
        integral_pitch += integral_pitch + error_pitch * DT;
        integral_roll += integral_roll + error_roll * DT;
        integral_yaw += integral_yaw + error_yaw * DT;

        float i_pitch = KI * integral_pitch;
        float i_roll = KI * integral_roll;
        float i_yaw = KI * integral_yaw;

        // Derivative
        float d_pitch = KD * (error_pitch - prev_error_pitch)/DT;
        float d_roll = KD * (error_roll - prev_error_roll)/DT;
        float d_yaw = KD * (error_yaw - prev_error_yaw)/DT;

        //PID
        float out_pitch = p_pitch + i_pitch + d_pitch;
        float out_roll = p_roll + i_roll + d_roll;
        float out_yaw = p_yaw + i_yaw + d_yaw;

        // Previous errors for the next loop
        prev_error_pitch = error_pitch;
        prev_error_roll = error_roll;
        prev_error_yaw = error_yaw;

        vTaskDelay(pdMS_TO_TICKS(10));

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

    // Mutex
    xMutex = xSemaphoreCreateMutex();

    // Launch FreeRTOS tasks
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