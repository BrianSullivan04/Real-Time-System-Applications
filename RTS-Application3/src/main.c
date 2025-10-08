/* --------------------------------------------------------------
   Application: 03 - Rev1
   Release Type: Complete code
   Class: Real Time Systems - Fall 2025
   Author: Brian Sullivan
   Theme: Space Systems Simulation
---------------------------------------------------------------*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "math.h"
#include "esp_log.h"

#define LED_PIN GPIO_NUM_2       // Beacon LED
#define BUTTON_GPIO GPIO_NUM_4   // Button Config
#define LDR_PIN GPIO_NUM_32      // Simulated solar sensor pin
#define LDR_ADC_CHANNEL ADC1_CHANNEL_4

//Define average window size globally
#define LOG_SIZE 50
static const char *TAG = "App03";

// Shared buffer and index
static uint16_t lux_log[LOG_SIZE];
static int log_index = 0;
static unsigned long total_readings = 0;   // Total readings taken
static int wrapped = 0;                    // Track how many times buffer has been wrapped

// Synchronization
static SemaphoreHandle_t xLogMutex;  // Hand off access to the buffer!
static SemaphoreHandle_t xButtonSem;  // Hand off on button press!

// === ISR: Triggered on button press ===
// === ISR: Triggered on button press with debouncing ===
void IRAM_ATTR button_isr_handler(void *arg) {
    static uint32_t last_press_time = 0;  // in ticks
    uint32_t current_time = xTaskGetTickCountFromISR();

    // Debounce: ignore presses within 200ms
    const uint32_t debounce_ticks = pdMS_TO_TICKS(200);

    if ((current_time - last_press_time) > debounce_ticks) {
        BaseType_t xHigherPrioTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xButtonSem, &xHigherPrioTaskWoken);
        portYIELD_FROM_ISR(xHigherPrioTaskWoken);
        last_press_time = current_time;
    }
}

// Task: Beacon Task 
void beacon_task(void *pvParameters) {
    bool led_on = false;
    TickType_t currentTime = pdTICKS_TO_MS(xTaskGetTickCount()); //Get current time of running code
    TickType_t previousTime = 0; //Initialize previous time to 0

    while (1) {
        //Handle timing
        previousTime = currentTime; //Update previous time
        currentTime = pdTICKS_TO_MS(xTaskGetTickCount()); //Update current time

        gpio_set_level(LED_PIN, led_on);
        led_on = !led_on; // toggle state for next time

        //Print message to indicate LED is blinking as intended (turned off)
        //printf("\n[Telemetry] Beacon %s | Timestamp: %lums | Period = %lu ms\n", led_on ? "ON" : "OFF", currentTime, period);

        vTaskDelay(pdMS_TO_TICKS(1400)); // Delay for 500 ms using MS to Ticks Function vs alternative which is MS / ticks per ms
    }
    vTaskDelete(NULL); // We'll never get here; tasks run forever
}

// Task: Telemetry Task
void telemetry_task(void *pvParameters) {
    TickType_t currentTime = pdTICKS_TO_MS(xTaskGetTickCount()); // Get current time of running code
    TickType_t previousTime = 0; // Initialize previous time to 0
    while (1) {
        previousTime = currentTime; // Update previous time
        currentTime = pdTICKS_TO_MS(xTaskGetTickCount()); // Update current time
        TickType_t period = (currentTime - previousTime); // Calculate period

        uint16_t latest_lux = 0;

        // --- Safely read latest lux using mutex ---
        if (xSemaphoreTake(xLogMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            latest_lux = lux_log[log_index - 1];
            xSemaphoreGive(xLogMutex);
        }

        // Print telemetry with latest lux
        printf("\n[Telemetry] All spacecraft systems nominal. "
               "Reading #%lu | Latest lux reading: %d | Wrapped: %d | Timestamp: %lums | Period = %lu ms\n",
               total_readings, latest_lux, wrapped, currentTime, period);

        vTaskDelay(pdMS_TO_TICKS(7000)); // Delay for 7s
    }
    vTaskDelete(NULL); // Never reached
}

// Task: Solar Sensor Task
// Task: Solar Sensor Task (Updated for App 3)
void solar_sensor_task(void *pvParameters) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);

    const TickType_t periodTicks = pdMS_TO_TICKS(200);
    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t previousTime = pdTICKS_TO_MS(lastWakeTime);

    while (1) {
        // --- Read current sensor value ---
        int raw = adc1_get_raw(LDR_ADC_CHANNEL);

        const float Vsource = 3.3;
        const float Rfixed = 10000.0;
        const float R_L10 = 50000.0;
        const float gamma = 0.7;

        float Vmeasure = ((float)raw / 4095.0) * Vsource;
        float Rmeasure = Rfixed * Vmeasure / (Vsource - Vmeasure);
        float lux = pow((R_L10 / Rmeasure), (1.0 / gamma)) * 10.0;

        // --- Store in circular buffer safely ---
        if (xSemaphoreTake(xLogMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lux_log[log_index] = (int)lux;
            log_index = (log_index + 1) % LOG_SIZE;
            xSemaphoreGive(xLogMutex);
        }


        total_readings++;
        if(total_readings % LOG_SIZE == 0 && total_readings != 0){
            wrapped++;
        }

        // --- Timing diagnostic ---
        TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount());
        previousTime = now;

        // --- Delay precisely until next sample ---
        vTaskDelayUntil(&lastWakeTime, periodTicks);
    }
}

// Task: Wait for button, "compress" and dump log
void logger_task(void *arg) {
    while (1) {
        // Block until ISR signals button press
        if (xSemaphoreTake(xButtonSem, portMAX_DELAY) == pdTRUE) {
            ESP_LOGV(TAG, "logger entered based on button semaphore; will release on exit\n");
            uint16_t copy[LOG_SIZE];
            int count = 0;

            // Lock mutex to read from buffer
            if (xSemaphoreTake(xLogMutex, pdMS_TO_TICKS(100))) {
                ESP_LOGV(TAG, "logger took log data semaphore for copy\n");
                memcpy(copy, lux_log, sizeof(copy));
                count = log_index;
                xSemaphoreGive(xLogMutex);
                ESP_LOGV(TAG, "logger releasing log data semaphore\n");
            }

            // Simulate compression (calculate stats)
            uint16_t min = 4095, max = 0;
            uint32_t sum = 0;

            count = (total_readings < LOG_SIZE) ? total_readings : LOG_SIZE;

            for (int i = 0; i < count; i++) {
                if (copy[i] < min) min = copy[i];
                if (copy[i] > max) max = copy[i];
                sum += copy[i];
            }

            uint16_t avg = (count > 0) ? (sum / count) : 0;

            ESP_LOGI(TAG, "[LOG DUMP] readings %d: min=%d, max=%d, avg=%d", total_readings, min, max, avg);
        }
    }
}

// Main: Initialize hardware and create tasks
void app_main() {
    // Beacon LED
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // Light sensor
    gpio_reset_pin(LDR_PIN);
    gpio_set_direction(LDR_PIN, GPIO_MODE_INPUT);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // Create synchronization primitives
    xLogMutex = xSemaphoreCreateMutex();
    xButtonSem = xSemaphoreCreateBinary();
    assert(xLogMutex && xButtonSem);
    xSemaphoreTake(xButtonSem, 0);  // ensure it starts empty

    //Button
    gpio_install_isr_service(0);
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_GPIO);
    gpio_set_intr_type(BUTTON_GPIO, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);

    // Create themed tasks pinned to core 1
    xTaskCreatePinnedToCore(beacon_task, "BeaconTask", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(telemetry_task, "TelemetryTask", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(solar_sensor_task, "SolarSensorTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(logger_task, "LoggerTask", 4096, NULL, 3, NULL, 1);
}