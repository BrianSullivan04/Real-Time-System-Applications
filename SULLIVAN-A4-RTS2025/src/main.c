/* --------------------------------------------------------------
   Application: 04 - Rev4
   Release Type: Complete code
   Class: Real Time Systems - Fall 2025
   Author: Brian Sullivan
---------------------------------------------------------------*/

#include <stdio.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"

// === GPIO ASSIGNMENTS ===
#define LED_GREEN   GPIO_NUM_5
#define LED_RED     GPIO_NUM_4
#define BUTTON_PIN  GPIO_NUM_18
#define HR_ADC_CH   ADC1_CHANNEL_6  // GPIO34

// === PARAMETERS ===
#define HEARTBEAT_MS       1000
#define SAMPLE_PERIOD_MS   100
#define BPM_MIN_SAFE       30
#define BPM_MAX_SAFE       200
#define MAX_COUNT_SEM      10  // maximum queued sensor alerts

// === SYNCHRONIZATION ===
static SemaphoreHandle_t xPrintMutex;
static SemaphoreHandle_t xButtonSem;
static SemaphoreHandle_t xSensorCountSem;     // counting semaphore (alerts)
static SemaphoreHandle_t xSensorRecoverySem;  // counting semaphore (recovery)

// === GLOBAL STATE ===
volatile bool emergency_mode = false;
volatile bool alert_lock = false;
volatile TickType_t last_button_isr_time = 0;
TaskHandle_t xHeartbeatTask = NULL;
TaskHandle_t xSensorTask = NULL;

// === ISR: Button Toggle ===
static void IRAM_ATTR button_isr_handler(void *arg) {
    TickType_t now = xTaskGetTickCountFromISR();
    const TickType_t debounce_ticks = pdMS_TO_TICKS(200);
    if ((now - last_button_isr_time) > debounce_ticks) {
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(xButtonSem, &hpw);
        last_button_isr_time = now;
        if (hpw == pdTRUE) portYIELD_FROM_ISR();
    }
}

// === Task: Heartbeat ===
static void heartbeat_task(void *arg) {
    bool on = false;
    while (1) {
        // Only blink if not in alert or emergency mode
        if (!alert_lock && !emergency_mode) {
            gpio_set_level(LED_GREEN, on);
            on = !on;
        } else {
            gpio_set_level(LED_GREEN, 0);  // ensure off during critical/emergency
        }
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_MS));
    }
}

// === Task: Sensor Monitor ===
static void sensor_task(void *arg) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(HR_ADC_CH, ADC_ATTEN_DB_11);
    int prev_state = 0;  // rising-edge gating for alert

    while (1) {
        int bpm = adc1_get_raw(HR_ADC_CH);

        if (!emergency_mode && xSemaphoreTake(xPrintMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            printf("[Sensor] Current Heart Rate: %d bpm\n", bpm);
            xSemaphoreGive(xPrintMutex);
        }

        // Detect unsafe range
        int unsafe = (bpm > BPM_MAX_SAFE || bpm < BPM_MIN_SAFE);

        // Alert edge (safe → unsafe)
        if (unsafe && !prev_state) {
            xSemaphoreGive(xSensorCountSem);
        }
        // Recovery edge (unsafe → safe)
        else if (!unsafe && prev_state) {
            xSemaphoreGive(xSensorRecoverySem);
        }

        prev_state = unsafe;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// === Task: Event Handler (button + sensor events) ===
static void event_handler_task(void *arg) {
    while (1) {
        // Handle sensor alert events (queued)
        if (xSemaphoreTake(xSensorCountSem, 0) == pdTRUE) {
            if (!emergency_mode && !alert_lock) {
                alert_lock = true;
                gpio_set_level(LED_RED, 1);
                gpio_set_level(LED_GREEN, 0);   // turn off green LED
                vTaskSuspend(xHeartbeatTask);

                if (xSemaphoreTake(xPrintMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    printf("[ALERT] Critical BPM out of range! System paused.\n");
                    xSemaphoreGive(xPrintMutex);
                }
            }
        }

        // Handle recovery (queued)
        if (xSemaphoreTake(xSensorRecoverySem, 0) == pdTRUE) {
            if (!emergency_mode && alert_lock) {
                alert_lock = false;
                gpio_set_level(LED_RED, 0);
                vTaskResume(xHeartbeatTask);

                if (xSemaphoreTake(xPrintMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    printf("[RECOVERY] Heart rate stabilized. System back to normal.\n");
                    xSemaphoreGive(xPrintMutex);
                }
            }
        }

        // Handle button toggles
        if (xSemaphoreTake(xButtonSem, 0) == pdTRUE) {
            emergency_mode = !emergency_mode;
            if (emergency_mode) {
                gpio_set_level(LED_RED, 1);
                gpio_set_level(LED_GREEN, 0);   // ensure off during emergency
                vTaskSuspend(xHeartbeatTask);
                vTaskSuspend(xSensorTask);

                if (xSemaphoreTake(xPrintMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    printf("[EMERGENCY] Manual override engaged.\n");
                    xSemaphoreGive(xPrintMutex);
                }
            } else {
                gpio_set_level(LED_RED, 0);
                vTaskResume(xHeartbeatTask);
                vTaskResume(xSensorTask);
                alert_lock = false;  // clear alert

                if (xSemaphoreTake(xPrintMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    printf("[RECOVERY] Emergency cleared. Resuming.\n");
                    xSemaphoreGive(xPrintMutex);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// === MAIN ===
void app_main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("Initializing Heart Rate Monitor (Counting Semaphore + LED Control)...\n");

    // LEDs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GREEN) | (1ULL << LED_RED),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(LED_RED, 0);

    // Button
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&btn_conf);

    // === Semaphores ===
    xPrintMutex = xSemaphoreCreateMutex();
    xButtonSem = xSemaphoreCreateBinary();
    xSensorCountSem = xSemaphoreCreateCounting(MAX_COUNT_SEM, 0);
    xSensorRecoverySem = xSemaphoreCreateCounting(MAX_COUNT_SEM, 0);
    assert(xPrintMutex && xButtonSem && xSensorCountSem && xSensorRecoverySem);

    // ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

    // Tasks
    xTaskCreatePinnedToCore(heartbeat_task, "HeartbeatTask", 2048, NULL, 1, &xHeartbeatTask, 1);
    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 3072, NULL, 2, &xSensorTask, 1);
    xTaskCreatePinnedToCore(event_handler_task, "EventHandlerTask", 4096, NULL, 3, NULL, 1);

    printf("System ready. Press button for emergency lock; move potentiometer to simulate BPM changes.\n");
}
