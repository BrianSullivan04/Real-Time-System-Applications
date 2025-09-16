/* --------------------------------------------------------------
   Application: 02 - Rev1
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

#define LED_PIN GPIO_NUM_2       // Beacon LED
#define LDR_PIN GPIO_NUM_32      // Simulated solar sensor pin
#define LDR_ADC_CHANNEL ADC1_CHANNEL_4

//Define average window size globally
#define AVG_WINDOW 10

// Task: Beacon Task 
void beacon_task(void *pvParameters) {
    bool led_on = false;
    TickType_t currentTime = pdTICKS_TO_MS(xTaskGetTickCount()); //Get current time of running code
    TickType_t previousTime = 0; //Initialize previous time to 0

    while (1) {
        //Handle timing
        previousTime = currentTime; //Update previous time
        currentTime = pdTICKS_TO_MS(xTaskGetTickCount()); //Update current time
        TickType_t period = (currentTime - previousTime); //Calculate period

        gpio_set_level(LED_PIN, led_on);
        led_on = !led_on; // toggle state for next time

        //Print message to indicate LED is blinking as intended
        printf("\n[Telemetry] Beacon %s | Timestamp: %lums | Period = %lu ms\n",
               led_on ? "ON" : "OFF", currentTime, period);

        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 ms using MS to Ticks Function vs alternative which is MS / ticks per ms
    }
    vTaskDelete(NULL); // We'll never get here; tasks run forever
}

// Task: Telemetry Task
void telemetry_task(void *pvParameters) {
    TickType_t currentTime = pdTICKS_TO_MS(xTaskGetTickCount()); //Get current time of running code
    TickType_t previousTime = 0; //Initialize previous time to 0
    while (1) {
        //Handle timing
        previousTime = currentTime; //Update previous time
        currentTime = pdTICKS_TO_MS(xTaskGetTickCount()); //Update current time
        TickType_t period = (currentTime - previousTime); //Calculate period

        //Print message to console showing that the spacecraft is working as intended
        printf("\n[Telemetry] All spacecraft systems nominal. "
               "Continuing orbital operations. Timestamp: %lums | Period = %lu ms\n",
               currentTime, period);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000 ms
    }
    vTaskDelete(NULL); // We'll never get here; tasks run forever
}

// Task: Solar Sensor Task
void solar_sensor_task(void *pvParameters) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // Variables to compute LUX
    int raw;
    float Vmeasure, Rmeasure, lux;
    // Variables to compute LUX
    int luxreadings[AVG_WINDOW] = {0};
    int idx = 0;
    float sum = 0;

    //TODO11a consider where AVG_WINDOW is defined, it could be here, or global value
    const int SENSOR_THRESHOLD = 500;   // Arbitrary lux threshold for eclipse entry

    //See TODO 99
    // Pre-fill the readings array with an initial sample to avoid startup anomaly
    for (int i = 0; i < AVG_WINDOW; ++i) {
        raw = adc1_get_raw(LDR_ADC_CHANNEL);
        //Define constant variables for calculating lux
        const float Vsource = 3.3;
        const float Rfixed = 10000.0;
        const float R_L10 = 50000.0;
        const float gamma = 0.7;

        Vmeasure = ((float)raw / 4095.0) * Vsource; //TODO11b correct this with the equation seen earlier
        Rmeasure = Rfixed * Vmeasure / (Vsource - Vmeasure); //TODO11c correct this with the equation seen earlier
        lux = pow((R_L10 / Rmeasure), (1.0 / gamma)) * 10.0; //TODO11d correct this with the equation seen earlier
        luxreadings[i] = lux;
        sum += luxreadings[i];
    }

    const TickType_t periodTicks = pdMS_TO_TICKS(500); // e.g. 500 ms period
    TickType_t lastWakeTime = xTaskGetTickCount(); // initialize last wake time

    // Track elapsed + period
    TickType_t previousTime = pdTICKS_TO_MS(lastWakeTime);

    while (1) {
        // Read current sensor value
        raw = adc1_get_raw(LDR_ADC_CHANNEL);
        //Define constant variables for calculating lux
        const float Vsource = 3.3;
        const float Rfixed = 10000.0;
        const float R_L10 = 50000.0;
        const float gamma = 0.7;

        //Compute lux
        Vmeasure = ((float)raw / 4095.0) * Vsource; //TODO11e correct this with the equation seen earlier
        Rmeasure = Rfixed * Vmeasure / (Vsource - Vmeasure); //TODO11f correct this with the equation seen earlier
        lux = pow((R_L10 / Rmeasure), (1.0 / gamma)) * 10.0; //TODO11g correct this with the equation seen earlier

        // Update moving average buffer
        sum -= luxreadings[idx]; // remove oldest value from sum


        luxreadings[idx] = lux; // place new reading
        sum += lux; // add new value to sum
        idx = (idx + 1) % AVG_WINDOW;
        int avg = sum / AVG_WINDOW; // compute average

        //TODO11h Check threshold and print alert if exceeded or below based on context
        if (avg < SENSOR_THRESHOLD) {
            printf("\n[Alert] Solar intensity drop detected! Avg=%d Lux "
                   "(Threshold=%d). Spacecraft may be entering eclipse.\n",
                   avg, SENSOR_THRESHOLD);
        } else {
            printf("\n[Sensor] Solar intensity nominal: Avg=%d Lux "
                   "(Threshold=%d)\n", avg, SENSOR_THRESHOLD);
        }

        //TODO11j: Print out time period [to help with answering Eng/Analysis quetionst (hint check Application Solution #1 )
        TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount());
        TickType_t period = now - previousTime;
        previousTime = now;

        //TODO11k Replace vTaskDelay with vTaskDelayUntil with parameters &lastWakeTime and periodTicks
        printf("[SensorTask] Mission Elapsed Time: %lu ms | Period = %lu ms\n",
               now, period);

        vTaskDelayUntil(&lastWakeTime, periodTicks);
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

    // Create themed tasks pinned to core 1
    xTaskCreatePinnedToCore(beacon_task, "BeaconTask", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(telemetry_task, "TelemetryTask", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(solar_sensor_task, "SolarSensorTask", 4096, NULL, 3, NULL, 1);
}