/* --------------------------------------------------------------
   Application: 01 - Rev1
   Release Type: Complete code
   Class: Real Time Systems - Fall 2025
   Author: Brian Sullivan
---------------------------------------------------------------*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_0  // Using GPIO0 for the LED

// Task to blink an LED at 2 Hz (500 ms period: 250 ms ON, 250 ms OFF)
void spacecraft_LED_blinker(void *pvParameters) {
    bool led_on = false;
    TickType_t currentTime = pdTICKS_TO_MS( xTaskGetTickCount() ); //Get current time of running code
    TickType_t previousTime = 0; //Initialize previous time to 0
    while (1) {
        //Handle timing
        previousTime = currentTime; //Update previous time
        currentTime = pdTICKS_TO_MS( xTaskGetTickCount()); //Update current time
        TickType_t period = (currentTime - previousTime); //Calculate period

        //Toggle LED on and off
        gpio_set_level(LED_PIN, led_on);
        led_on = !led_on;  // toggle state for next time

        //Print message to indicate LED is blinking as intended
        printf("Spacecraft LED is blinking on schedule and is %s: %lums -- Period = %lu\n", led_on ? "ON" : "OFF", currentTime, period);

        vTaskDelay(pdMS_TO_TICKS(250)); // Delay for 250 ms using MS to Ticks Function vs alternative which is MS / ticks per ms
    }
    vTaskDelete(NULL); // We'll never get here; tasks run forever
}

// Task to print a message every 10000 ms (10 seconds)
void print_safety_verification(void *pvParameters) {
    TickType_t currentTime = pdTICKS_TO_MS( xTaskGetTickCount()); //Get current time of running code
    TickType_t previousTime = 0; //Initialize previous time to 0
    while (1) {
        //Handle timing
        previousTime = currentTime; //Update previous time
        currentTime = pdTICKS_TO_MS( xTaskGetTickCount()); //Update current time
        TickType_t period = (currentTime - previousTime); //Calculate period

        //Print message to console showing that the spacecraft is working as intended
        printf("Spacecraft is safely collecting data while in orbit: %lums -- Period = %lu\n", currentTime, period);

        //vTaskDelay(pdMS_TO_TICKS(10000)); // Delay for 10000 ms
    }
    vTaskDelete(NULL); // We'll never get here; tasks run forever
}

void app_main() {
    // Initialize LED GPIO     
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    //Create Tasks
    xTaskCreate(spacecraft_LED_blinker, "Spacecraft LED Blinker", 2048, NULL, 1, NULL);
    xTaskCreate(print_safety_verification, "Print safety", 2048, NULL, 1, NULL);

}