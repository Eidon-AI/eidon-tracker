#ifndef BATTERY_H
#define BATTERY_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Battery monitoring configuration
#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_0  // GPIO0 (A0)
#define BATTERY_READ_INTERVAL_MS 5000         // Read battery every 5 seconds

// Battery state structure
typedef struct {
    float voltage;              // Battery voltage in volts
    uint8_t percentage;         // Battery percentage (0-100)
    bool present;               // Battery presence flag
} battery_state_t;

// Function declarations
esp_err_t battery_init(void);
void battery_task(void *pvParameters);
battery_state_t battery_get_state(void);

#endif // BATTERY_H
