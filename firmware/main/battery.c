#include "battery.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BATTERY";

// ADC configuration
#define BATTERY_ADC_UNIT        ADC_UNIT_1
#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_0    // GPIO0 (A0)
#define BATTERY_ADC_ATTEN       ADC_ATTEN_DB_12  // 0-3.3V range
#define BATTERY_ADC_BITWIDTH    ADC_BITWIDTH_12  // 12-bit resolution

// Battery detection thresholds
#define BATTERY_MIN_VALID_VOLTAGE   2.5f  // Below this = no battery present
#define BATTERY_FLOATING_MIN        3.8f  // Floating ADC typically reads in this range
#define BATTERY_FLOATING_MAX        4.0f  // when USB powered but no battery
#define BATTERY_MAX_VOLTAGE         4.2f  // Fully charged LiPo
#define BATTERY_MIN_VOLTAGE         3.0f  // Discharged LiPo (safe cutoff)
#define VOLTAGE_DIVIDER_RATIO       2.0f  // 1:2 voltage divider (200k resistors)
#define NUM_SAMPLES                 16    // Number of samples to average

// Static variables
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static battery_state_t current_battery_state = {
    .voltage = 0.0f,
    .percentage = 0,
    .present = false
};

// Internal function to read battery voltage
static float read_battery_voltage(void)
{
    uint32_t adc_reading = 0;
    int raw_value;

    // Multisampling
    for (int i = 0; i < NUM_SAMPLES; i++) {
        if (adc_oneshot_read(adc_handle, BATTERY_ADC_CHANNEL, &raw_value) == ESP_OK) {
            adc_reading += raw_value;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    adc_reading /= NUM_SAMPLES;

    // Convert ADC reading to voltage in mV
    int voltage_mv = 0;
    if (adc_cali_handle != NULL) {
        adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage_mv);
    } else {
        // Fallback: rough estimation if calibration not available
        voltage_mv = (adc_reading * 3300) / 4095;
    }

    // Apply voltage divider ratio and convert to volts
    float battery_voltage = (voltage_mv / 1000.0f) * VOLTAGE_DIVIDER_RATIO;

    return battery_voltage;
}

// Internal function to calculate battery percentage
static uint8_t calculate_battery_percentage(float voltage)
{
    if (voltage >= BATTERY_MAX_VOLTAGE) {
        return 100;
    } else if (voltage <= BATTERY_MIN_VOLTAGE) {
        return 0;
    } else {
        // Linear interpolation
        float percentage = ((voltage - BATTERY_MIN_VOLTAGE) /
                           (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0f;
        return (uint8_t)percentage;
    }
}

// Initialize battery monitoring
esp_err_t battery_init(void)
{
    esp_err_t ret;

    // Configure ADC oneshot unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = BATTERY_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure ADC channel
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };
    ret = adc_oneshot_config_channel(adc_handle, BATTERY_ADC_CHANNEL, &channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize ADC calibration
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = BATTERY_ADC_UNIT,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration: Curve Fitting");
    } else {
        ESP_LOGW(TAG, "ADC calibration not available, using fallback calculation");
        adc_cali_handle = NULL;
    }

    ESP_LOGI(TAG, "Battery monitoring initialized on GPIO0 (ADC1_CH0)");

    // Start battery monitoring task
    TaskHandle_t battery_task_handle = NULL;
    BaseType_t task_created = xTaskCreate(battery_task, "battery_task", 3072, NULL, 2, &battery_task_handle);
    if (task_created == pdPASS) {
        ESP_LOGI(TAG, "Battery monitoring task created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create battery monitoring task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Battery monitoring task
void battery_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Battery monitoring task started");

    while (1) {
        // Read battery voltage
        float voltage = read_battery_voltage();

        // Detect battery presence
        bool battery_present = true;
        if (voltage < BATTERY_MIN_VALID_VOLTAGE) {
            battery_present = false;
            ESP_LOGD(TAG, "NO BATTERY (voltage too low: %.3fV)", voltage);
        } else if (voltage >= BATTERY_FLOATING_MIN && voltage <= BATTERY_FLOATING_MAX) {
            battery_present = false;
            ESP_LOGD(TAG, "NO BATTERY (floating ADC: %.3fV)", voltage);
        }

        // Update battery state
        current_battery_state.voltage = voltage;
        current_battery_state.present = battery_present;

        if (battery_present) {
            current_battery_state.percentage = calculate_battery_percentage(voltage);
            ESP_LOGI(TAG, "Voltage: %.2fV, Percentage: %d%%",
                     voltage, current_battery_state.percentage);
        } else {
            current_battery_state.percentage = 0;
            ESP_LOGI(TAG, "NO BATTERY DETECTED (USB powered only)");
        }

        // Wait for next reading
        vTaskDelay(pdMS_TO_TICKS(BATTERY_READ_INTERVAL_MS));
    }
}

// Get current battery state
battery_state_t battery_get_state(void)
{
    return current_battery_state;
}
