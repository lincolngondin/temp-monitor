#include <stdio.h>
#include "dht.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "hal/gpio_types.h"
#include "portmacro.h"

static const char* TAG = "tempMonitor";

#define PIN_DATA GPIO_NUM_25
#define PIN_POWER_SENSOR GPIO_NUM_26

TaskHandle_t taskReadSensorHandler;
/**
 * Main task that reads the sensor DHT11 and save the values
 */
void vTaskReadSensor(void *pvParameters) {
    // reset the power pin
    gpio_reset_pin(PIN_POWER_SENSOR);
    gpio_set_direction(PIN_POWER_SENSOR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_POWER_SENSOR, 1);

    dht_sensor_type_t sensor_type = DHT_TYPE_DHT11;
    float humidity = 0;
    float temperature = 0;
    esp_err_t read_err;
    
    for(;;){
        ESP_LOGW(TAG, "Reading sensor data!");

        read_err = dht_read_float_data(sensor_type, PIN_DATA, &humidity, &temperature);
        if(read_err != ESP_OK) {
            ESP_LOGE(TAG, "Error reading DHT11 sensor data: %s", esp_err_to_name(read_err));
        }
        ESP_LOGW(TAG, "Humidity: %f  Temperature: %f", humidity, temperature);
        vTaskDelay(2500/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreatePinnedToCore(vTaskReadSensor, "TASK_READ_SENSOR", 8096, NULL, 1, &taskReadSensorHandler, 1);
}
