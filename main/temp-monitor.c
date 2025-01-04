#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include "dht.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "u8g2.h"
#include "u8g2_esp32_hal.h"

static const char* TAG = "tempMonitor";

#define PIN_DATA GPIO_NUM_23

// LCD configs
#define PIN_SDA_NUM GPIO_NUM_21
#define PIN_SCL_NUM GPIO_NUM_22
#define PIN_RST_NUM -1
#define LCD_H_RES 128
#define LCD_V_RES 64


// Buzzer configs
#define PIN_BUZZER_NUM GPIO_NUM_34

TaskHandle_t taskReadSensorHandler;
TaskHandle_t taskDisplayReadings;
u8g2_t u8g2;

typedef struct sensor_readings {
    float humidity;
    float temperature;
} sensor_readings;

sensor_readings sensor_values = {0, 0};
SemaphoreHandle_t sensor_mutex;

/**
 * Main task that reads the sensor DHT11 and save the values
 */
void vTaskReadSensor(void *pvParameters) {
    dht_sensor_type_t sensor_type = DHT_TYPE_DHT11;
    float humidity = 0;
    float temperature = 0;
    esp_err_t read_err;
    
    for(;;){
        read_err = dht_read_float_data(sensor_type, PIN_DATA, &humidity, &temperature);
        if(read_err != ESP_OK) {
            ESP_LOGE(TAG, "Error reading DHT11 sensor data: %s", esp_err_to_name(read_err));
        }
        ESP_LOGI(TAG, "Humidity: %f  Temperature: %f", humidity, temperature);
        // Save the values in global variable
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        {
            sensor_values.temperature = temperature;
            sensor_values.humidity = humidity;
        }
        xSemaphoreGive(sensor_mutex);
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


/**
 * Task to update the OLED display with the sensor readings
 */
void vTaskDisplayTemperatureAndHumidity(void *pvParameters){
    char str[40];

    for(;;){
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_DigitalDisco_tf);
        snprintf(str, sizeof(str), "Temp: %.2f",sensor_values.temperature);
        u8g2_DrawStr(&u8g2, 2, 20, str);
        snprintf(str, sizeof(str), "Hum: %.2f",sensor_values.humidity);
        u8g2_DrawStr(&u8g2, 2, 40, str);
        u8g2_SendBuffer(&u8g2);

        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void setup_display(){
    // Setup display
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.bus.i2c.sda = PIN_SDA_NUM;
    u8g2_esp32_hal.bus.i2c.scl = PIN_SCL_NUM;
    u8g2_esp32_hal_init(u8g2_esp32_hal);
    u8g2_Setup_ssd1306_i2c_128x64_noname_f( &u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);

    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

    ESP_LOGI(TAG, "u8g2_InitDisplay");
    u8g2_InitDisplay(&u8g2);  // send init sequence to the display, display is in
                            // sleep mode after this,

    ESP_LOGI(TAG, "u8g2_SetPowerSave");
    u8g2_SetPowerSave(&u8g2, 0);  // wake up display
}

void app_main(void)
{
    setup_display();
    sensor_mutex = xSemaphoreCreateMutex();
    if(sensor_mutex == NULL){
        ESP_LOGE(TAG, "Failed to create mutex!");
        abort();
    }
    xTaskCreatePinnedToCore(vTaskReadSensor, "TASK_READ_SENSOR", 8096, NULL, 1, &taskReadSensorHandler, 1);
    BaseType_t taskDisplayCreation = xTaskCreatePinnedToCore(vTaskDisplayTemperatureAndHumidity, "TASK_DISPLAY_READING", 4096, NULL, 2, &taskDisplayReadings, tskNO_AFFINITY);
    if(taskDisplayCreation != pdPASS) {
        ESP_LOGE(TAG, "Error creating task: %s", esp_err_to_name(taskDisplayCreation));
    }
}

