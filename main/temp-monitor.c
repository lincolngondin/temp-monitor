#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include "dht.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "u8g2.h"
#include "u8g2_esp32_hal.h"

static const char* TAG = "tempMonitor";

#define PIN_DATA GPIO_NUM_23

// Buttons
#define PIN_UP_VALUE_NUM GPIO_NUM_5
#define PIN_DOWN_VALUE_NUM GPIO_NUM_18
#define PIN_CHANGE_VALUE_NUM GPIO_NUM_19

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

void updateSensorReading(float hum ,float temp){
    // Save the values in global variable
    xSemaphoreTake(sensor_mutex, portMAX_DELAY);
    {
        sensor_values.temperature = temp;
        sensor_values.humidity = hum;
    }
    xSemaphoreGive(sensor_mutex);
}

typedef struct temperature_limits{
    int maxSuperiorTemperatureInCelsius;
    int minInferiorTemperatureInCelsius;
    SemaphoreHandle_t mu;
} temperature_limits;

// Temperatures values
// sensor_readings.temperature must be minInferiorTemperatureInCelsius < and maxSuperiorTemperatureInCelsius > to alert
temperature_limits limits;

int init_temperature_limits(temperature_limits *tl) {
    tl->mu = xSemaphoreCreateMutex();
    if(tl->mu == NULL){
        return -1;
    }
    tl->maxSuperiorTemperatureInCelsius = 0;
    tl->minInferiorTemperatureInCelsius = 0;
    return 0;
}

int get_max_superior_temperature(temperature_limits *tl){
    int value = 0;
    xSemaphoreTake(tl->mu, portMAX_DELAY);
    {
        value = tl->maxSuperiorTemperatureInCelsius;
    }
    xSemaphoreGive(tl->mu);
    return value;
}

int get_min_inferior_temperature(temperature_limits *tl){
    int value = 0;
    xSemaphoreTake(tl->mu, portMAX_DELAY);
    {
        value = tl->minInferiorTemperatureInCelsius;
    }
    xSemaphoreGive(tl->mu);
    return value;
}

void get_values(temperature_limits *tl, int *max_sup, int *min_inf){
    xSemaphoreTake(tl->mu, portMAX_DELAY);
    {
        *max_sup = tl->maxSuperiorTemperatureInCelsius;
        *min_inf = tl->minInferiorTemperatureInCelsius;
    }
    xSemaphoreGive(tl->mu);
}

void increase_temperature_value(temperature_limits *tl, int maxValue){
    xSemaphoreTake(tl->mu, portMAX_DELAY);
    {
        if(maxValue != 0){
            tl->maxSuperiorTemperatureInCelsius++;
        } else {
            tl->minInferiorTemperatureInCelsius++;
        }
    }
    xSemaphoreGive(tl->mu);
}

void decrease_temperature_value(temperature_limits *tl, int maxValue){
    xSemaphoreTake(tl->mu, portMAX_DELAY);
    {
        if(maxValue != 0){
            tl->maxSuperiorTemperatureInCelsius--;
        } else {
            tl->minInferiorTemperatureInCelsius--;
        }
    }
    xSemaphoreGive(tl->mu);
}


static QueueHandle_t gpio_evt_queue = NULL;

/**
 * Task that receives button input
 *
 */
void vTaskButtons(void *pvParametes) {
    uint32_t io_num;
    int level = 0;
    for(;;){
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)){
            printf("Received input: %d value: %d\n", io_num, gpio_get_level(io_num));
            level = gpio_get_level(io_num);
            if(level == 1){
                if(io_num == PIN_UP_VALUE_NUM){
                    increase_temperature_value(&limits, 0);
                }
                else if(io_num == PIN_DOWN_VALUE_NUM){
                    decrease_temperature_value(&limits, 0);
                }
                else {
                }
            }

        }
    }
    vTaskDelete(NULL);
}

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
        updateSensorReading(humidity, temperature);
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


/**
 * Task to update the OLED display with the sensor readings
 */
void vTaskDisplayTemperatureAndHumidity(void *pvParameters){
    char str[40];
    int mi = 0;
    int ms = 0;

    for(;;){
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_DigitalDisco_tf);
        snprintf(str, sizeof(str), "Temp: %.2f",sensor_values.temperature);
        u8g2_DrawStr(&u8g2, 2, 20, str);
        snprintf(str, sizeof(str), "Hum: %.2f",sensor_values.humidity);
        u8g2_DrawStr(&u8g2, 2, 40, str);

        
        get_values(&limits, &ms, &mi);
        snprintf(str, sizeof(str), "%d < X < %d",mi, ms);
        u8g2_DrawStr(&u8g2, 2, 60, str);

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

static void IRAM_ATTR get_switch_input(void *arg){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void app_main(void)
{

    init_temperature_limits(&limits);

    setup_display();
    sensor_mutex = xSemaphoreCreateMutex();
    if(sensor_mutex == NULL){
        ESP_LOGE(TAG, "Failed to create mutex!");
        abort();
    }

    // create the queue of the switch inputs
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //gpio_reset_pin(PIN_UP_VALUE_NUM);
    gpio_set_direction(PIN_UP_VALUE_NUM, GPIO_MODE_INPUT);
    gpio_pulldown_en(PIN_UP_VALUE_NUM);
    gpio_pullup_dis(PIN_UP_VALUE_NUM);
    gpio_set_intr_type(PIN_UP_VALUE_NUM, GPIO_INTR_POSEDGE);

    /*
    gpio_reset_pin(PIN_DOWN_VALUE_NUM);
    gpio_set_direction(PIN_DOWN_VALUE_NUM, GPIO_MODE_INPUT);
    gpio_pulldown_en(PIN_DOWN_VALUE_NUM);
    gpio_pullup_dis(PIN_DOWN_VALUE_NUM);
    gpio_set_intr_type(PIN_DOWN_VALUE_NUM, GPIO_INTR_POSEDGE);
    */

    esp_err_t install_err = gpio_install_isr_service(0);
    if(install_err != ESP_OK){
        ESP_LOGE(TAG, "Error installing isr service: %s", esp_err_to_name(install_err));
    }
    esp_err_t add_up_handler_err = gpio_isr_handler_add(PIN_UP_VALUE_NUM, get_switch_input, (void*)PIN_UP_VALUE_NUM);
    if(add_up_handler_err != ESP_OK){
        ESP_LOGE(TAG, "Error adding up handler: %s", esp_err_to_name(add_up_handler_err));
    }

    /*
    
    esp_err_t add_down_handler_err = gpio_isr_handler_add(PIN_DOWN_VALUE_NUM, decrease_value, NULL);
    if(add_down_handler_err != ESP_OK){
        ESP_LOGE(TAG, "Error adding down handler: %s", esp_err_to_name(add_down_handler_err));
    }
    */

    BaseType_t taskButtonsCreation = xTaskCreatePinnedToCore(vTaskButtons, "TASK_GET_INPUTS", 4096, NULL, 10, NULL, 0);
    if(taskButtonsCreation != pdPASS) {
        ESP_LOGE(TAG, "Error creating task: %s", esp_err_to_name(taskButtonsCreation));
    }

    xTaskCreatePinnedToCore(vTaskReadSensor, "TASK_READ_SENSOR", 8096, NULL, 1, &taskReadSensorHandler, 1);
    BaseType_t taskDisplayCreation = xTaskCreatePinnedToCore(vTaskDisplayTemperatureAndHumidity, "TASK_DISPLAY_READING", 4096, NULL, 2, &taskDisplayReadings, tskNO_AFFINITY);
    if(taskDisplayCreation != pdPASS) {
        ESP_LOGE(TAG, "Error creating task: %s", esp_err_to_name(taskDisplayCreation));
    }
}

