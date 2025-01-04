#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include "dht.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "lvgl.h"
#include "esp_lcd_panel_vendor.h"

static const char* TAG = "tempMonitor";

#define I2C_BUS_PORT 0
#define I2C_DEVICE_ADDRESS 0x3c
#define LCD_PIXEL_CLOCK_HZ (400 * 1000)
#define LCD_CMD_BITS 8

#define PIN_DATA GPIO_NUM_25
#define PIN_POWER_SENSOR GPIO_NUM_26

// LCD configs
#define PIN_SDA_NUM GPIO_NUM_33
#define PIN_SCL_NUM GPIO_NUM_32
#define PIN_RST_NUM -1
#define LCD_H_RES 128
#define LCD_V_RES 64

// LVGL configs
#define LVGL_PALETTE_SIZE 8
#define LVGL_TICK_PERIOD_MS 5
#define LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     2

// To use LV_COLOR_FORMAT_I1, we need an extra buffer to hold the converted data
static uint8_t oled_buffer[LCD_H_RES * LCD_V_RES / 8];
// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

// Buzzer configs
#define PIN_BUZZER_NUM GPIO_NUM_34

TaskHandle_t taskReadSensorHandler;
TaskHandle_t taskDisplayReadings;

typedef struct sensor_readings {
    float humidity;
    float temperature;
} sensor_readings;

sensor_readings sensor_values = {0, 0};
SemaphoreHandle_t sensor_mutex;

esp_lcd_panel_handle_t panel_handler;
esp_lcd_panel_io_handle_t io_handler;
lv_display_t *display;

void configure_i2c_bus();


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


    for(;;){
        ESP_LOGW(TAG, "Displaying sensor reading!");

        /*
        _lock_acquire(&lvgl_api_lock);

        lv_obj_t *scr = lv_display_get_screen_active(display);
        lv_obj_t *label = lv_label_create(scr);
        lv_label_set_text_fmt(label, "Hum: %f", sensor_values.humidity);

        // Size of the screen (if you use rotation 90 or 270, please use lv_display_get_vertical_resolution) 
        //lv_obj_set_width(label, lv_display_get_horizontal_resolution(display));
        //lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

        _lock_release(&lvgl_api_lock);
        */

        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        usleep(1000 * time_till_next_ms);
    }
}

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io_panel, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);

    // This is necessary because LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette. Skip the palette here
    // More information about the monochrome, please refer to https://docs.lvgl.io/9.2/porting/display.html#monochrome-displays
    px_map += LVGL_PALETTE_SIZE;

    uint16_t hor_res = lv_display_get_physical_horizontal_resolution(disp);
    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            /* The order of bits is MSB first
                        MSB           LSB
               bits      7 6 5 4 3 2 1 0
               pixels    0 1 2 3 4 5 6 7
                        Left         Right
            */
            bool chroma_color = (px_map[(hor_res >> 3) * y  + (x >> 3)] & 1 << (7 - x % 8));

            /* Write to the buffer as required for the display.
            * It writes only 1-bit for monochrome displays mapped vertically.*/
            uint8_t *buf = oled_buffer + hor_res * (y >> 3) + (x);
            if (chroma_color) {
                (*buf) &= ~(1 << (y % 8));
            } else {
                (*buf) |= (1 << (y % 8));
            }
        }
    }
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, oled_buffer);
}

void configure_i2c_bus(){
    // create the I2C bus
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = PIN_SDA_NUM,
        .scl_io_num = PIN_SCL_NUM,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    // Allocate the LCD IO device handler
    ESP_LOGI(TAG, "Install panel IO");
    io_handler = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_DEVICE_ADDRESS,
        .scl_speed_hz = LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_CMD_BITS,
        .dc_bit_offset = 6,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handler));

    // install the LCD control driver
    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    panel_handler = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = PIN_RST_NUM,
    };
    // ssd1306 specific config
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handler, &panel_config, &panel_handler));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handler));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handler));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handler, true));
}


void app_main(void)
{
    configure_i2c_bus();

    ESP_LOGI(TAG, "Initialize LVGL");
    lv_init();
    display = lv_display_create(LCD_H_RES, LCD_V_RES);
    if (display == NULL) {
        ESP_LOGE(TAG, "Failed to create an display!");
    }
    
    lv_display_set_user_data(display, panel_handler);
    void *buf = NULL;
    size_t draw_buffer_sz = LCD_H_RES * LCD_V_RES / 8 + LVGL_PALETTE_SIZE;
    buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(buf);
    // LVGL9 suooprt new monochromatic format.
    lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    /* Register done callback */
    esp_lcd_panel_io_register_event_callbacks(io_handler, &cbs, display);

    ESP_LOGI(TAG, "Use esp_timer as LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    _lock_acquire(&lvgl_api_lock);
    lv_obj_t *scr = lv_display_get_screen_active(display);
    if (scr == NULL) {
        ESP_LOGE(TAG, "Screen is NULL");
    }
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, "Hello Espressif, Hello LVGL.");
    /* Size of the screen (if you use rotation 90 or 270, please use lv_display_get_vertical_resolution) */
    lv_obj_set_width(label, lv_display_get_horizontal_resolution(display));
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    _lock_release(&lvgl_api_lock);

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

