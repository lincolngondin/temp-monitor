#include "dht.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "u8g2.h"
#include "u8g2_esp32_hal.h"
#include "u8x8.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/lock.h>
#include <unistd.h>

static const char *TAG = "tempMonitor";

// Sensor
#define PIN_DATA GPIO_NUM_23

// Buttons
#define PIN_UP_VALUE_NUM GPIO_NUM_18
#define PIN_DOWN_VALUE_NUM GPIO_NUM_4
#define PIN_CHANGE_STATE GPIO_NUM_19

// LCD configs
#define PIN_SDA_NUM GPIO_NUM_21
#define PIN_SCL_NUM GPIO_NUM_22
#define PIN_RST_NUM -1
#define LCD_H_RES 128
#define LCD_V_RES 64

// Buzzer configs
#define PIN_BUZZER_NUM GPIO_NUM_5

// Sensor ranges
#define MIN_SENSOR_TEMPERATURE_IN_CELSIUS 0
#define MAX_SENSOR_TEMPERATURE_IN_CELSIUS 50

// Timers
#define HOLD_TIME_US 1000000

u8g2_t u8g2;

// struct que armazena as leituras do sensor
typedef struct sensor_readings {
  float humidity;
  float temperature;
} sensor_readings;

// instancia global que armazena a ultima leitura do sensor
sensor_readings sensor_values = {0, 0};
SemaphoreHandle_t sensor_mutex;

// atualiza o valor da leitura na instancia global
void updateSensorReading(float hum, float temp) {
  // Save the values in global variable
  xSemaphoreTake(sensor_mutex, portMAX_DELAY);
  {
    sensor_values.temperature = temp;
    sensor_values.humidity = hum;
  }
  xSemaphoreGive(sensor_mutex);
}

// salva a leitura do sensor nas variaveis
void getSensorReadings(float *hum, float *temp) {
  xSemaphoreTake(sensor_mutex, portMAX_DELAY);
  {
    *hum = sensor_values.humidity;
    *temp = sensor_values.temperature;
  }
  xSemaphoreGive(sensor_mutex);
}

// struct que armazena os intervalos de temperatura superior e inferior esperado
// pelo sensor
typedef struct temperature_limits {
  int maxSuperiorTemperatureInCelsius;
  int minInferiorTemperatureInCelsius;
  SemaphoreHandle_t mu;
} temperature_limits;

// Temperatures values
// sensor_readings.temperature must be minInferiorTemperatureInCelsius < and
// maxSuperiorTemperatureInCelsius > to alert
temperature_limits limits;

int init_temperature_limits(temperature_limits *tl) {
  tl->mu = xSemaphoreCreateMutex();
  if (tl->mu == NULL) {
    return -1;
  }
  tl->maxSuperiorTemperatureInCelsius = MAX_SENSOR_TEMPERATURE_IN_CELSIUS;
  tl->minInferiorTemperatureInCelsius = MIN_SENSOR_TEMPERATURE_IN_CELSIUS;
  return 0;
}

int get_max_superior_temperature(temperature_limits *tl) {
  int value = 0;
  xSemaphoreTake(tl->mu, portMAX_DELAY);
  { value = tl->maxSuperiorTemperatureInCelsius; }
  xSemaphoreGive(tl->mu);
  return value;
}

int get_min_inferior_temperature(temperature_limits *tl) {
  int value = 0;
  xSemaphoreTake(tl->mu, portMAX_DELAY);
  { value = tl->minInferiorTemperatureInCelsius; }
  xSemaphoreGive(tl->mu);
  return value;
}

void get_values(temperature_limits *tl, int *max_sup, int *min_inf) {
  xSemaphoreTake(tl->mu, portMAX_DELAY);
  {
    *max_sup = tl->maxSuperiorTemperatureInCelsius;
    *min_inf = tl->minInferiorTemperatureInCelsius;
  }
  xSemaphoreGive(tl->mu);
}

void increase_temperature_value(temperature_limits *tl, int maxValue) {
  xSemaphoreTake(tl->mu, portMAX_DELAY);
  {
    if (maxValue != 0) {
      if ((++tl->maxSuperiorTemperatureInCelsius) >
          MAX_SENSOR_TEMPERATURE_IN_CELSIUS) {
        tl->maxSuperiorTemperatureInCelsius = MAX_SENSOR_TEMPERATURE_IN_CELSIUS;
      }
    } else {
      if ((++tl->minInferiorTemperatureInCelsius) >=
          tl->maxSuperiorTemperatureInCelsius) {
        tl->minInferiorTemperatureInCelsius =
            tl->maxSuperiorTemperatureInCelsius - 1;
      }
    }
  }
  xSemaphoreGive(tl->mu);
}

void decrease_temperature_value(temperature_limits *tl, int maxValue) {
  xSemaphoreTake(tl->mu, portMAX_DELAY);
  {
    if (maxValue != 0) {
      if ((--tl->maxSuperiorTemperatureInCelsius) <=
          tl->minInferiorTemperatureInCelsius) {
        tl->maxSuperiorTemperatureInCelsius =
            tl->minInferiorTemperatureInCelsius + 1;
      }
    } else {
      if ((--tl->minInferiorTemperatureInCelsius) <
          MIN_SENSOR_TEMPERATURE_IN_CELSIUS) {
        tl->minInferiorTemperatureInCelsius = MIN_SENSOR_TEMPERATURE_IN_CELSIUS;
      }
    }
  }
  xSemaphoreGive(tl->mu);
}

static QueueHandle_t gpio_evt_queue = NULL;

// todos os estados em que o dispositivo pode estar
typedef enum {
  idle = 0,
  monitoring,
  changing_inferior_temperature,
  changing_superior_temperature,
  alerting,
} monitor_state;

// struct que armazena o estado atual do dispositivo e gerencia o acesso a essa
// informacao
typedef struct monitor {
  monitor_state state;
  SemaphoreHandle_t mu;
} monitor;

void init_monitor(monitor *m) {
  m->state = idle;
  m->mu = xSemaphoreCreateMutex();
}

// vai para o proximo estado
void go_next_state(monitor *m) {
  monitor_state actual_state;
  xSemaphoreTake(m->mu, portMAX_DELAY);
  {
    actual_state = m->state;
    if (actual_state == idle) {
      m->state = changing_inferior_temperature;
    } else if (actual_state == monitoring) {
      m->state = idle;
    } else if (actual_state == changing_inferior_temperature) {
      m->state = changing_superior_temperature;
    } else if (actual_state == changing_superior_temperature) {
      m->state = idle;
    } else if (actual_state == alerting) {
      m->state = idle;
    }
  }
  xSemaphoreGive(m->mu);
}

// muda para um estado arbitrario
// usada para o alerta
void change_state(monitor *m, monitor_state state) {
  xSemaphoreTake(m->mu, portMAX_DELAY);
  { m->state = state; }
  xSemaphoreGive(m->mu);
}

// obtem o estado atual
monitor_state get_state(monitor *m) {
  monitor_state st;
  xSemaphoreTake(m->mu, portMAX_DELAY);
  { st = m->state; }
  xSemaphoreGive(m->mu);

  return st;
}

// instancia global do estado do dispositivo
monitor global_monitor;

// icones usados no display
const uint8_t bell_icon_ringing[] = {
    0x20, 0x04, 0x48, 0x12, 0x93, 0xc9, 0xa4, 0x25, 0xa4, 0x25, 0x28,
    0x14, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x10, 0x08, 0x10, 0x08,
    0x10, 0x08, 0x1f, 0xf8, 0x02, 0x40, 0x01, 0x80, 0x00, 0x00};

const uint8_t bell_icon[] = {0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x04, 0x20,
                             0x04, 0x20, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10,
                             0x08, 0x10, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08,
                             0x1f, 0xf8, 0x02, 0x40, 0x01, 0x80, 0x00, 0x00};

/**
 * Task that receives button input
 *
 */
void vTaskButtons(void *pvParametes) {
  uint32_t io_num;
  int level = 0;
  monitor_state state;

  // armazena o estado de cada botao
  bool pin_up_state;
  bool pin_down_state;
  bool pin_change_state;

  // tempos usados para ativar o monitoramento
  int64_t temp_before;
  int64_t temp_after;
  for (;;) {
    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      level = gpio_get_level(io_num);
      if (level == 1) {
        if (io_num == PIN_UP_VALUE_NUM) {
          pin_up_state = true;
        } else if (io_num == PIN_DOWN_VALUE_NUM) {
          pin_down_state = true;
        } else if (io_num == PIN_CHANGE_STATE) {
          temp_before = esp_timer_get_time();
          pin_change_state = true;
        }
      }
      if (level == 0) {
        if (io_num == PIN_UP_VALUE_NUM) {
          pin_up_state = false;
        } else if (io_num == PIN_DOWN_VALUE_NUM) {
          pin_down_state = false;
        } else if (io_num == PIN_CHANGE_STATE) {
          temp_after = esp_timer_get_time();
          pin_change_state = false;
        }
      }
    }

    state = get_state(&global_monitor);
    if (pin_up_state) {
      if (state == changing_inferior_temperature) {
        increase_temperature_value(&limits, 0);
      } else if (state == changing_superior_temperature) {
        increase_temperature_value(&limits, 1);
      }
    }
    if (pin_down_state) {
      if (state == changing_inferior_temperature) {
        decrease_temperature_value(&limits, 0);
      } else if (state == changing_superior_temperature) {
        decrease_temperature_value(&limits, 1);
      }
    }
    if ((temp_after - temp_before) > HOLD_TIME_US) {
      change_state(&global_monitor, monitoring);
    } else if (pin_change_state) {
      go_next_state(&global_monitor);
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

  for (;;) {
    read_err =
        dht_read_float_data(sensor_type, PIN_DATA, &humidity, &temperature);
    if (read_err != ESP_OK) {
      ESP_LOGE(TAG, "Error reading DHT11 sensor data: %s",
               esp_err_to_name(read_err));
    }
    // ESP_LOGI(TAG, "Humidity: %f  Temperature: %f", humidity, temperature);
    updateSensorReading(humidity, temperature);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

/**
 * Task to update the OLED display with the sensor readings
 */
void vTaskDisplayTemperatureAndHumidity(void *pvParameters) {
  char str[40];
  char tempStrCI[5];
  char tempStrCS[5];
  int mi = 0;
  int ms = 0;
  monitor_state m_state;
  float hum;
  float temp;
  uint8_t w;
  uint8_t width_cs;
  uint8_t width_ci;
  u8g2_uint_t x;
  bool ringing = true;

  for (;;) {
    getSensorReadings(&hum, &temp);

    m_state = get_state(&global_monitor);
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawFrame(&u8g2, 4, 4, 124, 60);

    if (m_state == idle || m_state == monitoring) {
      u8g2_SetFont(&u8g2, u8g2_font_10x20_mf);
      snprintf(str, sizeof(str), "Temp: %.0f", temp);
      u8g2_DrawStr(&u8g2, 10, 35, str);
      w = u8g2_GetStrWidth(&u8g2, str);
      u8g2_SetFont(&u8g2, u8g2_font_helvB12_tf);
      u8g2_DrawUTF8(&u8g2, w + 10, 35, "°C");
    } else if (m_state == changing_inferior_temperature ||
               m_state == changing_superior_temperature) {
      u8g2_SetFont(&u8g2, u8g2_font_6x12_mf);
      snprintf(str, sizeof(str), "intervalo de temp:");
      u8g2_DrawStr(&u8g2, 10, 20, str);

      u8g2_SetFont(&u8g2, u8g2_font_9x15_mf);
      get_values(&limits, &ms, &mi);
      snprintf(str, sizeof(str), "%d < %.0f < %d", mi, temp, ms);

      w = u8g2_GetStrWidth(&u8g2, str);
      u8g2_DrawStr(&u8g2, 10, 45, str);

      if (m_state == changing_inferior_temperature) {
        snprintf(tempStrCI, sizeof(tempStrCI), "%d", mi);
        width_ci = u8g2_GetStrWidth(&u8g2, tempStrCI);
        x = 8;
        u8g2_DrawFrame(&u8g2, x, 30, width_ci + 6, 20);
      } else {
        snprintf(tempStrCS, sizeof(tempStrCS), "%d", ms);
        width_cs = u8g2_GetStrWidth(&u8g2, tempStrCS);
        x = w - width_cs + 6;
        u8g2_DrawFrame(&u8g2, x, 30, width_cs + 6, 20);
      }
    }

    if (m_state == monitoring) {
      snprintf(str, sizeof(str), "monitorando...");
      u8g2_DrawStr(&u8g2, 10, 54, str);
    } else if (m_state == alerting) {
      snprintf(str, sizeof(str), "AVISO!!");
      u8g2_DrawStr(&u8g2, 30, 25, str);
      if (ringing) {
        u8g2_DrawBitmap(&u8g2, 54, 35, sizeof(uint8_t) * 2, 16, bell_icon);
      } else {
        u8g2_DrawBitmap(&u8g2, 54, 35, sizeof(uint8_t) * 2, 16,
                        bell_icon_ringing);
      }
      ringing = !ringing;
    }

    u8g2_SendBuffer(&u8g2);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

/**
 * Task to trigger the buzzer
 */
void vTaskAlarm(void *pvParameters) {
  gpio_reset_pin(PIN_BUZZER_NUM);
  gpio_set_direction(PIN_BUZZER_NUM, GPIO_MODE_OUTPUT);
  monitor_state state;
  float actualTemp;
  float actualHum;
  for (;;) {
    state = get_state(&global_monitor);
    getSensorReadings(&actualHum, &actualTemp);
    if (state == monitoring || state == alerting) {
      if ((actualTemp >= get_max_superior_temperature(&limits)) ||
          (actualTemp <= get_min_inferior_temperature(&limits)) ||
          state == alerting) {
        if (state != alerting) {
          change_state(&global_monitor, alerting);
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(PIN_BUZZER_NUM, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(PIN_BUZZER_NUM, 0);
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

// configura o display oled ssd1306
void setup_display() {
  // Setup display
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.bus.i2c.sda = PIN_SDA_NUM;
  u8g2_esp32_hal.bus.i2c.scl = PIN_SCL_NUM;
  u8g2_esp32_hal_init(u8g2_esp32_hal);
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb,
                                         u8g2_esp32_gpio_and_delay_cb);

  u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

  ESP_LOGI(TAG, "u8g2_InitDisplay");
  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in
                           // sleep mode after this,

  ESP_LOGI(TAG, "u8g2_SetPowerSave");
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
}

// delay de debounce
#define DEBOUNCE_DELAY_MS 50

// interrupção que ira executar sempre que algum botao for pressionado
static void IRAM_ATTR get_switch_input(void *arg) {
  static uint32_t last_time = 0;
  uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
  if ((current_time - last_time) > DEBOUNCE_DELAY_MS) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
  }
  last_time = current_time;
}

// configura os botoes, o tipo de interrupção deve captura tanto o ato de
// pressionar quanto o de soltar o botao
void config_button(gpio_num_t pin) {
  // gpio_reset_pin(pin);
  gpio_set_direction(pin, GPIO_MODE_INPUT);
  gpio_pulldown_en(pin);
  gpio_pullup_dis(pin);
  gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE);
}

// instala todos os servicos de interrupções nos botoes chamando a mesma funcao
// de interrupção e passando o botao pressionado
void add_interrupt_services() {
  esp_err_t install_err = gpio_install_isr_service(0);
  if (install_err != ESP_OK) {
    ESP_LOGE(TAG, "Error installing isr service: %s",
             esp_err_to_name(install_err));
  }
  esp_err_t add_up_handler_err = gpio_isr_handler_add(
      PIN_UP_VALUE_NUM, get_switch_input, (void *)PIN_UP_VALUE_NUM);
  if (add_up_handler_err != ESP_OK) {
    ESP_LOGE(TAG, "Error adding up handler: %s",
             esp_err_to_name(add_up_handler_err));
  }

  esp_err_t add_down_handler_err = gpio_isr_handler_add(
      PIN_DOWN_VALUE_NUM, get_switch_input, (void *)PIN_DOWN_VALUE_NUM);
  if (add_down_handler_err != ESP_OK) {
    ESP_LOGE(TAG, "Error adding down handler: %s",
             esp_err_to_name(add_down_handler_err));
  }

  esp_err_t add_change_handler_err = gpio_isr_handler_add(
      PIN_CHANGE_STATE, get_switch_input, (void *)PIN_CHANGE_STATE);
  if (add_change_handler_err != ESP_OK) {
    ESP_LOGE(TAG, "Error adding change handler: %s",
             esp_err_to_name(add_change_handler_err));
  }
}

void app_main(void) {

  init_monitor(&global_monitor);
  init_temperature_limits(&limits);

  setup_display();
  sensor_mutex = xSemaphoreCreateMutex();
  if (sensor_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create mutex!");
    abort();
  }

  // create the queue of the switch inputs
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

  // config all the interface buttons
  config_button(PIN_UP_VALUE_NUM);
  config_button(PIN_DOWN_VALUE_NUM);
  config_button(PIN_CHANGE_STATE);

  add_interrupt_services();

  BaseType_t taskButtonsCreation = xTaskCreatePinnedToCore(
      vTaskButtons, "TASK_GET_INPUTS", 4096, NULL, 10, NULL, 0);
  if (taskButtonsCreation != pdPASS) {
    ESP_LOGE(TAG, "Error creating task: %s",
             esp_err_to_name(taskButtonsCreation));
  }

  BaseType_t taskReadSensorCreation = xTaskCreatePinnedToCore(
      vTaskReadSensor, "TASK_READ_SENSOR", 8096, NULL, 10, NULL, 1);
  if (taskReadSensorCreation != pdPASS) {
    ESP_LOGE(TAG, "Error creating task: %s",
             esp_err_to_name(taskReadSensorCreation));
  }

  BaseType_t taskDisplayCreation =
      xTaskCreatePinnedToCore(vTaskDisplayTemperatureAndHumidity,
                              "TASK_DISPLAY_READING", 4096, NULL, 2, NULL, 0);
  if (taskDisplayCreation != pdPASS) {
    ESP_LOGE(TAG, "Error creating task: %s",
             esp_err_to_name(taskDisplayCreation));
  }

  BaseType_t taskAlarmCreation =
      xTaskCreatePinnedToCore(vTaskAlarm, "TASK_ALARM", 4096, NULL, 4, NULL, 1);
  if (taskAlarmCreation != pdPASS) {
    ESP_LOGE(TAG, "Error creating task: %s",
             esp_err_to_name(taskAlarmCreation));
  }
}
