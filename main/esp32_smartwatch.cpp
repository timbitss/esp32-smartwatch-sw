#include <cstdio>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp32_smartwatch.h"
#include "ButtonDebouncer.h"
#include "I2C.h"
#include "RTClib.h"

extern "C" void app_main(void);

enum State : uint8_t;

// Private function prototypes
static uint8_t io_read_buttons();
void RunTask1Hz(void *parameters);
void RunTask200Hz(void * parameters);
static void change_state(State next_state);

// Unique component tag for logging data.
static const char *TAG = "APP_MAIN";

SemaphoreHandle_t statemachine_mutex;

// State machine.
enum State : uint8_t
{
    NORMAL = 0,
    ALARM_ADJUST,
    ALARM_FIRED,
    NUM_OF_STATES
} present_state;

const char* state_names[NUM_OF_STATES] = {"NORMAL", "ALARM_ADJUST", "ALARM_FIRED"};

void app_main(void)
{
    static I2C i2c_bus(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000, false);

    // Initialize RTC.
    static RTC_DS3231 rtc{};

    if(!rtc.begin(&i2c_bus))
    {
        ESP_LOGI(TAG, "Could not find DS3231M.");
    }
    else
    {
        ESP_LOGI(TAG, "Connected to DS3231M.");
    }

    if(rtc.lostPower())
    {
        ESP_LOGI(TAG, "RTC lost power. "
                      "Initializing date and time to %s, %s.", __DATE__, __TIME__);
        rtc.adjust(DateTime(__DATE__, __TIME__)); // Set RTC to time and date of compilation.
    }

    // Set initial alarm time to arbitrary value.
    rtc.setAlarm1(rtc.now() - TimeSpan(60), DS3231_A1_Hour);

    // Initialize button inputs
    gpio_config_t io_config;
    io_config.pin_bit_mask = (1 << GPIO_NUM_13);
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);

    io_config.pin_bit_mask = (1 << GPIO_NUM_0);
    io_config.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_config);

    xTaskCreatePinnedToCore(RunTask1Hz,
                            "RunTask1Hz",
                            TASK1HZ_STACK_SIZE,
                            &rtc,
                            2,
                            0,
                            APP_CPU_NUM);


    xTaskCreatePinnedToCore(RunTask200Hz,
    "RunTask200Hz",
    TASK200HZ_STACK_SIZE,
    &rtc,
    1,
    0,
    APP_CPU_NUM);

    statemachine_mutex = xSemaphoreCreateMutex();

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS((100000)));
    }
}

void RunTask1Hz(void *parameters)
{
    RTC_DS3231* rtc = (RTC_DS3231*) parameters;
    TickType_t lastWakeTime;
    const TickType_t period_ms = pdMS_TO_TICKS( 1000 );
    static char current_time[25], alarm_time[25];

    lastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        if(rtc->alarmFired(ALARM1)) present_state = ALARM_FIRED;

        rtc->now().timestamp(current_time);
        rtc->getAlarm1().timestamp(alarm_time, DateTime::TIMESTAMP_TIME);
        ESP_LOGI(TAG, "(1Hz) Time: %s Alarm 1: %s State: %s",
                 current_time,
                 alarm_time,
                 state_names[present_state]);

        vTaskDelayUntil(&lastWakeTime, period_ms);
    }
}

void RunTask200Hz(void *parameters)
{
    RTC_DS3231* rtc = (RTC_DS3231*) parameters;

    TickType_t lastWakeTime;
    const TickType_t period_ms = pdMS_TO_TICKS( 5 );

    uint8_t switch_bitmask = 0x3; // Using two switches
    ButtonDebouncer Buttons(io_read_buttons, switch_bitmask);
    uint8_t button_states = 0, prev_button_states = 0;
    uint8_t press_detected = 0, press_released = 0;
    uint32_t timeheld0_ms = 0, timeheld1_ms = 0;

    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        Buttons.ProcessSwitches200Hz(&button_states, &press_detected, &press_released);

        if (button_states & 0x01)
        {
            timeheld0_ms += 5;
            if (timeheld0_ms == LONG_PRESS_DURATION_MS)
            {
                change_state(present_state == NORMAL ? ALARM_ADJUST : NORMAL);
                ESP_LOGI(TAG, "Button 0 long press");
            }
        }
        if (button_states & 0x02)
        {
            timeheld1_ms += 5;
            if (timeheld1_ms == LONG_PRESS_DURATION_MS)
            {
                change_state(present_state == NORMAL ? ALARM_ADJUST : NORMAL);
                ESP_LOGI(TAG, "Button 1 long press");
            }
        }
        if (press_released & 0x01)
        {
            if (timeheld0_ms < LONG_PRESS_DURATION_MS)
            {
                if(present_state == NORMAL)
                {
                    rtc->incrementMinute();
                    DateTime now = rtc->now();
                    ESP_LOGI(TAG, "200Hz: %s", now.timestamp());
                }
                else if (present_state == ALARM_ADJUST)
                {
                    rtc->incrementAlarm1Minute();
                    DateTime alarm1 = rtc->getAlarm1();
                    ESP_LOGI(TAG, "200Hz: Alarm 1 = %s", alarm1.timestamp(DateTime::TIMESTAMP_TIME));
                }
                else
                {
                    rtc->clearAlarm(ALARM1);
                    ESP_LOGI(TAG, "Alarm 1 cleared");
                    change_state(NORMAL);
                }
            }

            ESP_LOGI(TAG, "Button 0 released");
            timeheld0_ms = 0;
        }
        if (press_released & 0x02)
        {
            if (timeheld1_ms < LONG_PRESS_DURATION_MS)
            {
                if (present_state == NORMAL)
                {
                    rtc->decrementMinute();
                    DateTime now = rtc->now();
                    ESP_LOGI(TAG, "200Hz: %s", now.timestamp());
                }
                else if (present_state == ALARM_ADJUST)
                {
                    rtc->decrementAlarm1Minute();
                    DateTime alarm1 = rtc->getAlarm1();
                    ESP_LOGI(TAG, "200Hz: Alarm 1 = %s", alarm1.timestamp(DateTime::TIMESTAMP_TIME));
                }
                else
                {
                    rtc->clearAlarm(ALARM1);
                    ESP_LOGI(TAG, "Alarm 1 cleared");
                    change_state(NORMAL);
                }
            }

            ESP_LOGI(TAG, "Button 1 released");
            timeheld1_ms = 0;
        }

        vTaskDelayUntil(&lastWakeTime, period_ms);
    }
}

// IO 13 and 15
static uint8_t io_read_buttons()
{
    uint8_t button_1_status = (uint8_t)gpio_get_level(GPIO_NUM_13) ^ 0x01; // 0 = open, 1 = closed (active-low)
    uint8_t button_2_status = (uint8_t)gpio_get_level(GPIO_NUM_0) ^ 0x01;
    return button_2_status << 1U | button_1_status ;
}

static void change_state(State next_state)
{
    xSemaphoreTake(statemachine_mutex, portMAX_DELAY);
    present_state = next_state;
    xSemaphoreGive(statemachine_mutex);
}