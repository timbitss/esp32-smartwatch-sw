#include <cstdio>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp32_smartwatch.h"
#include "Buzzer.h"
#include "ButtonDebouncer.h"
#include "I2C.h"
#include "RTClib.h"

extern "C" void app_main(void);

enum State : uint8_t;

/**
 * Private function prototypes.
 */
static uint8_t io_read_buttons();
void RunTask1Hz(void *parameters);
void RunTask200Hz(void * parameters);
static void change_state(State next_state);

/**
 * Unique component tag for logging data.
 */
static const char *TAG = "APP_MAIN";

/**
 * Mutex guarding transition of states.
 */
SemaphoreHandle_t statemachine_mutex;

/**
 * State machine.
 */
enum State : uint8_t
{
    NORMAL = 0,    // Pressing a button adjusts the current time.
    ALARM_ADJUST,  // Pressing a button adjusts the alarm time.
    ALARM_FIRED,   // Pressing a button clears the alarm flag and turns off the buzzer.
    NUM_OF_STATES
} present_state;

/**
 * Flag indicating if the alarm is enabled.
 */
static bool alarm_enabled;

/**
 * Global structure acting as an interface to various peripherals.
 */
struct Peripherals_t
{
    Buzzer* buzzer;
    RTC_DS3231* rtc;
    ButtonDebouncer* buttonDebouncer;
} peripherals;

void app_main(void)
{
    /**
     * Initialize I2C bus.
     */
    static I2C i2c_bus(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000, false);

    /**
     * Initialize RTC.
     */
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

    /**
     * Set alarm time to arbitrary value.
     */
    rtc.setAlarm1(rtc.now() - TimeSpan(60), DS3231_A1_Hour);

    /**
     * Initialize button debouncer and button inputs.
     */
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

    uint8_t switch_bitmask = 0x3; // Using two switches.
    static ButtonDebouncer buttonDebouncer(io_read_buttons, switch_bitmask);

    /**
     * Initialize buzzer.
     */
    static Buzzer buzzer(LEDC_TIMER_0, 4000, GPIO_NUM_33, LEDC_CHANNEL_0);

    /**
     * Create interface from statically-allocated objects.
     */
    peripherals.buzzer = &buzzer;
    peripherals.rtc = &rtc;
    peripherals.buttonDebouncer = &buttonDebouncer;

    // Initialize state machine mutex.
    statemachine_mutex = xSemaphoreCreateMutex();

    /**
     * Create tasks.
     */
    xTaskCreatePinnedToCore(RunTask1Hz,
                            "RunTask1Hz",
                            TASK1HZ_STACK_SIZE,
                            nullptr,
                            2,
                            nullptr,
                            APP_CPU_NUM);


    xTaskCreatePinnedToCore(RunTask200Hz,
    "RunTask200Hz",
    TASK200HZ_STACK_SIZE,
    nullptr,
    1,
    nullptr,
    APP_CPU_NUM);

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS((100000)));
    }
}

void RunTask1Hz(void *parameters)
{
    TickType_t lastWakeTime;
    const TickType_t period_ms = pdMS_TO_TICKS( 1000 );
    char current_time[25] = {}, alarm_time[25] = {};
    static const char* state_names[NUM_OF_STATES] = {"NORMAL", "ALARM_ADJUST", "ALARM_FIRED"};
    uint8_t buzzer_flag = 0;

    lastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        /**
         *  Do something if alarm was fired.
         */
        if(peripherals.rtc->alarmFired(ALARM1))
        {
            if(alarm_enabled)
            {
                if(buzzer_flag & 0x01) peripherals.buzzer->TurnOnBuzzer();
                else                   peripherals.buzzer->TurnOffBuzzer();
                buzzer_flag ^= 0x01;
                present_state = ALARM_FIRED;
            }
            else
            {
                peripherals.rtc->clearAlarm(ALARM1);
            }
        }

        /**
         * Display various information to the user.
         */
        peripherals.rtc->now().timestamp(current_time);
        peripherals.rtc->getAlarm1().timestamp(alarm_time, DateTime::TIMESTAMP_TIME);
        ESP_LOGI(TAG, "(1Hz) Time: %s Alarm: %s State: %s Alarm %s",
                 current_time,
                 alarm_time,
                 state_names[present_state],
                 alarm_enabled ? "enabled" : "disabled");

        vTaskDelayUntil(&lastWakeTime, period_ms);
    }
}

void RunTask200Hz(void *parameters)
{
    TickType_t lastWakeTime;
    const TickType_t period_ms = pdMS_TO_TICKS( 5 );

    uint8_t button_states = 0;
    uint8_t press_detected = 0, press_released = 0;
    uint32_t timeheld0_ms = 0, timeheld1_ms = 0;    // Time buttons were held for in ms.

    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        peripherals.buttonDebouncer->ProcessSwitches200Hz(&button_states, &press_detected, &press_released);

        /**
         * Do something if button is held for more than LONG_PRESS_DURATION_MS.
         */
        if (button_states & BUTTON0_BITMASK)
        {
            timeheld0_ms += 5;
            if (timeheld0_ms == LONG_PRESS_DURATION_MS)
            {
                change_state(present_state == NORMAL ? ALARM_ADJUST : NORMAL);
                ESP_LOGI(TAG, "Button 0 long press");
            }
        }
        if (button_states & BUTTON1_BITMASK)
        {
            timeheld1_ms += 5;
            if (timeheld1_ms == LONG_PRESS_DURATION_MS)
            {
                alarm_enabled = alarm_enabled ? false : true;
                ESP_LOGI(TAG, "Button 1 long press");
            }
        }

        /**
         * Do something when a button is released if it was held for less than LONG_PRESS_DURATION_MS.
         */
        if (press_released & BUTTON0_BITMASK)
        {
            if (timeheld0_ms < LONG_PRESS_DURATION_MS)
            {
                switch (present_state)
                {
                    case NORMAL: {
                        peripherals.rtc->incrementMinute();
                        DateTime now = peripherals.rtc->now();
                        ESP_LOGI(TAG, "200Hz: %s", now.timestamp());
                        break;
                    }
                    case ALARM_ADJUST: {
                        peripherals.rtc->incrementAlarm1Minute();
                        DateTime alarm1 = peripherals.rtc->getAlarm1();
                        ESP_LOGI(TAG, "200Hz: Alarm 1 = %s", alarm1.timestamp(DateTime::TIMESTAMP_TIME));
                        break;
                    }
                    case ALARM_FIRED: {
                        peripherals.rtc->clearAlarm(ALARM1);
                        peripherals.buzzer->TurnOffBuzzer();
                        ESP_LOGI(TAG, "Alarm 1 cleared");
                        change_state(NORMAL);
                        break;
                    }
                    default: {
                        ESP_LOGW(TAG, "Unknown state");
                        break;
                    }
                }
            }

            ESP_LOGI(TAG, "Button 0 released");
            timeheld0_ms = 0;
        }
        if (press_released & BUTTON1_BITMASK)
        {
            if (timeheld1_ms < LONG_PRESS_DURATION_MS)
            {
                switch(present_state)
                {
                    case NORMAL: {
                        peripherals.rtc->decrementMinute();
                        DateTime now = peripherals.rtc->now();
                        ESP_LOGI(TAG, "200Hz: %s", now.timestamp());
                        break;
                    }
                    case ALARM_ADJUST: {
                        peripherals.rtc->decrementAlarm1Minute();
                        DateTime alarm1 = peripherals.rtc->getAlarm1();
                        ESP_LOGI(TAG, "200Hz: Alarm 1 = %s", alarm1.timestamp(DateTime::TIMESTAMP_TIME));
                        break;
                    }
                    case ALARM_FIRED: {
                        peripherals.rtc->clearAlarm(ALARM1);
                        peripherals.buzzer->TurnOffBuzzer();
                        ESP_LOGI(TAG, "Alarm 1 cleared");
                        change_state(NORMAL);
                        break;
                    }
                    default: {
                        ESP_LOGW(TAG, "Unknown state");
                        break;
                    }
                }
            }

            ESP_LOGI(TAG, "Button 1 released");
            timeheld1_ms = 0;
        }

        vTaskDelayUntil(&lastWakeTime, period_ms);
    }
}

/**
 * @brief Read status of buttons 1 and 0.
 *
 * @return 1 in bit position x if button x is closed (active-low) and vice-versa.
 */
static uint8_t io_read_buttons()
{
    uint8_t button_0_status = (uint8_t)gpio_get_level(BUTTON0_GPIO_PIN) ^ 0x01;
    uint8_t button_1_status = (uint8_t)gpio_get_level(BUTTON1_GPIO_PIN) ^ 0x01;
    return button_1_status << 1U | button_0_status;
}

/**
 * @brief Change state of watch.
 *
 * @param next_state State to change to.
 */
static void change_state(State next_state)
{
    xSemaphoreTake(statemachine_mutex, portMAX_DELAY);
    present_state = next_state;
    xSemaphoreGive(statemachine_mutex);
}