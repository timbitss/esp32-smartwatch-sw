#include <cstdio>
#include "esp_log.h"
#include "soc/soc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp32_smartwatch.h"
#include "Buzzer.h"
#include "ButtonDebouncer.h"
#include "I2C.h"
#include "RTClib.h"
#include "bma423.h"
#include "bma4_common.h"
#include "lvgl.h"
#include "lvgl_helpers.h"
#include "LVGL_App.h"

extern "C" void app_main(void);

enum State : uint8_t;

/**
 * Private function prototypes.
 */
static uint8_t io_read_buttons();
void RunTask1Hz(void *parameters);
void RunTask200Hz(void * parameters);
static void change_state(State next_state);
static void configure_wakeup_sources();
static void initialize_bma423();
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);

/**
 * Unique component tag for logging data.
 */
static const char *TAG = "APP_MAIN";

/**
 * Mutex guarding transition of states.
 */
SemaphoreHandle_t statemachine_mutex;

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

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
 *
 * @note Stored in RTC retention memory in case the RTC alarm wakes up the device from deep sleep.
 */
RTC_DATA_ATTR static bool alarm_enabled;

/**
 * Global structure acting as an interface to various peripherals.
 */
struct Peripherals_t
{
    Buzzer* buzzer;
    RTC_DS3231* rtc;
    ButtonDebouncer* buttonDebouncer;
    I2C* i2c_bus;
    struct bma4_dev bma;
} peripherals;

/**
 * Time that device was inactive without any button presses or wrist tilt.
 */
static uint32_t time_inactive_ms;

void app_main(void)
{
    /* Initialize I2C bus. */
    static I2C i2c_bus(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000, false);
    peripherals.i2c_bus = &i2c_bus;

    /* Initialize RTC. */
    static RTC_DS3231 rtc{};
    peripherals.rtc = &rtc;

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

    /* Initialize button debouncer and button inputs. */
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

    uint8_t switch_bitmask = BUTTONS_BITMASK; // Using two switches.
    static ButtonDebouncer buttonDebouncer(io_read_buttons, switch_bitmask);
    peripherals.buttonDebouncer = &buttonDebouncer;

    /* Initialize buzzer. */
    static Buzzer buzzer(LEDC_TIMER_0, 4000, GPIO_NUM_33, LEDC_CHANNEL_0);
    peripherals.buzzer = &buzzer;

    /* Initialize accelerometer */
    initialize_bma423();

    /* Initialize state machine mutex. */
    statemachine_mutex = xSemaphoreCreateMutex();

    /* Disable alarm. */
    if(!alarm_enabled)
    {
        peripherals.rtc->disableAlarm1Interrupt();
    }

    rtc.setAlarm1(DateTime(__DATE__, __TIME__) - TimeSpan(1), DS3231_A1_Hour);

    /* Create tasks. */
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

    /* If you want to use a task to create the graphic, you NEED to create a Pinned task
  * Otherwise there can be problem such as memory corruption and so on.
  * NOTE: When not using Wi-Fi nor Bluetooth you can pin the guiTask to core 0 */
    xTaskCreatePinnedToCore(guiTask, "gui", 4096*2, NULL, 0, NULL, 1);

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
        /* Do something if alarm was fired. */
        if(peripherals.rtc->alarmFired(ALARM1))
        {
            if(alarm_enabled)
            {
                present_state = ALARM_FIRED;
            }
            else
            {
                peripherals.rtc->clearAlarm(ALARM1);
            }
        }

        if(present_state == ALARM_FIRED)
        {
            if(buzzer_flag & 0x01) peripherals.buzzer->TurnOnBuzzer();
            else                   peripherals.buzzer->TurnOffBuzzer();
            buzzer_flag ^= 0x01;
        }

        /* Display various information to the user. */
        peripherals.rtc->now().timestamp(current_time);
        peripherals.rtc->getAlarm1().timestamp(alarm_time, DateTime::TIMESTAMP_TIME);
        ESP_LOGI(TAG, "(1Hz) Time: %s Alarm: %s State: %s Alarm %s Time released: %u",
                 current_time,
                 alarm_time,
                 state_names[present_state],
                 alarm_enabled ? "enabled" : "disabled",
                 time_inactive_ms);

        /* Enter deep-sleep if inactive for longer than INACTIVE_TIME_MS and alarm has been cleared. */
        if(time_inactive_ms >= INACTIVE_TIME_MS && present_state != ALARM_FIRED)
        {
            configure_wakeup_sources();
            ESP_LOGI(TAG, "Going to sleep");
            esp_deep_sleep_start();
        }

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
    uint16_t bma423_int_status = 0;

    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        peripherals.buttonDebouncer->ProcessSwitches200Hz(&button_states, &press_detected, &press_released);

        /* Do something if button is held for more than LONG_PRESS_DURATION_MS. */
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
                if(alarm_enabled)
                {
                    peripherals.rtc->enableAlarm1Interrupt();
                }
                else
                {
                    peripherals.rtc->disableAlarm1Interrupt();
                }
                ESP_LOGI(TAG, "Button 1 long press");
            }
        }

        /**
         * If all buttons are open, increment time released by 5 ms.
         */
        if ((button_states & BUTTONS_BITMASK) == 0x00)
        {
            time_inactive_ms += 5;
        }

        /* Check if wrist wear or single tap interrupt was triggered. */
        bma423_read_int_status(&bma423_int_status, &peripherals.bma);

        /**
         * Reset inactive time if any activity was detected.
         */
        if (press_detected || bma423_int_status & (BMA423_SINGLE_TAP_INT | BMA423_WRIST_WEAR_INT))
        {
            time_inactive_ms = 0;
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

/**
 * @brief Configure wakeup sources from deep sleep.
 */
static void configure_wakeup_sources()
{
    /* Wake up when RTC alarm is triggered. */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    gpio_pulldown_dis(RTC_INTERRUPT_PIN);
    gpio_pullup_en(RTC_INTERRUPT_PIN);
    esp_sleep_enable_ext0_wakeup(RTC_INTERRUPT_PIN, 0);

    /* Wake up when device is tapped or wrist is tilted. */
    esp_sleep_enable_ext1_wakeup(BMA423_INT1_PIN_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
}

static void initialize_bma423()
{
    struct bma4_accel_config accel_conf{};
    int8_t rslt;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated
     * Variant information given as parameter
     *         For B variant        : BMA42X_B_VARIANT
     *         For Non-B variant    : BMA42X_VARIANT
     */
    rslt = bma4_interface_selection(&peripherals.bma, BMA42X_VARIANT, peripherals.i2c_bus);
    bma4_error_codes_print_result("bma4_interface_selection status", rslt);

    /* Sensor initialization */
    rslt = bma423_init(&peripherals.bma);
    bma4_error_codes_print_result("bma423_init", rslt);

    if(esp_reset_reason() == ESP_RST_POWERON)
    {
        /* Upload the configuration file to enable the features of the sensor. */
        rslt = bma423_write_config_file(&peripherals.bma);
        bma4_error_codes_print_result("bma423_write_config", rslt);

        /* Enable the accelerometer */
        rslt = bma4_set_accel_enable(1, &peripherals.bma);
        bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

        /* Accelerometer Configuration Setting */
        /* Output data Rate */
        accel_conf.odr = BMA4_OUTPUT_DATA_RATE_200HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
        accel_conf.range = BMA4_ACCEL_RANGE_2G;

        /* Bandwidth configure number of sensor samples required to average
         * if value = 2, then 4 samples are averaged
         * averaged samples = 2^(val(accel bandwidth))
         * Note1 : More info refer datasheets
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but since the
         * performance power mode phase is increased, the power consumption will also rise.
         */
        accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Averaging samples (Default)
         *  1 -> No averaging
         * For more info on No Averaging mode refer datasheets.
         */
        accel_conf.perf_mode = BMA4_CIC_AVG_MODE;

        /* Set the accel configurations */
        rslt = bma4_set_accel_config(&accel_conf, &peripherals.bma);
        bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

        /* Enable single tap and wrist wear feature */
        rslt = bma423_feature_enable(BMA423_SINGLE_TAP | BMA423_WRIST_WEAR, BMA4_ENABLE, &peripherals.bma);
        bma4_error_codes_print_result("bma423_feature_enable status", rslt);

        /* Mapping line interrupt 1 with that of wrist wear and single tap feature interrupts. */
        rslt = bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_SINGLE_TAP_INT | BMA423_WRIST_WEAR_INT,
                                    BMA4_ENABLE, &peripherals.bma);
        bma4_error_codes_print_result("bma423_map_interrupt", rslt);

        /* Enable output for INT1 pin (active-low, push-pull) */
        bma4_int_pin_config bma4IntPinConfig{};
        bma4IntPinConfig.output_en = 0xFF;
        bma4_set_int_pin_config(&bma4IntPinConfig, BMA4_INTR1_MAP, &peripherals.bma);

        /* Flip y and x-axis for wrist wear feature. */
        bma423_axes_remap bma423AxesRemap{};
        bma423AxesRemap.x_axis = 1;
        bma423AxesRemap.y_axis = 0;
        bma423AxesRemap.z_axis = 2;
        bma423_set_remap_axes(&bma423AxesRemap, &peripherals.bma);

        /* Adjust sensitivity of single tap feature */
        bma423_single_tap_set_sensitivity(0x05, &peripherals.bma);

        ESP_LOGI(TAG, "BMA423 Initialized");
    }
}

static void guiTask(void *pvParameter) {

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t* buf1 = static_cast<lv_color_t *>(heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t),
                                                                  MALLOC_CAP_DMA));
    assert(buf1 != nullptr);

    /* Use double buffered when not working with monochrome displays */
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    lv_color_t* buf2 = static_cast<lv_color_t *>(heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t),
                                                                  MALLOC_CAP_DMA));
    assert(buf2 != nullptr);
#else
    static lv_color_t *buf2 = NULL;
#endif

    static lv_disp_draw_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820         \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A    \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D     \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306

    /* Actual size in pixels, not bytes. */
    size_in_px *= 8;
#endif

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

    /* When using a monochrome display we need to register the callbacks:
     * - rounder_cb
     * - set_px_cb */
#ifdef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb = disp_driver_set_px;
#endif

    disp_drv.draw_buf = &disp_buf;
    disp_drv.ver_res = 240;
    disp_drv.hor_res = 240;
    lv_disp_drv_register(&disp_drv);

    /* Register an input device when enabled on the menuconfig */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
#endif

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &lv_tick_task,
            .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /* Create the application */
    LVGL_App();

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
    }

    /* A task should NEVER return */
    free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(nullptr);
}

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}
