#include <cstdio>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp32_smartwatch.h"
#include "Buzzer.h"
#include "ButtonDebouncer.h"
#include "I2C.h"
#include "bma423.h"
#include "bma4_common.h"

/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH       (9.80665f)

/*! Macro that holds the total number of accel x,y and z axes sample counts to be printed */
#define ACCEL_SAMPLE_COUNT  100

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
} peripherals;

/**
 * Time that both buttons were released for.
 */
static uint32_t time_released_ms;
// Unique component tag for logging data.
static const char *TAG = "APP_MAIN";

static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);


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

    uint8_t switch_bitmask = BUTTONS_BITMASK; // Using two switches.
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

    /**
     * Initialize state machine mutex.
     */
    statemachine_mutex = xSemaphoreCreateMutex();

    /**
     * Disable alarm
     */
    if(!alarm_enabled)
    {
        peripherals.rtc->disableAlarm1Interrupt();
    }

    rtc.setAlarm1(DateTime(__DATE__, __TIME__) - TimeSpan(1), DS3231_A1_Hour);
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
    i2c_port_t port_num = I2C_NUM_0;
    i2c_config_t i2c_conf;
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.master.clk_speed = i2c_clk_speed;
    i2c_conf.scl_io_num = scl_io_num;
    i2c_conf.sda_io_num = sda_io_num;
    i2c_conf.scl_pullup_en = false;
    i2c_conf.sda_pullup_en = false;
    i2c_conf.clk_flags = 0;

    static I2C i2c_bus(port_num, &i2c_conf);

    struct bma4_dev bma{};
    struct bma423_any_no_mot_config any_no_mot{};
    struct bma4_accel_config accel_conf{};
    int8_t rslt;
    uint16_t int_status = 0;


    /* Function to select interface between SPI and I2C, according to that the device structure gets updated
     * Variant information given as parameter
     *         For B variant        : BMA42X_B_VARIANT
     *         For Non-B variant    : BMA42X_VARIANT
     */
    rslt = bma4_interface_selection(&bma, BMA42X_VARIANT, &i2c_bus);
    bma4_error_codes_print_result("bma4_interface_selection status", rslt);

    /* Sensor initialization */
    rslt = bma423_init(&bma);
    bma4_error_codes_print_result("bma423_init status", rslt);

void RunTask1Hz(void *parameters)
{
    TickType_t lastWakeTime;
    const TickType_t period_ms = pdMS_TO_TICKS( 1000 );
    char current_time[25] = {}, alarm_time[25] = {};
    static const char* state_names[NUM_OF_STATES] = {"NORMAL", "ALARM_ADJUST", "ALARM_FIRED"};
    uint8_t buzzer_flag = 0;
    /* Upload the configuration file to enable the features of the sensor. */
    rslt = bma423_write_config_file(&bma);
    bma4_error_codes_print_result("bma423_write_config status", rslt);

    /* Enable the accelerometer */
    rslt = bma4_set_accel_enable(1, &bma);
    bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

    for(;;)
    {
        /**
        *  Do something if alarm was fired.
        */
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

        /**
         * Display various information to the user.
         */
        peripherals.rtc->now().timestamp(current_time);
        peripherals.rtc->getAlarm1().timestamp(alarm_time, DateTime::TIMESTAMP_TIME);
        ESP_LOGI(TAG, "(1Hz) Time: %s Alarm: %s State: %s Alarm %s Time released: %u",
                 current_time,
                 alarm_time,
                 state_names[present_state],
                 alarm_enabled ? "enabled" : "disabled",
                 time_released_ms);

        /**
         * Enter deep-sleep if inactive for longer than INACTIVE_TIME_MS and alarm has been cleared.
         */
        if(time_released_ms >= INACTIVE_TIME_MS && present_state != ALARM_FIRED)
        {
            configure_wakeup_sources();
            ESP_LOGI(TAG, "Going to sleep");
            esp_deep_sleep_start();
        }
    /* Accelerometer Configuration Setting */
    /* Output data Rate */
    accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;

    /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
    accel_conf.range = BMA4_ACCEL_RANGE_2G;

void RunTask200Hz(void *parameters)
{
    TickType_t lastWakeTime;
    const TickType_t period_ms = pdMS_TO_TICKS( 5 );

    uint8_t button_states = 0;
    uint8_t press_detected = 0, press_released = 0;
    uint32_t timeheld0_ms = 0, timeheld1_ms = 0;    // Time buttons were held for in ms.
    /* Bandwidth configure number of sensor samples required to average
     * if value = 2, then 4 samples are averaged
     * averaged samples = 2^(val(accel bandwidth))
     * Note1 : More info refer datasheets
     * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
     * since the performance power mode phase is increased, the power consumption will also rise.
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
    rslt = bma4_set_accel_config(&accel_conf, &bma);
    bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

    struct bma423_axes_remap bma423AxesRemap{};
    bma423AxesRemap.x_axis = 1;
    bma423AxesRemap.y_axis = 0; // Flip x and y-axis.
    bma423AxesRemap.z_axis = 2;
    rslt = bma423_set_remap_axes(&bma423AxesRemap, &bma);
    bma4_error_codes_print_result("Remap axes", rslt);

    /* Enable wrist wear feature */
    rslt = bma423_feature_enable(BMA423_WRIST_WEAR, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma4_feature_enable status", rslt);

    rslt = bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_WRIST_WEAR_INT, BMA4_ENABLE, &bma);
    bma4_error_codes_print_result("bma423_map_interrupt", rslt);

    ESP_LOGI(TAG, "Move wrist");

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
            time_released_ms += 5;
        }

        /**
         * Reset time buttons were released if any button press is detected.
         */
        if (press_detected)
        {
            time_released_ms = 0;
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
        /* Read the interrupt register to check if wrist wear interrupt is received */
        bma423_read_int_status(&int_status, &bma);

        /* Check if any-motion interrupt is triggered */
        if (int_status & BMA423_WRIST_WEAR_INT)
        {
            ESP_LOGI(TAG, "Wrist wear interrupt received");
        }
        if (int_status & BMA423_ERROR_INT)
        {
            ESP_LOGI(TAG, "Error interrupt.");
        }
        int_status = 0;

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
    /* Wake up when button 0 is pressed */
    esp_sleep_enable_ext0_wakeup(BUTTON0_GPIO_PIN, 0);

    /* Wake up when alarm goes off */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    gpio_pulldown_dis(RTC_INTERRUPT_PIN);
    gpio_pullup_en(RTC_INTERRUPT_PIN);
    esp_sleep_enable_ext1_wakeup((1U << RTC_INTERRUPT_PIN), ESP_EXT1_WAKEUP_ALL_LOW);
}

/*! @brief Converts raw sensor values(LSB) to meters per seconds square.
 *
 *  @param[in] val      : Raw sensor value.
 *  @param[in] g_range  : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] bit_width    : Resolution of the sensor.
 *
 *  @return Accel values in meters per second square.
 *
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = (float)(1 << bit_width) / 2.0f;

    return GRAVITY_EARTH * val * g_range / half_scale;
}
