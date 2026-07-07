// Standard libraries
#include <stdlib.h>
#include <stdio.h>
#include <cstdio>

// ESP32 libraries
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"       // called gptimer.h on espidf 5.x and timer.h on 4.x
#include "driver/gpio.h"

// Arduino libraries
#include <Arduino.h>
#include <SimpleFOC.h>

// Custom system libraries
#include "Gimbal.h"
#include "SpeedControl.h"
#include "Motor.h"
#include "AS5048AEncoder.h"
#include "LSM6DSV16X_IMU.h"
#include "SPIBus.h"

// Pin definitions
#define PIN_SPI_MOSI                35
#define PIN_SPI_MISO                37
#define PIN_SPI_SCLK                36
#define PIN_SPI_IMU_CS              34
#define PIN_SPI_ENC_YAW_CS          9
#define PIN_SPI_ENC_PITCH_CS        8
#define PIN_SPI_ENC_ROLL_CS         7
#define PIN_MOTOR_YAW_PWM_1         10
#define PIN_MOTOR_YAW_PWM_2         11
#define PIN_MOTOR_YAW_PWM_3         12
#define PIN_MOTOR_PITCH_PWM_1       13
#define PIN_MOTOR_PITCH_PWM_2       14
#define PIN_MOTOR_PITCH_PWM_3       15
#define PIN_MOTOR_ROLL_PWM_1        16
#define PIN_MOTOR_ROLL_PWM_2        17
#define PIN_MOTOR_ROLL_PWM_3        18

// Add BMS and power management stuff later
// #define 

// Initialisation of classes
SPIBus  spi_bus_2(SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCLK);
LSM6DSV16X_IMU imu(PIN_SPI_IMU_CS);
AS5048AEncoder yaw_enc(PIN_SPI_ENC_YAW_CS);
// AS5048AEncoder pitch_enc(PIN_SPI_ENC_PITCH_CS);
// AS5048AEncoder roll_enc(PIN_SPI_ENC_ROLL_CS);
Motor yaw_motor(PIN_MOTOR_YAW_PWM_1, PIN_MOTOR_YAW_PWM_2, PIN_MOTOR_YAW_PWM_3);
// Motor pitch_motor(PIN_MOTOR_PITCH_PWM_1, PIN_MOTOR_PITCH_PWM_2, PIN_MOTOR_PITCH_PWM_3);
// Motor roll_motor(PIN_MOTOR_ROLL_PWM_1, PIN_MOTOR_ROLL_PWM_2, PIN_MOTOR_ROLL_PWM_3);
// Gimbal gimbal(&spi_bus_2, &imu, &yaw_enc, &pitch_enc, &roll_enc);
SpeedControl speed_control(&spi_bus_2, &yaw_enc);

// Interrupt setup
// volatile bool control_flag = false;
static TaskHandle_t x_task_to_notify;
volatile uint32_t control_ticks = 0;

// bool IRAM_ATTR control_timer_isr(void *arg)
// {
//     control_ticks++;

//     return false;
// }

bool IRAM_ATTR control_timer_isr(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(x_task_to_notify, &higher_priority_task_woken);

    return false;
}

void control_timer_setup(int control_loop_frequency, uint32_t divider, timer_group_t group, timer_idx_t timer, timer_autoreload_t auto_reload) 
{
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = auto_reload,
        .divider = divider,
    };

    // Initialise timer
    timer_init(group, timer, &config);

    // Set auto-reload value
    timer_set_counter_value(group, timer, 0);

    // Calculate alarm value depending on target control loop frequency
    float N = TIMER_BASE_CLK / (divider * control_loop_frequency);
    int N_floor = floor(N);
    if (N - N_floor != 0) {
        printf("\nN = %f is not an integer and has been rounded to %d.\n", N, N_floor);
        float new_control_loop_frequency = N * divider / TIMER_BASE_CLK;
        printf("This leads to an adjusted control loop frequency of %f.", new_control_loop_frequency);
    }

    // Set alarm value
    timer_set_alarm_value(group, timer, N_floor);

    // Enable interrupt
    timer_enable_intr(group, timer);

    // Link callback function to ISR (func is the callback function)
    // timer_isr_register(group, timer, func, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_isr_callback_add(group, timer, control_timer_isr, NULL, ESP_INTR_FLAG_IRAM);

    // Start the timer
    timer_start(group, timer);

}

void gpio_setup(int pin, gpio_mode_t io_mode) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = io_mode,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
}

// -------- CAN OPTIMISE INTEGER SIZES LATER!!! --------

// Main
extern "C" void app_main(void) 
{
    // Initialise Arduino for SimpleFOC
    initArduino();

    // Setup interrupt timer
    x_task_to_notify = xTaskGetCurrentTaskHandle();
    int control_loop_frequency = 1;                         // Hz
    uint32_t divider = 80;
    timer_group_t group = TIMER_GROUP_0;
    timer_idx_t timer = TIMER_0;
    timer_autoreload_t auto_reload = TIMER_AUTORELOAD_EN;    
    control_timer_setup(control_loop_frequency, divider, group, timer, auto_reload);

    // Setup GPIO
    const gpio_num_t led_pin = GPIO_NUM_15;
    gpio_setup(led_pin, GPIO_MODE_OUTPUT);

    // 2. Call setup and capture the result
    esp_err_t err = speed_control.setup();

    // 3. Evaluate the result
    if (err != ESP_OK) {
        // Use the built-in error-to-string helper for debugging
        printf("Speed Control Setup FAILED: %s (0x%X)\n", esp_err_to_name(err), err);
        
        // Safety: Don't start the PID loop if the IMU or Encoders are offline
        // abort();
    }
    else {
        printf("%d\n", err);
    }

    // Interrupt triggers execution of control loop code inside the main loop
    // rather than everything inside the ISR.
    bool led_on = false;

    while (true) {
        // No task handle necessary because it is only ever waiting on one task at a time
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Update the state of the system
        speed_control.update();

        // Test by blinking an LED
        led_on = !led_on;
        if (led_on) {
            gpio_set_level(led_pin, 1);     // Set GPIO high
        }
        else {
            gpio_set_level(led_pin, 0);     // Set GPIO low
        }
    }
}

    // while (true) {
    //     // control_ticks will count the number of missed events
    //     if (control_ticks > 0) {
    //         control_ticks--;

    //         // Update the state of the system
    //         speed_control.update();

    //         // Test by blinking an LED
    //         led_on = !led_on;
    //         if (led_on) {
    //             gpio_set_level(led_pin, 1);     // Set GPIO high
    //         }
    //         else {
    //             gpio_set_level(led_pin, 0);     // Set GPIO low
    //         }
    //     }
    // }