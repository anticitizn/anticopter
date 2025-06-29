
#ifndef ANTICOPTER_MOTORS
#define ANTICOPTER_MOTORS

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define MOTOR_GPIO_1 39
#define MOTOR_GPIO_2 40
#define MOTOR_GPIO_3 41
#define MOTOR_GPIO_4 42

#define PWM_CHANNEL_BASE LEDC_CHANNEL_1
#define PWM_TIMER_BASE LEDC_TIMER_1
#define PWM_FREQ_HZ 40000
#define PWM_RESOLUTION LEDC_TIMER_10_BIT

void setup_pwm()
{
    ledc_timer_config_t timer_conf = {
        .duty_resolution = PWM_RESOLUTION, .freq_hz = PWM_FREQ_HZ, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = PWM_TIMER_BASE};

    ledc_timer_config(&timer_conf);

    // Configure PWM channels for each motor
    for (int i = 0; i < 4; i++)
    {
        ledc_channel_config_t ledc_conf = {.channel = PWM_CHANNEL_BASE + i,
                                           .duty = 0,
                                           .gpio_num = MOTOR_GPIO_1 + i,
                                           .speed_mode = LEDC_LOW_SPEED_MODE,
                                           .timer_sel = PWM_TIMER_BASE};

        ledc_channel_config(&ledc_conf);
    }
}

void motors_check()
{
    for (int i = 0; i < 4; i++)
    {
        // Set duty cycle to 5%
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + i, 2 * ((1 << LEDC_TIMER_13_BIT) - 1) / 100);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + i);

        vTaskDelay(75 / portTICK_PERIOD_MS); // Delay for 75 ms

        // Turn off PWM
        ledc_stop(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + i, 0);
        vTaskDelay(600 / portTICK_PERIOD_MS);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
}

void motors_off()
{
    for (int i = 0; i < 4; i++)
    {
        // Turn off PWM
        ledc_stop(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + i, 0);
    }
}

// duty_cycle is in percentages, between 0 and 100
void motors_pwm(int duty_cycle)
{
    for (int i = 0; i < 4; i++)
    {
        // Set duty cycle
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + i, duty_cycle * ((1 << LEDC_TIMER_13_BIT) - 1) / 100);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + i);

        //vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// duty_cycle is in percentages, between 0 and 100
void motor_pwm(int motor, int duty_cycle)
{
    // Set duty cycle
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + motor, duty_cycle * ((1 << LEDC_TIMER_13_BIT) - 1) / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + motor);

    //vTaskDelay(10 / portTICK_PERIOD_MS);
}

void pwm_motors_main()
{
    setup_pwm();

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 75 ms

    motors_check();

    for (int i = 0; i < 100; i++)
    {
        motors_pwm(i);
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    for (int i = 0; i < 4; i++)
    {
        // Turn off PWM
        ledc_stop(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + i, 0);
    }

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    motors_check();

    vTaskDelete(NULL); // This task does not need to run continuously
}

#endif
