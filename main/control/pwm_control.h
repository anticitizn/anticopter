
#ifndef ANTICOPTER_MOTORS
#define ANTICOPTER_MOTORS

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "../led.h"
#include "comms/msg_type.h"

#define MOTOR_GPIO_1 39
#define MOTOR_GPIO_2 40
#define MOTOR_GPIO_3 41
#define MOTOR_GPIO_4 42

#define PWM_CHANNEL_BASE LEDC_CHANNEL_1
#define PWM_TIMER_BASE LEDC_TIMER_1
#define PWM_FREQ_HZ 40000
#define PWM_RESOLUTION LEDC_TIMER_10_BIT

bool motors_armed = false;

float current_motor_pwm[4] = {0};

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

// Set the PWM value of one motor
// duty_cycle is in percentages, between 0 and 100
static void motor_pwm(int motor_i, float duty_cycle)
{
    if (motors_armed)
    {
        // Set the duty cycle
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + motor_i, (int)((duty_cycle * (1 << PWM_RESOLUTION)) / 100));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + motor_i);
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + i, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + i);
        }
    }
}

// Set the PWM value of all motors
// duty_cycle is in percentages, between 0 and 100
static void motors_pwm(float duty_cycle)
{
    for (int i = 0; i < 4; i++)
    {
        motor_pwm(i, duty_cycle);
    }
}

// Spin each motor very briefly at low power in sequence
void motors_check()
{
    for (int i = 0; i < 4; i++)
    {
        // Set motor to 5% duty cycle
        motor_pwm(i, 3);

        // Wait 75 ms
        vTaskDelay(75 / portTICK_PERIOD_MS); 

        // Turn off the motor
        motor_pwm(i, 0);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void handle_arm_msg(const void *payload)
{
    motors_armed = true;
    set_leds(0, 10, 0);
}

void handle_disarm_msg(const void *payload)
{
    motors_armed = false;
    set_leds(10, 0, 0);
}

void handle_motor_pwm_msg(const void *payload)
{
    msg_control_motor_pwm_t* msg = (msg_control_motor_pwm_t*)payload;

    memcpy(&motor_pwm, &msg->pwm, sizeof(msg->pwm));
}


#endif
