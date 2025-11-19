
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

// The higher the beta value, the faster the actual PWM value reaches the desired PWM
#define LPF_BETA_LOW 0.2   // This is used for current PWM values under 30
# define LPF_BETA_HIGH 0.5 // This is used for current PWM values equal to or over 30

float current_motor_pwm[4] = {0};
float desired_motor_pwm[4] = {0};

// Simple digital low-pass filter to prevent hard PWM jumps that cause hte inrush current to spike and the board to reset
float lpf_smooth(float current_value, float new_value, float beta)
{
    return current_value - (beta * (current_value - new_value));
}

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
static void motor_pwm(int motor_i, int duty_cycle)
{
    // Set the duty cycle
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + motor_i, duty_cycle * ((1 << LEDC_TIMER_13_BIT) - 1) / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_BASE + motor_i);
}

// Set the PWM value of all motors
// duty_cycle is in percentages, between 0 and 100
static void motors_pwm(int duty_cycle)
{
    for (int i = 0; i < 4; i++)
    {
        motor_pwm(i, duty_cycle);
    }
}

// This needs to be called in the main control loop
void motors_tick()
{
    for (int i = 0; i < 4; i++)
    {
        // On increasing PWM values, we run the PWM output through a digital low-pass filter to reduce the current inrush spike
        // When the PWM is decreasing instead, there's no need to do it, so we don't do it
        // (also it is useful to be able to stop all motors immediately)
        if (desired_motor_pwm[i] > current_motor_pwm[i])
        {
            float beta = current_motor_pwm[i] >= 30 ? LPF_BETA_HIGH : LPF_BETA_LOW;
            int smoothed_duty_cycle = (int)floor(lpf_smooth(current_motor_pwm[i], desired_motor_pwm[i], beta));
            motor_pwm(i, smoothed_duty_cycle);
            current_motor_pwm[i] = smoothed_duty_cycle;
        }
        else
        {
            motor_pwm(i, desired_motor_pwm[i]);
            current_motor_pwm[i] = desired_motor_pwm[i];
        }
        
        printf("Pwm | Current: %f | Desired: %f\n", current_motor_pwm[i], desired_motor_pwm[i]);
    }
}

void set_motor_pwm(int motor_i, int duty_cycle)
{
    desired_motor_pwm[motor_i] = duty_cycle;
}

// Spin each motor very briefly at low power in sequence
void motors_check()
{
    for (int i = 0; i < 4; i++)
    {
        // Set motor to 5% duty cycle
        motor_pwm(i, 2);

        // Wait 75 ms
        vTaskDelay(75 / portTICK_PERIOD_MS); 

        // Turn off the motor
        motor_pwm(i, 0);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
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
