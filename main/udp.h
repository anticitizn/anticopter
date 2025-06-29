#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <string.h>
#include <sys/param.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "imu.h"
#include "pwm_control.h"
//#include "camera.h"
#include "led.h"

#define PORT 3333

typedef struct
{
    char imu_data[256];
    uint8_t cam_data[128000];
} telemetry;

telemetry data_out = {0};

// 16kb rx buffer
char rx_buffer[16000];
char *header = rx_buffer;
char *payload = rx_buffer + 128;

char data_buffer[256] = {0};
bool connected = 0;
int sock;
struct sockaddr_storage source_addr;

// Function to format the data into a string
void format_data(char *buffer, float *acceleration, float *angular_rate, float temperature, float *orientation) {
    // Assuming the format "Acceleration[X,Y,Z];AngularRate[X,Y,Z];Temperature[T];"
    sprintf(buffer, "\nAcceleration[%f,%f,%f]\nAngularRate[%f,%f,%f]\nOrientation[%f,%f,%f]\nTemperature[%f]\n",
            acceleration[0], acceleration[1], acceleration[2],
            angular_rate[0], angular_rate[1], angular_rate[2],
            orientation[0], orientation[1], orientation[2],
            temperature);
}

static void udp_server_task(void *pvParameters)
{
    char addr_str[128];

    int addr_family = AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1)
    {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;

        sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        socklen_t socklen = sizeof(source_addr);

        while (1)
        {
            ESP_LOGI(TAG, "Waiting for data");
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            connected = true;

            // Error occurred during receiving
            if (len < 0)
            {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                connected = false;
                break;
            }
            // Data received
            else
            {
                // Get the sender's ip address as string
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);

                header[127] = 0; // The header should already be null-terminated but just in case
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "Header: %s", header);
                ESP_LOGI(TAG, "Payload: %s", payload);
                
                if (strcmp("get_telemetry", header) == 0)
                {
                    format_data(data_out.imu_data, acceleration_mg, angular_rate_mdps, temperature_degC, orientation);
                    memcpy(data_out.cam_data, _jpg_buf, _jpg_buf_len);
                    sendto(sock, &data_out, sizeof(data_out), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                }
                else if (strcmp("get_camera", header) == 0)
                {
                    int64_t start_time = esp_timer_get_time(); // Get the start time in microseconds

                    // The code block you want to measure
                    sendto(sock, _jpg_buf, _jpg_buf_len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));

                    int64_t end_time = esp_timer_get_time(); // Get the end time in microseconds

                    // Calculate the elapsed time in milliseconds
                    double elapsed_time_ms = (double)(end_time - start_time) / 1000;

                    // Print the elapsed time
                    printf("Time taken for execution: %lf ms\n", elapsed_time_ms);
                    printf("Size: %d b\n", _jpg_buf_len);
                }
                else if (strcmp("get_imu", header) == 0)
                {
                    sendto(sock, data_buffer, strlen(data_buffer), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                }                
                else if (strcmp("set_led", header) == 0)
                {
                    int led_num, r, g, b;

                    if (sscanf(payload, "%d %d %d %d", &led_num, &r, &g, &b) == 4) 
                    {
                        if (led_num == 4)
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                set_led(i, r, g, b);
                            }
                        }
                        else
                        {
                            set_led(led_num, r, g, b);
                        }
                        
                        ESP_LOGI(TAG, "LED# %d | RGB: %d, %d, %d", led_num, r, g, b);
                    } 
                    else 
                    {
                        ESP_LOGI(TAG, "Failed to extract LED data from the payload!");
                    }
                }
                else if (strcmp("set_res_vga", header) == 0)
                {
                    printf("Setting RES to VGA!\n\n");
                    sensor_t *sensor = esp_camera_sensor_get();
                    sensor->set_framesize(sensor, FRAMESIZE_VGA);
                }
                else if (strcmp("set_res_hd", header) == 0)
                {
                    sensor_t *sensor = esp_camera_sensor_get();
                    sensor->set_framesize(sensor, FRAMESIZE_HD);
                }
                else if (strcmp("set_motors", header) == 0)
                {
                    int mot0, mot1, mot2, mot3;

                    if (sscanf(payload, "%d %d %d %d", &mot0, &mot1, &mot2, &mot3) == 4)
                    {
                        motor_pwm(0, mot0);
                        motor_pwm(1, mot1);
                        motor_pwm(2, mot2);
                        motor_pwm(3, mot3);
                        ESP_LOGI(TAG, "MOTORS: %d, %d, %d, %d", mot0, mot1, mot2, mot3);
                    } 
                    else 
                    {
                        ESP_LOGI(TAG, "Failed to extract motor data from the payload!");
                        ESP_LOGI(TAG, "PAYLOAD: %s", payload);
                    }
                }
            }

            memset(rx_buffer, 0, 128);
        }
        
        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            connected = false;
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}