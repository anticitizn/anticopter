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
#include "camera.h"
#include "led.h"

#define PORT 3333

// 64kb rx buffer
char rx_buffer[16000];
char *header = rx_buffer;
char *payload = rx_buffer + 128;

// Function to format the data into a string
void format_data(char *buffer, int16_t *acceleration, int16_t *angular_rate, int16_t temperature) {
    // Assuming the format "Acceleration[X,Y,Z];AngularRate[X,Y,Z];Temperature[T];"
    sprintf(buffer, "\nAcceleration[%d,%d,%d]\nAngularRate[%d,%d,%d]\nTemperature[%d]\n",
            acceleration[0], acceleration[1], acceleration[2],
            angular_rate[0], angular_rate[1], angular_rate[2],
            temperature);
}

char data_buffer[256] = {0};
bool connected = 0;
int sock;
struct sockaddr_storage source_addr;

void udp_camera_send_data()
{
    ESP_LOGI(TAG, "Taking a photo!");
    cam_take_picture();

    sendto(sock, _jpg_buf, _jpg_buf_len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
    
    vTaskDelete(NULL);
}

static void udp_imu_send_task()
{
    printf("Sending udp data!\n");
    format_data(data_buffer, data_raw_acceleration, data_raw_angular_rate, data_raw_temperature);
    sendto(sock, data_buffer, strlen(data_buffer), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
    vTaskDelay(20 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);
    
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
                
                if (strcmp("get_camera", header) == 0)
                {
                    xTaskCreate(udp_camera_send_data, "udp_send_task", 4096, NULL, 5, NULL);
                }
                else if (strcmp("get_imu", header) == 0)
                {
                    xTaskCreate(udp_imu_send_task, "udp_send_task", 4096, NULL, 5, NULL);
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