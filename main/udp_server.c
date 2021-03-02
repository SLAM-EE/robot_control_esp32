/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define PORT CONFIG_EXAMPLE_PORT

#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A, LEFT MOTOR
#define GPIO_PWM0B_OUT 16   //Set GPIO 16 as PWM0B, RIGHTT MOTOR

#define GPIO_OUTPUT_IO_1    2
#define GPIO_OUTPUT_IO_2    4
#define GPIO_OUTPUT_IO_3    17
#define GPIO_OUTPUT_IO_4    5
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<GPIO_OUTPUT_IO_2) | (1ULL<<GPIO_OUTPUT_IO_3) | (1ULL<<GPIO_OUTPUT_IO_4)) //MASK BITS

static const char *TAG = "example";

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

static void left_motor_duty_cycle(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    //mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void right_motor_duty_cycle(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    //mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

static void left_motor_stop()
{
	//mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
	gpio_set_level(GPIO_OUTPUT_IO_1, 0);
	gpio_set_level(GPIO_OUTPUT_IO_2, 0);
}

static void right_motor_stop()
{
	//mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
	gpio_set_level(GPIO_OUTPUT_IO_3, 0);
	gpio_set_level(GPIO_OUTPUT_IO_4, 0);
}

static void both_motor_stop()
{
    left_motor_stop();
    right_motor_stop();
}

static void left_motor_forward()
{
	gpio_set_level(GPIO_OUTPUT_IO_1, 1);
	gpio_set_level(GPIO_OUTPUT_IO_2, 0);
}

static void left_motor_backward()
{
	gpio_set_level(GPIO_OUTPUT_IO_1, 0);
	gpio_set_level(GPIO_OUTPUT_IO_2, 1);
}

static void right_motor_forward()
{
	gpio_set_level(GPIO_OUTPUT_IO_3, 1);
	gpio_set_level(GPIO_OUTPUT_IO_4, 0);
}

static void right_motor_backward()
{
	gpio_set_level(GPIO_OUTPUT_IO_3, 0);
	gpio_set_level(GPIO_OUTPUT_IO_4, 1);
}

static void both_motor_forward()
{
	left_motor_forward();
	right_motor_forward();
}

static void both_motor_backward()
{
	left_motor_backward();
	right_motor_backward();
}

static void left_turn()
{
	left_motor_stop();
	right_motor_forward();
}

static void right_turn()
{
	left_motor_forward();
	right_motor_stop();
}

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
                switch(rx_buffer[0])
                {
                //forward
                case 'w':
                	both_motor_forward();
                	break;
                case 's':
                	both_motor_backward();
                	break;
                case 'd':
                	left_turn();
                	break;
                case 'a':
                	right_turn();
                	break;
                default:
                	both_motor_stop();
                	break;
                }

                left_motor_duty_cycle(MCPWM_UNIT_0, MCPWM_TIMER_0, 100);
                right_motor_duty_cycle(MCPWM_UNIT_0, MCPWM_TIMER_0, 100);

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

/*
static void brushed_motor_control()
{

    while (1) {
    	left_motor_duty_cycle(MCPWM_UNIT_0, MCPWM_TIMER_0, 80.0);
        right_motor_duty_cycle(MCPWM_UNIT_0, MCPWM_TIMER_0, 40.0);

        both_motor_forward();
        vTaskDelay(2000 / portTICK_RATE_MS);
        both_motor_backward();
        vTaskDelay(2000 / portTICK_RATE_MS);
        left_turn(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_RATE_MS);
        right_turn(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_RATE_MS);
        both_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_RATE_MS);

        left_motor_duty_cycle(MCPWM_UNIT_0, MCPWM_TIMER_0, 80.0);
        right_motor_duty_cycle(MCPWM_UNIT_0, MCPWM_TIMER_0, 40.0);
        vTaskDelay(2000 / portTICK_RATE_MS);
        both_motor_forward();
        vTaskDelay(2000 / portTICK_RATE_MS);

    }
}
*/

static void motor_initialize()
{
		//1. mcpwm gpio initialization
	    mcpwm_example_gpio_initialize();

	    //2. initial mcpwm configuration
	    printf("Configuring Initial Parameters of mcpwm...\n");
	    mcpwm_config_t pwm_config;
	    pwm_config.frequency = 1000;    //frequency = 500Hz,
	    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
	    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
	    pwm_config.counter_mode = MCPWM_UP_COUNTER;
	    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

	    //3. Normal gpio initialization
	    gpio_config_t io_conf;
	    io_conf.intr_type = GPIO_INTR_DISABLE;		//disable interrupt
	    io_conf.mode = GPIO_MODE_OUTPUT;			//set as output mode
	    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;	//bit mask of the pins that you want to set,e.g.GPIO18/19
	    io_conf.pull_down_en = 0;					//disable pull-down mode
	    io_conf.pull_up_en = 0;						//disable pull-up mode
	    gpio_config(&io_conf);						//configure GPIO with the given settings

	    left_motor_duty_cycle(MCPWM_UNIT_0, MCPWM_TIMER_0, 35.0);
	    right_motor_duty_cycle(MCPWM_UNIT_0, MCPWM_TIMER_0, 35.0);

}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    motor_initialize();

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
#endif

}
