#include <string.h>
#include <stdio.h>

#include "neopixel.h"
#include "nvs-manager.h"
#include "wifi.h"
#include "config-server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"

#include "bno08x.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/header.h>
#include <lar_msgs/msg/car_control_stamped.h>
#include <lar_msgs/msg/imu_stamped.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

static const char *TAG = "MAIN";

#define STRING_BUFFER_LEN 50

#define GPIO_NUM_LED   GPIO_NUM_13
#define GPIO_NUM_SERVO GPIO_NUM_33
#define GPIO_NUM_MOTOR GPIO_NUM_27

#define GPIO_NUM_NEOPIXEL       GPIO_NUM_0
#define GPIO_NUM_NEOPIXEL_POWER GPIO_NUM_2
#define NEOPIXEL_NR_LED         1
#define	NEOPIXEL_RMT_CHANNEL    RMT_CHANNEL_1

#define MOTOR_MIN_PULSEWIDTH_US 1000  // Minimum pulse width in microsecond
#define MOTOR_MAX_PULSEWIDTH_US 2000  // Maximum pulse width in microsecond
#define MOTOR_ARM_PULSEWIDTH_US 1500  // Arm pulse width in microsecond
#define MOTOR_CALIBRATION_DELAY 8000  // Calibration delay in microsecond

// Time for watchdog to turn off motors if no commands arrive: 100ms
#define MOTOR_WATCHDOG_TIMEOUT  200

#define IMU_SAMPLE_TIME_MS 10

#define MSG_BUFFER_LEN 100

#define CHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != 1)){printf("Failed on line %d. Aborting.\n",__LINE__);vTaskDelete(NULL);}}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECKRET(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);return temp_rc;}}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

pixel_settings_t px;
uint32_t pixels[NEOPIXEL_NR_LED];

raw_imu_t imu;

rcl_init_options_t init_options;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_subscription_t control_subscriber;
lar_msgs__msg__CarControlStamped incoming_control_msg;
uint8_t incoming_control_msg_buffer[MSG_BUFFER_LEN];
int64_t last_control_time;

rcl_publisher_t pong_publisher;
rcl_subscription_t ping_subscriber;
std_msgs__msg__Header incoming_ping;
uint8_t incoming_ping_buffer[MSG_BUFFER_LEN];

rcl_timer_t imu_timer;
rcl_publisher_t imu_publisher;
lar_msgs__msg__IMUStamped outgoing_imu_measurement_msg;
uint8_t outgoing_imu_measurement_msg_buffer[MSG_BUFFER_LEN];

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED_IDLE,
  AGENT_CONNECTED_ACTIVE,
  AGENT_DISCONNECTED
} state;

static inline uint32_t convert_servo_steer_to_duty_us(float signal)
{
    if (global_config.servo_reverse)
    {
        signal = -signal;
    }
    if (signal < -1) signal = -1;
    if (signal > 1) signal = 1;

    return global_config.servo_center + signal * global_config.servo_max_d;
}

static inline uint32_t convert_motor_throttle_to_duty_us(float signal)
{
    if (signal < -1) signal = -1;
    if (signal > 1) signal = 1;
    return (signal + 1) / 2 * (MOTOR_MAX_PULSEWIDTH_US - MOTOR_MIN_PULSEWIDTH_US) + MOTOR_MIN_PULSEWIDTH_US;
}

void configure_led()
{
    gpio_reset_pin(GPIO_NUM_LED);
    // Set the GPIO as a push/pull output
    gpio_set_direction(GPIO_NUM_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_LED, 1);
}

void configure_neopixel()
{
    gpio_reset_pin(GPIO_NUM_NEOPIXEL_POWER);
    // Set the GPIO as a push/pull output
    gpio_set_direction(GPIO_NUM_NEOPIXEL_POWER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_NEOPIXEL_POWER, 1);

    ESP_ERROR_CHECK(neopixel_init(GPIO_NUM_NEOPIXEL, NEOPIXEL_RMT_CHANNEL));

    for	(int i = 0; i < NEOPIXEL_NR_LED; i++) {
        pixels[i] = 0;
    }
    px.pixels = (uint8_t*) pixels;
    px.pixel_count = NEOPIXEL_NR_LED;
    strcpy(px.color_order, "GRB");

    memset(&px.timings, 0, sizeof(px.timings));
    px.timings.mark.level0 = 1;
    px.timings.space.level0 = 1;
    px.timings.mark.duration0 = 12;
    px.nbits = 24;
    px.timings.mark.duration1 = 14;
    px.timings.space.duration0 = 7;
    px.timings.space.duration1 = 16;
    px.timings.reset.duration0 = 600;
    px.timings.reset.duration1 = 600;

    px.brightness = 0x80;
    np_set_pixel_rgbw(&px, 0, 0xFF, 0x00, 0x00, 0x00); // red
    np_show(&px, NEOPIXEL_RMT_CHANNEL);
}

void configure_servo()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_SERVO); // To drive a RC servo, one MCPWM generator is enough

    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, convert_servo_steer_to_duty_us(0.0f)));
}

void configure_motor()
{
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, GPIO_NUM_MOTOR); // To drive a RC servo, one MCPWM generator is enough

    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);

    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, convert_motor_throttle_to_duty_us(0.0f)));
}

void arm_motor()
{
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MOTOR_ARM_PULSEWIDTH_US));
}

void calibrate_motor()
{
    ESP_LOGI(TAG, "Starting calibration - Neutral - Press Set");
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MOTOR_ARM_PULSEWIDTH_US));
    vTaskDelay(MOTOR_CALIBRATION_DELAY / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Continuing calibration - Full throttle - Press Set");
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MOTOR_MAX_PULSEWIDTH_US));
    vTaskDelay(MOTOR_CALIBRATION_DELAY / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Continuing calibration - Full reverse - Press Set");
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MOTOR_MIN_PULSEWIDTH_US));
    vTaskDelay(MOTOR_CALIBRATION_DELAY / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Calibration finished - Arming...");
    arm_motor();
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) //add error checks
{
    RCLC_UNUSED(last_call_time);

    if (timer != NULL) {

        bno08x_read_packet();
        int64_t nanos = rmw_uros_epoch_nanos();
        handle_report_bno08x(&imu);

        outgoing_imu_measurement_msg.header.stamp.sec = nanos / 1000000000;
        outgoing_imu_measurement_msg.header.stamp.nanosec = nanos % 1000000000;
        
        outgoing_imu_measurement_msg.acc.status = imu.status[0];
        outgoing_imu_measurement_msg.acc.delay_us = imu.delay[0]*100;

        outgoing_imu_measurement_msg.acc.x = (double) imu.acc[0] / 256;
        outgoing_imu_measurement_msg.acc.y = (double) imu.acc[1] / 256;
        outgoing_imu_measurement_msg.acc.z = (double) imu.acc[2] / 256;

        outgoing_imu_measurement_msg.gyro.status = imu.status[1];
        outgoing_imu_measurement_msg.gyro.delay_us = imu.delay[1]*100;

        outgoing_imu_measurement_msg.gyro.x = (double) imu.gyro[0] / 512;
        outgoing_imu_measurement_msg.gyro.y = (double) imu.gyro[1] / 512;
        outgoing_imu_measurement_msg.gyro.z = (double) imu.gyro[2] / 512;

        RCSOFTCHECK(rcl_publish(&imu_publisher, (const void*) &outgoing_imu_measurement_msg, NULL));
    }
}

void control_subscription_callback(const void * msgin)
{
    const lar_msgs__msg__CarControlStamped *msg = (const lar_msgs__msg__CarControlStamped*) msgin;

    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, convert_servo_steer_to_duty_us(msg->steering)));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, convert_motor_throttle_to_duty_us(msg->throttle)));


    if (state != AGENT_CONNECTED_ACTIVE) {
        state = AGENT_CONNECTED_ACTIVE;

        np_set_pixel_rgbw(&px, 0, 0x00, 0x00, 0xFF, 0x00); // blue
        np_show(&px, NEOPIXEL_RMT_CHANNEL);
    }
    last_control_time = uxr_millis();
}

void ping_subscription_callback(const void * msgin)
{
    int64_t nanos = rmw_uros_epoch_nanos();
    incoming_ping.stamp.sec = nanos / 1000000000;
    incoming_ping.stamp.nanosec = nanos % 1000000000;

    RCSOFTCHECK(rcl_publish(&pong_publisher, (const void*) &incoming_ping, NULL));
}

rcl_ret_t create_entities()
{
    ESP_LOGI(TAG, "Setup ros node, publishers and subscribers.");
    
    // create init_options
    if (global_config.domain_id != 0)
    {
        RCCHECK(rcl_init_options_set_domain_id(&init_options, global_config.domain_id));
    }
    RCSOFTCHECKRET(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    RCSOFTCHECKRET(rclc_node_init_default(&node, "la_racer", "", &support));

    // create control subscriber
    RCSOFTCHECKRET(rclc_subscription_init_best_effort(&control_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(lar_msgs, msg, CarControlStamped), global_config.control_topic));

    // Ping and pong publisher and subscriber
    RCSOFTCHECKRET(rclc_publisher_init_best_effort(&pong_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), global_config.pong_topic));
    RCSOFTCHECKRET(rclc_subscription_init_best_effort(&ping_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), global_config.ping_topic));

    // IMU measurements publisher 
    RCSOFTCHECKRET(rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(lar_msgs, msg, IMUStamped), global_config.imu_topic));

    // Create a timer to sample imu
    RCSOFTCHECKRET(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(IMU_SAMPLE_TIME_MS), imu_timer_callback));

    // create executor
    RCSOFTCHECKRET(rclc_executor_init(&executor, &support.context, 2 + global_config.imu_enabled, &allocator));
    RCSOFTCHECKRET(rclc_executor_add_subscription(&executor, &control_subscriber, &incoming_control_msg, &control_subscription_callback, ON_NEW_DATA));
    RCSOFTCHECKRET(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping, &ping_subscription_callback, ON_NEW_DATA));
    if (global_config.imu_enabled) {
        RCSOFTCHECKRET(rclc_executor_add_timer(&executor, &imu_timer));
    }

    return RCL_RET_OK;
}

void destroy_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_subscription_fini(&control_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&ping_subscriber, &node));

    RCSOFTCHECK(rcl_publisher_fini(&pong_publisher, &node));
    
    RCSOFTCHECK(rcl_timer_fini(&imu_timer));

    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));

    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

void micro_ros_task(void * arg)
{
    static micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 50;

    CHECK(micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(lar_msgs, msg, IMUStamped),
        &outgoing_imu_measurement_msg,
        conf,
        outgoing_imu_measurement_msg_buffer,
        sizeof(outgoing_imu_measurement_msg_buffer)
    ));
    outgoing_imu_measurement_msg.header.frame_id = micro_ros_string_utilities_set(outgoing_imu_measurement_msg.header.frame_id, "body");
    
    CHECK(micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(lar_msgs, msg, CarControlStamped),
        &incoming_control_msg,
        conf,
        incoming_control_msg_buffer,
        sizeof(incoming_control_msg_buffer)
    ));

    CHECK(micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
        &incoming_ping,
        conf,
        incoming_ping_buffer,
        sizeof(incoming_ping_buffer)
    ));

    configure_servo();
    configure_motor();
    if (global_config.imu_enabled) {
        configure_bno08x();
    }

    // calibrate_motor();
    arm_motor();
    vTaskDelay(2000 / portTICK_RATE_MS);

    state = WAITING_AGENT;

    allocator = rcl_get_default_allocator();
    node = rcl_get_zero_initialized_node();

    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    char agent_port[10];
    sprintf(agent_port, "%d", global_config.agent_port);
    RCCHECK(rmw_uros_options_set_udp_address(global_config.agent_ip, agent_port, rmw_options));

    while(1) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent_options(100, 1, rmw_options)) ? AGENT_AVAILABLE : WAITING_AGENT;);
                break;
            case AGENT_AVAILABLE:
                state = (RCL_RET_OK == create_entities()) ? AGENT_CONNECTED_IDLE : WAITING_AGENT;

                if (state == AGENT_CONNECTED_IDLE) {
                    state = (RMW_RET_OK == rmw_uros_sync_session(500)) ? AGENT_CONNECTED_IDLE : WAITING_AGENT;
                }

                if (state == AGENT_CONNECTED_IDLE) {
                    np_set_pixel_rgbw(&px, 0, 0x00, 0xFF, 0x00, 0x00); // green
                    np_show(&px, NEOPIXEL_RMT_CHANNEL);
                }

                if (state == WAITING_AGENT) {
                    destroy_entities();
                }
                break;
            case AGENT_CONNECTED_ACTIVE:
                if (uxr_millis() - last_control_time > MOTOR_WATCHDOG_TIMEOUT) {
                    state = AGENT_CONNECTED_IDLE;

                    np_set_pixel_rgbw(&px, 0, 0x00, 0xFF, 0x00, 0x00); // green
                    np_show(&px, NEOPIXEL_RMT_CHANNEL);
                }
                break;
            case AGENT_CONNECTED_IDLE:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 3)) ? AGENT_CONNECTED_IDLE : AGENT_DISCONNECTED;);
                break;
            case AGENT_DISCONNECTED:
                np_set_pixel_rgbw(&px, 0, 0xFF, 0x00, 0x00, 0x00); // red
                np_show(&px, NEOPIXEL_RMT_CHANNEL);

                destroy_entities();

                state = WAITING_AGENT;
                break;
            default:
                break;
        }

        if (state == AGENT_CONNECTED_IDLE || state == AGENT_CONNECTED_ACTIVE) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }

        if (state != AGENT_CONNECTED_ACTIVE) {
            // stop motors
            ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, convert_servo_steer_to_duty_us(0.0f)));
            ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, convert_motor_throttle_to_duty_us(0.0f)));
        }
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    configure_led();
    configure_neopixel();
    nvs_init();
    nvs_load_config();
    nvs_print_config();

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(network_interface_initialize());
    // disable wifi power safe mode
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#endif

    config_server_start();

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
