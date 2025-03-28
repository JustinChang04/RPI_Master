#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/i2c.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return temp_rc;}}

// LED Pin
const uint LED_PIN = 25;

// I2C Pins
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

// I2C Addresses
#define THUMB_ADDR 0
#define INDEX_ADDR 1
#define MIDDLE_ADDR 2
#define RING_ADDR 3
#define PINKY_ADDR 4

// Limits
const float CMC_THUMB = 1.0 / 1.8545;
const float MCP_THUMB = 1.0 / 0.9203;
const float IP_THUMB = 1.0 / 0.7858;

const float MCP_INDEX = 1.0 / 1.8002;
const float PIP_INDEX = 1.0 / 1.3104;
const float DIP_INDEX = 1.0 / 1.1885;

const float MCP_MIDDLE = 1.0 / 1.7791;
const float PIP_MIDDLE = 1.0 / 1.2823;
const float DIP_MIDDLE = 1.0 / 1.275;

const float MCP_RING = 1.0 / 1.7553;
const float PIP_RING = 1.0 / 1.273;
const float DIP_RING = 1.0 / 1.2842;

const float MCP_PINKY = 1.0 / 1.7269;
const float PIP_PINKY = 1.0 / 1.2573;
const float DIP_PINKY = 1.0 / 1.2577;

// Joint data in radians
float data[18] = {0.0f};

// Writes joint angle data to I2C
void writeData(float jointData[18]) {
    float thumbArr[3] = {jointData[0] * CMC_THUMB, jointData[1] * MCP_THUMB, jointData[2] * IP_THUMB};
    float indexArr[3] = {jointData[4] * MCP_INDEX, jointData[5] * PIP_INDEX, jointData[6] * DIP_INDEX};
    float middleArr[3] = {jointData[7] * MCP_MIDDLE, jointData[8] * PIP_MIDDLE, jointData[9] * DIP_MIDDLE};
    float ringArr[3] = {jointData[11] * MCP_RING, jointData[12] * PIP_RING, jointData[13] * DIP_RING};
    float pinkyArr[3] = {jointData[15] * MCP_PINKY, jointData[16] * PIP_PINKY, jointData[17] * DIP_PINKY};

    i2c_write_blocking(I2C_PORT, THUMB_ADDR , (uint8_t*) thumbArr , 12U, false);
    i2c_write_blocking(I2C_PORT, INDEX_ADDR , (uint8_t*) indexArr , 12U, false);
    i2c_write_blocking(I2C_PORT, MIDDLE_ADDR, (uint8_t*) middleArr, 12U, false);
    i2c_write_blocking(I2C_PORT, RING_ADDR  , (uint8_t*) ringArr  , 12U, false);
    i2c_write_blocking(I2C_PORT, PINKY_ADDR , (uint8_t*) pinkyArr , 12U, false);
}

// Subscriber callback
void message_callback(const void* msgin)
{  
    const std_msgs__msg__Float32MultiArray* jointMsg = (const std_msgs__msg__Float32MultiArray*) msgin; 
    memcpy((void*) (data), (void*) (jointMsg->data.data), sizeof(data));

    writeData(data);
}

int main()
{
    stdio_init_all();

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    i2c_init(I2C_PORT, 100*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_subscription_t subscriber;
    rcl_node_t node;
    rclc_support_t support;
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    RCCHECK(rmw_uros_ping_agent(timeout_ms, attempts));

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "main", "", &support));

    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);

    char topic_name[12] = "/right_hand";
    RCCHECK(rclc_subscription_init_default(
        &subscriber, &node,
        type_support, topic_name));
        

    float msg_data[18] = {0.0f};
    std_msgs__msg__Float32MultiArray msg;
    msg.data.capacity = 18;
    msg.data.data = msg_data;
    msg.data.size = 0;

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &subscriber, &msg,
        &message_callback, ON_NEW_DATA));

    rclc_executor_spin(&executor);
    
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    return 0;
}
