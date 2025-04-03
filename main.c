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
#define THUMB_ADDR 1
#define MCP_ADDR 2
#define PIP_ADDR 3
#define DIP_ADDR 4
#define ABDUCTION_ADDR 5

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

const float CMC_INDEX_RING = 1.0 / (0.4666 * 3.0);
const float CMC_PINKY = 1.0 / (0.6872 * 3.0);

// Joint data in radians
float data[18] = {0.0f};

// Normalizes array values between 0 and 1
void normalize(float* arr, int size) {
    for (int i = 0; i < size; i++) {
        if (arr[i] > 1.0f) {
            arr[i] = 1.0f;
        }
        else if (arr[i] < 0.0f) {
            arr[i] = 0.0f;
        }
    }
}

// Writes joint angle data to I2C
void writeData(float jointData[18]) {
    float thumbArr[3] = {jointData[0] * CMC_THUMB, jointData[1] * MCP_THUMB, jointData[2] * IP_THUMB};
    float MCPArr[4] = {jointData[4] * MCP_INDEX, jointData[7] * MCP_MIDDLE, jointData[11] * MCP_RING, jointData[15] * MCP_PINKY};
    float PIPArr[4] = {jointData[5] * PIP_INDEX, jointData[8] * PIP_MIDDLE, jointData[12] * PIP_RING, jointData[16] * PIP_PINKY};
    float DIPArr[4] = {jointData[6] * PIP_INDEX, jointData[9] * DIP_MIDDLE, jointData[13] * DIP_RING, jointData[17] * DIP_PINKY};
    float abduction = (jointData[3] + jointData[10]) * CMC_INDEX_RING + jointData[14] * CMC_PINKY; 

    normalize(thumbArr, 3);
    normalize(MCPArr, 4);
    normalize(PIPArr, 4);
    normalize(DIPArr, 4);
    normalize(&abduction, 1);

    i2c_write_blocking(I2C_PORT, THUMB_ADDR , (uint8_t*) thumbArr , 12U, false);
    i2c_write_blocking(I2C_PORT, MCP_ADDR , (uint8_t*) MCPArr , 16U, false);
    i2c_write_blocking(I2C_PORT, PIP_ADDR , (uint8_t*) PIPArr , 16U, false);
    i2c_write_blocking(I2C_PORT, DIP_ADDR , (uint8_t*) DIPArr , 16U, false);
    i2c_write_blocking(I2C_PORT, ABDUCTION_ADDR , (uint8_t*) &abduction , 4U, false);
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

    gpio_put(LED_PIN, true);

    rclc_executor_spin(&executor);
    
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    return 0;
}
