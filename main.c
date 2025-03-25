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

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

const uint LED_PIN = 25;

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;

float data[18];

const size_t msg_size = sizeof(data);
const size_t fingerArrSize = 3 * sizeof(float);

void message_callback(const void* msgin)
{
    const std_msgs__msg__Float32MultiArray* jointMsg = (const std_msgs__msg__Float32MultiArray*) msgin; 
    memcpy((void*) (msg.data.data), (void*) (jointMsg->data.data), msg_size);
}

void writeData() {
    float* floatPtr = msg.data.data;

    float thumbArr[3] = {floatPtr[0], floatPtr[1], floatPtr[2]};
    float indexArr[3] = {floatPtr[4], floatPtr[5], floatPtr[6]};
    float middleArr[3] = {floatPtr[7], floatPtr[8], floatPtr[9]};
    float ringArr[3] = {floatPtr[11], floatPtr[12], floatPtr[13]};
    float pinkyArr[3] = {floatPtr[15], floatPtr[16], floatPtr[17]};

    i2c_write_blocking(I2C_PORT, 0, (uint8_t*) thumbArr, fingerArrSize, false);
    i2c_write_blocking(I2C_PORT, 1, (uint8_t*) indexArr, fingerArrSize, false);
    i2c_write_blocking(I2C_PORT, 2, (uint8_t*) middleArr, fingerArrSize, false);
    i2c_write_blocking(I2C_PORT, 3, (uint8_t*) ringArr, fingerArrSize, false);
    i2c_write_blocking(I2C_PORT, 4, (uint8_t*) pinkyArr, fingerArrSize, false);
}

int main()
{
    msg.data.capacity = 18;
    msg.data.data = data;
    msg.data.size = 0;

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

    rcl_node_t node;
    rclc_support_t support;
    rclc_executor_t executor;

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);

    ret = rclc_subscription_init_default(
        &subscriber, &node,
        type_support, "/right_hand");
      
    if (ret != RCL_RET_OK) {
        return -1;
    }

    ret = rclc_executor_add_subscription(
        &executor, &subscriber, &msg,
        &message_callback, ON_NEW_DATA);
      
    if (ret != RCL_RET_OK) {
        return -1;
    }

    gpio_put(LED_PIN, 1);

    while (true)
    {
        writeData();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    rcl_subscription_fini(&subscriber, &node);

    return 0;
}
