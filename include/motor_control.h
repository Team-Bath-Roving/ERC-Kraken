#pragma once

#include <std_msgs/msg/int32_multi_array.h>
#include "Motor.h"
#include "configuration.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

extern Motor joints[6];
void motor_command_callback(const void* msgin);
void setup_motor_subscriber(rcl_node_t* node, rclc_executor_t* executor);
