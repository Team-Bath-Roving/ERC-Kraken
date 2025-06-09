#pragma once

#include "motors.h"
#include "configuration.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/set_bool.h>

// void motor_command_callback(const void* msgin);
// void setup_motor_subscriber(rcl_node_t* node, rclc_executor_t* executor);


// ================ ROS 2 Node, Executor, Msgs ================
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t homing_done_pub;
std_msgs__msg__Bool homing_done_msg;

rcl_subscription_t motor_command_sub;
std_msgs__msg__Int32MultiArray motor_command_msg;

rcl_service_t homing_service;
std_srvs__srv__Trigger_Request homing_req;
std_srvs__srv__Trigger_Response homing_res;

rcl_service_t enable_motors_service;
std_srvs__srv__SetBool_Request enable_motors_req;
std_srvs__srv__SetBool_Response enable_motors_res;

rcl_subscription_t motor_sub;
std_msgs__msg__Int32MultiArray motor_cmd_msg;

void motor_command_callback(const void* msgin) {
  const std_msgs__msg__Int32MultiArray* msg = (const std_msgs__msg__Int32MultiArray*)msgin;

  size_t motor_count = NUM_JOINTS;

  for (size_t i = 0; i < motor_count; i++) {
    int32_t target = msg->data.data[i];
    joints[i].moveTo(target);
  }
}

void setup_motor_subscriber(rcl_node_t* node, rclc_executor_t* executor) {
  const rosidl_message_type_support_t* type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray);

  const rcl_subscription_options_t options = rcl_subscription_get_default_options();
  auto ret = rcl_subscription_init(&motor_sub, node, type_support, "motor_commands",&options);

  if (ret != RCL_RET_OK) {
    // Handle error (e.g. log, blink LED, halt, etc.)
    Serial.println("Failed to init motor_sub subscription");
  }

  rclc_executor_add_subscription(executor, &motor_sub, &motor_cmd_msg, &motor_command_callback, ON_NEW_DATA);
}

// ================ Homing Service Callback =================
void homing_service_callback(const void* req, void* res) {
  perform_homing();
  auto* response = (std_srvs__srv__Trigger_Response*)res;
  response->success = true;
  response->message.data = strdup("Homing complete.");
}



void enable_motors_service_callback(const void* req, void* res) {
  auto* request = (const std_srvs__srv__SetBool_Request*)req;
  bool enable = request->data;

  for (Motor& joint : joints) {
    enable ? joint.enable() : joint.disable();
  }
  enable ? drill1.enable() : drill1.disable();
  enable ? drill2.enable() : drill2.disable();

  auto* response = (std_srvs__srv__SetBool_Response*)res;
  response->success = true;
  response->message.data = strdup(enable ? "Motors enabled" : "Motors disabled");
}
