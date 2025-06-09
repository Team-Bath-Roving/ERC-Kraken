#include "motor_control.h"
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>

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
