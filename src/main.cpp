#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include "kraken_pinout.h"
#include "configuration.h"
#include "motors.h"

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/set_bool.h>

// ===================== ROS 2 Core =====================
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// ===================== Messages & Services =====================
rcl_publisher_t homing_done_pub;
std_msgs__msg__Bool homing_done_msg;

rcl_subscription_t motor_command_sub;
std_msgs__msg__Int32MultiArray motor_command_msg;

rcl_subscription_t estop_sub;
std_msgs__msg__Bool estop_msg;

rcl_service_t homing_service;
std_srvs__srv__Trigger_Request homing_req;
std_srvs__srv__Trigger_Response homing_res;

rcl_service_t enable_motors_service;
std_srvs__srv__SetBool_Request enable_motors_req;
std_srvs__srv__SetBool_Response enable_motors_res;

rcl_service_t manual_home_service;
std_srvs__srv__Trigger_Request manual_home_req;
std_srvs__srv__Trigger_Response manual_home_res;

// ===================== Flags =====================
bool homing_complete = false;
bool estop_active = false;

// ===================== Callbacks =====================
void motor_command_callback(const void* msgin) {
  const std_msgs__msg__Int32MultiArray* msg = (const std_msgs__msg__Int32MultiArray*)msgin;
  if (estop_active) return;

  for (size_t i = 0; i < NUM_JOINTS; i++) {
    joints[i].moveTo(msg->data.data[i]);
  }
}

void estop_callback(const void* msgin) {
  const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
  estop_active = msg->data;

  for (Motor& joint : joints) estop_active ? joint.disable() : joint.enable();
  estop_active ? drill1.disable() : drill1.enable();
  estop_active ? drill2.disable() : drill2.enable();
}

void homing_service_callback(const void* req, void* res) {
  perform_homing();
  auto* response = (std_srvs__srv__Trigger_Response*)res;
  response->success = true;
  response->message.data = strdup("Homing complete.");
}

void enable_motors_service_callback(const void* req, void* res) {
  auto* request = (const std_srvs__srv__SetBool_Request*)req;
  bool enable = request->data;

  for (Motor& joint : joints) enable ? joint.enable() : joint.disable();
  enable ? drill1.enable() : drill1.disable();
  enable ? drill2.enable() : drill2.disable();

  auto* response = (std_srvs__srv__SetBool_Response*)res;
  response->success = true;
  response->message.data = strdup(enable ? "Motors enabled" : "Motors disabled");
}

void manual_home_service_callback(const void* req, void* res) {
  joints[4].disable();
  joints[5].disable();

  delay(10000); // or wait for ROS signal to continue

  joints[4].enable();
  joints[5].enable();

  // Assume manual alignment happens now, then reset zero
  joints[4].setCurrentPosition(0);
  joints[5].setCurrentPosition(0);
  auto* response = (std_srvs__srv__Trigger_Response*)res;
  response->success = true;
  response->message.data = strdup("Wrist manually homed.");
}

// ===================== Homing Function =====================
void perform_homing() {
  // Don't home the last joint?
  for (int i=0; i<5; i++) {
    joints[i].home();
  }
  // for (Motor& joint : joints) joint.home();
  drill1.home();
  drill2.home();
  homing_complete = true;

  homing_done_msg.data = true;
  rcl_publish(&homing_done_pub, &homing_done_msg, NULL);
}

// ===================== Physical E-stop =====================
void check_estop_pin() {
  static bool last_state = false;
  bool current_state = digitalRead(ESTOP_PIN) == LOW; // Active-low

  if (current_state && !last_state) {
    estop_active = true;
    for (Motor& joint : joints) joint.stop();
    drill1.stop();
    drill2.stop();
  }

  last_state = current_state;
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(2000);

  for (Motor& joint : joints) joint.begin();
  drill1.begin();
  drill2.begin();

  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_node", "", &support);

  rclc_publisher_init_default(
    &homing_done_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "homing_done"
  );

  rclc_subscription_init_default(
    &motor_command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "joint_positions"
  );

  rclc_subscription_init_default(
    &estop_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "estop"
  );

  rclc_service_init_default(
    &homing_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "home_all"
  );

  rclc_service_init_default(
    &enable_motors_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "set_motors_enabled"
  );

  rclc_service_init_default(
    &manual_home_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "manual_home_wrist"
  );

  rclc_executor_init(&executor, &support.context, 5, &allocator);

  rclc_executor_add_subscription(
    &executor, &motor_command_sub, &motor_command_msg,
    &motor_command_callback, ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor, &estop_sub, &estop_msg,
    &estop_callback, ON_NEW_DATA
  );

  rclc_executor_add_service(
    &executor, &homing_service, &homing_req, &homing_res,
    homing_service_callback
  );

  rclc_executor_add_service(
    &executor, &enable_motors_service, &enable_motors_req, &enable_motors_res,
    enable_motors_service_callback
  );

  rclc_executor_add_service(
    &executor, &manual_home_service, &manual_home_req, &manual_home_res,
    manual_home_service_callback
  );
}

// ===================== Loop =====================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  check_estop_pin();

  for (Motor& joint : joints) joint.run();
  drill1.run();
  drill2.run();
}
