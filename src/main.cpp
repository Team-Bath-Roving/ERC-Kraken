#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include "kraken_pinout.h"
#include "configuration.h"
#include "motors.h"

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/set_bool.h>
#include <sensor_msgs/msg/joint_state.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

/* ------------------------------- ROS objects ------------------------------ */

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;


/* ------------------------------- ROS Topics ------------------------------- */

rcl_publisher_t homing_done_pub;
std_msgs__msg__Bool homing_done_msg;

rcl_publisher_t joint_state_pub;
sensor_msgs__msg__JointState joint_state_msg;

rcl_subscription_t motor_command_sub;
std_msgs__msg__Float32MultiArray motor_command_msg;

rcl_subscription_t estop_sub;
std_msgs__msg__Bool estop_msg;

rcl_subscription_t trajectory_sub;
trajectory_msgs__msg__JointTrajectory trajectory_msg;

rcl_service_t homing_service;
std_srvs__srv__Trigger_Request homing_req;
std_srvs__srv__Trigger_Response homing_res;

rcl_service_t enable_motors_service;
std_srvs__srv__SetBool_Request enable_motors_req;
std_srvs__srv__SetBool_Response enable_motors_res;

rcl_service_t manual_home_service;
std_srvs__srv__Trigger_Request manual_home_req;
std_srvs__srv__Trigger_Response manual_home_res;


/* -------------------------------- Variables ------------------------------- */

bool homing_complete = false;
bool estop_active = false;
double joint_positions[NUM_JOINTS];
rosidl_runtime_c__String joint_name_strings[NUM_JOINTS];


/* ---------------------------- Helper functions ---------------------------- */

int get_joint_index_by_name(const char* name) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (strcmp(joint_names[i], name) == 0) {
            return i;
        }
    }
    return -1; // Not found
}
/* --------------------------------- Homing --------------------------------- */

void perform_homing() {
  for (int i=0; i<NUM_JOINTS; i++) {
    joints[i].home();
  }
  // D1 is spare, not homed here
  homing_complete = true;

  homing_done_msg.data = true;
  rcl_publish(&homing_done_pub, &homing_done_msg, NULL);
}

/* -------------------------------- Callbacks ------------------------------- */

void motor_command_callback(const void* msgin) {
  const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)msgin;
  if (estop_active) return;

  // Fix: Only iterate up to min(NUM_JOINTS, msg->data.size)
  size_t limit = (msg->data.size < NUM_JOINTS) ? msg->data.size : NUM_JOINTS;
  for (size_t i = 0; i < limit; i++) {
    joints[i].moveTo(msg->data.data[i]);
  }
}

void estop_callback(const void* msgin) {
  const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msgin;
  estop_active = msg->data;

  for (Motor& joint : joints) estop_active ? joint.disable() : joint.enable();
  // d1 is not used
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
  // d1 is not used

  auto* response = (std_srvs__srv__SetBool_Response*)res;
  response->success = true;
  response->message.data = strdup(enable ? "Motors enabled" : "Motors disabled");
}

void manual_home_service_callback(const void* req, void* res) {
  // Example: disable last two joints for manual wrist homing (adjust as needed for 7DOF)
  joints[5].disable();
  joints[6].disable();

  delay(10000); // or wait for ROS signal to continue

  joints[5].enable();
  joints[6].enable();

  // Assume manual alignment happens now, then reset zero
  joints[5].setCurrentPosition(0);
  joints[6].setCurrentPosition(0);
  auto* response = (std_srvs__srv__Trigger_Response*)res;
  response->success = true;
  response->message.data = strdup("Wrist manually homed.");
}

void trajectory_callback(const void* msgin) {
  const trajectory_msgs__msg__JointTrajectory* msg = (const trajectory_msgs__msg__JointTrajectory*)msgin;

  if (msg->points.size == 0) return;

  const auto* point = &msg->points.data[0];
  size_t limit = (msg->joint_names.size < NUM_JOINTS && point->positions.size < NUM_JOINTS) ? msg->joint_names.size : NUM_JOINTS;
  if (point->positions.size < limit) limit = point->positions.size;
  for (size_t i = 0; i < limit; ++i) {
    const char* joint_name = msg->joint_names.data[i].data;
    uint joint_idx = get_joint_index_by_name(joint_name);
    if (joint_idx >= 0 && joint_idx < NUM_JOINTS) {
      double rad = point->positions.data[i];
      joints[joint_idx].moveTo(degrees(rad));
    }
  }
}


/* ---------------------------------- Estop --------------------------------- */

void check_estop_pin() {
  static bool last_state = false;
  bool current_state = digitalRead(ESTOP_PIN) == LOW; // Active-low

  if (current_state && !last_state) {
    estop_active = true;
    for (Motor& joint : joints) joint.stop();
    // d1 is not used
  }

  last_state = current_state;
}

/* ---------------------------------- Setup --------------------------------- */

void setup() {
  SerialUSB.begin();
  delay(2000);

  // Heartbeat pin setup
  pinMode(HEATER_0_PIN, OUTPUT);
  digitalWrite(HEATER_0_PIN, LOW);
  delay(500);
  digitalWrite(HEATER_0_PIN, HIGH);
  delay(500);
  digitalWrite(HEATER_0_PIN, LOW);
  delay(500);
  digitalWrite(HEATER_0_PIN, HIGH);
  delay(500);
  digitalWrite(HEATER_0_PIN, LOW);

  set_microros_serial_transports(SerialUSB);

  // Start motors
  for (Motor& joint : joints) joint.begin();
  // d1 is not used

  // Create the node
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_node", "", &support);

  /* ----------------------------- Register Topics ---------------------------- */
  // Publishers
  rclc_publisher_init_default(&joint_state_pub,&node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states");
    
  rclc_publisher_init_default(&homing_done_pub,&node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "homing_done");

  // Subscriptions
  rclc_subscription_init_default(&motor_command_sub,&node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "joint_positions");
    
  rclc_subscription_init_default(&estop_sub,&node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "estop");

  rclc_subscription_init_default(&trajectory_sub,&node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
    "arm_controller/joint_trajectory");

  // Services
  rclc_service_init_default(&homing_service,&node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "home_all");
    
  rclc_service_init_default(&enable_motors_service,&node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "set_motors_enabled");
    
  rclc_service_init_default(&manual_home_service,&node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "manual_home_wrist");

  // Prepare static JointState name strings once
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    rosidl_runtime_c__String__assign(&joint_name_strings[i], joint_names[i]);
  }

  // Wire up JointState message fields once
  joint_state_msg.name.size = NUM_JOINTS;
  joint_state_msg.name.capacity = NUM_JOINTS;
  joint_state_msg.name.data = joint_name_strings;

  joint_state_msg.position.size = NUM_JOINTS;
  joint_state_msg.position.capacity = NUM_JOINTS;
  joint_state_msg.position.data = joint_positions;

  
  /* --------------------------- Register Callbacks --------------------------- */

  rclc_executor_init(&executor, &support.context, 5, &allocator);

  rclc_executor_add_subscription(&executor, &motor_command_sub, &motor_command_msg,
    &motor_command_callback, ON_NEW_DATA);

  rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg,
    &estop_callback, ON_NEW_DATA);

  rclc_executor_add_service(&executor, &homing_service, &homing_req, &homing_res,
    homing_service_callback);

  rclc_executor_add_service(&executor, &enable_motors_service, &enable_motors_req, &enable_motors_res,
    enable_motors_service_callback);

  rclc_executor_add_service(&executor, &manual_home_service, &manual_home_req, &manual_home_res,
    manual_home_service_callback);
}

/* ---------------------------------- Loop ---------------------------------- */

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  check_estop_pin();

  for (Motor& joint : joints) joint.run();
  // d1 is not used

  // Heartbeat: toggle FAN0_PIN every second
  static uint32_t last_toggle = 0;
  static bool fan_state = false;
  uint32_t now = millis();
  if (now - last_toggle > 1000) {
    fan_state = !fan_state;
    digitalWrite(FAN0_PIN, fan_state ? HIGH : LOW);
    last_toggle = now;
  }

  uint64_t now_ns = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec = now_ns / 1000000000ULL;
  joint_state_msg.header.stamp.nanosec = now_ns % 1000000000ULL;

  // Fix: Only iterate up to NUM_JOINTS and check array bounds
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    joint_positions[i] = radians(joints[i].currentPosition()); // Must be in radians
  }
  rcl_publish(&joint_state_pub, &joint_state_msg, NULL);

  // Serial echo using SerialUSB
  if (SerialUSB.available()) {
    char c = SerialUSB.read();
    SerialUSB.write(c);
  }
}
