#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "Motor.h"
#include "kraken_pinout.h"
#include "configuration.h"
#include "motor_control.h"

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_srvs/srv/trigger.h>

// Motor(step_pin, dir_pin, cs_pin, en_pin, endstop_pin, RSense
//       current_mA, use_stealthchop, use_stallguard,
//       homing_dir, homing_speed, invert_dir)

Motor joints[6] = {                                                                 
//Motor(STEP        , DIR        , CS        , ENABLE        , ENDSTOP     , RSENSE    , CURRENT, STEPS_PER_REV, SG HOMING, SGT, HOMING DIR, HOMING SPEED, INVERT )
  Motor(X_STEP_PIN  , X_DIR_PIN  , X_CS_PIN  , X_ENABLE_PIN  , X_STOP_PIN  , X_RSENSE  , 800    , 200          , false    , 0  , -1        , 1           , false  ), // Yaw
  Motor(Y_STEP_PIN  , Y_DIR_PIN  , Y_CS_PIN  , Y_ENABLE_PIN  , Y_STOP_PIN  , Y_RSENSE  , 800    , 200          , false    , 0  , -1        , 1           , false  ), // Shoulder
  Motor(Z_STEP_PIN  , Z_DIR_PIN  , Z_CS_PIN  , Z_ENABLE_PIN  , Z_STOP_PIN  , Z_RSENSE  , 800    , 200          , false    , 0  , -1        , 1           , false  ), // Elbow
  Motor(E0_STEP_PIN , E0_DIR_PIN , E0_CS_PIN , E0_ENABLE_PIN , E0_DIAG_PIN , E0_RSENSE , 800    , 200          , false    , 0  , -1        , 1           , false  ), // Wrist
  Motor(E1_STEP_PIN , E1_DIR_PIN , E1_CS_PIN , E1_ENABLE_PIN , E1_DIAG_PIN , E1_RSENSE , 800    , 200          , false    , 0  , -1        , 1           , false  ), // Wrist
  Motor(E2_STEP_PIN , E2_DIR_PIN , E2_CS_PIN , E2_ENABLE_PIN , E3_DIAG_PIN , E2_RSENSE , 800    , 200          , false    , 0  , -1        , 1           , false  ) // Wrist
};

Motor drill1(E3_STEP_PIN, E3_DIR_PIN, E3_CS_PIN, E3_ENABLE_PIN, E3_DIAG_PIN, E3_RSENSE, 800, 200, false, 0, -1, 1, false);
Motor drill2(E4_STEP_PIN, E3_DIR_PIN, E4_CS_PIN, E4_ENABLE_PIN, E4_DIAG_PIN, E4_RSENSE, 800, 200, false, 0, -1, 1, false);
bool homing_complete = false;

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

// ================ Homing Function =================
void perform_homing() {
  for (Motor& joint : joints){
    joint.home();
  } 
  drill1.home();
  drill2.home();
  homing_complete = true;

  homing_done_msg.data = true;
  rcl_publish(&homing_done_pub, &homing_done_msg, NULL);
}

// ================ Homing Service Callback =================
void homing_service_callback(const void* req, void* res) {
  perform_homing();
  auto* response = (std_srvs__srv__Trigger_Response*)res;
  response->success = true;
  response->message.data = strdup("Homing complete.");
}

// ==================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(2000);

  for (Motor& joint : joints) joint.begin();
  drill1.begin();
  drill2.begin();

  // ==== micro-ROS Setup ====
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_node", "", &support);

  // ==== Publisher ====
  rclc_publisher_init_default(
    &homing_done_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "homing_done"
  );

  // ==== Subscriber ====
  rclc_subscription_init_default(
    &motor_command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "joint_positions"
  );

  // ==== Homing Service ====
  rclc_service_init_default(
    &homing_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "home_all"
  );

  // ==== Executor ====
  rclc_executor_init(&executor, &support.context, 3, &allocator);

  rclc_executor_add_subscription(
    &executor, &motor_command_sub, &motor_command_msg,
    &motor_command_callback, ON_NEW_DATA
  );

  rclc_executor_add_service(
    &executor, &homing_service, &homing_req, &homing_res,
    homing_service_callback
  );
}

// ==================== Main Loop =====================
void loop() {
  for (Motor& joint : joints) {
    joint.run();
  }
  drill1.run();
  drill2.run();

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}