#include "ros_bridge.h"

#if ENABLE_MICRO_ROS

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

#include "config.h"
#include "machine.h"

static rcl_allocator_t allocator;
static rcl_init_options_t initOptions;
static rclc_support_t support;
static rcl_node_t node;
static rcl_timer_t telemetryTimer;
static rclc_executor_t executor;

static rcl_publisher_t jointStatePublisher;
static rcl_publisher_t axisPositionPublisher;
static rcl_publisher_t motorRpmPublisher;
static rcl_publisher_t statusPublisher;
static rcl_publisher_t faultPublisher;
static rcl_subscription_t emergencyStopSubscriber;

static sensor_msgs__msg__JointState jointStateMsg;
static std_msgs__msg__Float32MultiArray axisPositionMsg;
static std_msgs__msg__Float32MultiArray motorRpmMsg;
static std_msgs__msg__String statusMsg;
static std_msgs__msg__String faultMsg;
static std_msgs__msg__Bool emergencyStopMsg;

static rosidl_runtime_c__String jointNames[4];
static double jointPositions[4];
static double jointVelocities[4];
static double jointEfforts[4];
static float axisPositionsMm[3];
static float motorRpms[4];
static char statusBuffer[512];
static char faultBuffer[96];
static char frameIdBuffer[] = "palletizer_base";
static char jointNameBuffers[4][4] = {"X1", "X2", "Y", "Z"};

static bool rosReady = false;

static bool check(rcl_ret_t result) {
  return result == RCL_RET_OK;
}

static void setString(rosidl_runtime_c__String &text, char *buffer, size_t capacity, size_t size = 0) {
  text.data = buffer;
  text.capacity = capacity;
  text.size = size;
}

static void stampJointState() {
  const int64_t epochMs = rmw_uros_epoch_millis();
  if (epochMs > 0) {
    jointStateMsg.header.stamp.sec = static_cast<int32_t>(epochMs / 1000);
    jointStateMsg.header.stamp.nanosec = static_cast<uint32_t>((epochMs % 1000) * 1000000);
    return;
  }

  const uint32_t nowMs = millis();
  jointStateMsg.header.stamp.sec = nowMs / 1000;
  jointStateMsg.header.stamp.nanosec = (nowMs % 1000) * 1000000;
}

static void fillTelemetryMessages() {
  stampJointState();

  float xSumMm = 0.0f;
  uint8_t xSamples = 0;
  const uint32_t now = millis();

  for (size_t i = 0; i < 4; i++) {
    const MotorNode &motor = motors[i];
    if (motor.encoderOk) {
      jointPositions[i] = static_cast<double>(motor.encoder) * 2.0 * M_PI / ENCODER_COUNTS_PER_REV;
      const float mm = encoderCountsToMm(motor.encoder);
      if (i < 2) {
        xSumMm += mm;
        xSamples++;
      } else {
        axisPositionsMm[i - 1] = mm;
      }
    } else {
      jointPositions[i] = 0.0;
      if (i >= 2) axisPositionsMm[i - 1] = 0.0f;
    }

    jointVelocities[i] = motor.rpmOk
                             ? static_cast<double>(motor.rpm) * 2.0 * M_PI / 60.0
                             : 0.0;
    jointEfforts[i] = 0.0;
    motorRpms[i] = motor.rpmOk ? static_cast<float>(motor.rpm) : 0.0f;
  }

  axisPositionsMm[0] = xSamples ? xSumMm / xSamples : 0.0f;

  const int written = snprintf(
      statusBuffer,
      sizeof(statusBuffer),
      "{\"fault\":%u,\"reason\":\"%s\",\"online\":[%u,%u,%u,%u],\"enc\":[%lld,%lld,%lld,%lld],\"mm\":[%.3f,%.3f,%.3f,%.3f],\"rpm\":[%d,%d,%d,%d],\"acc\":[%u,%u,%u,%u],\"moveStatus\":[%u,%u,%u,%u],\"homeStatus\":[%u,%u,%u,%u]}",
      safetyFaultIsActive() ? 1 : 0,
      safetyFaultText(),
      motorIsOnline(motors[0], now) ? 1 : 0,
      motorIsOnline(motors[1], now) ? 1 : 0,
      motorIsOnline(motors[2], now) ? 1 : 0,
      motorIsOnline(motors[3], now) ? 1 : 0,
      static_cast<long long>(motors[0].encoderOk ? motors[0].encoder : 0),
      static_cast<long long>(motors[1].encoderOk ? motors[1].encoder : 0),
      static_cast<long long>(motors[2].encoderOk ? motors[2].encoder : 0),
      static_cast<long long>(motors[3].encoderOk ? motors[3].encoder : 0),
      motors[0].encoderOk ? encoderCountsToMm(motors[0].encoder) : 0.0f,
      motors[1].encoderOk ? encoderCountsToMm(motors[1].encoder) : 0.0f,
      motors[2].encoderOk ? encoderCountsToMm(motors[2].encoder) : 0.0f,
      motors[3].encoderOk ? encoderCountsToMm(motors[3].encoder) : 0.0f,
      motors[0].rpmOk ? motors[0].rpm : 0,
      motors[1].rpmOk ? motors[1].rpm : 0,
      motors[2].rpmOk ? motors[2].rpm : 0,
      motors[3].rpmOk ? motors[3].rpm : 0,
      motors[0].lastAcc,
      motors[1].lastAcc,
      motors[2].lastAcc,
      motors[3].lastAcc,
      motors[0].moveStatus,
      motors[1].moveStatus,
      motors[2].moveStatus,
      motors[3].moveStatus,
      motors[0].homeStatus,
      motors[1].homeStatus,
      motors[2].homeStatus,
      motors[3].homeStatus);
  statusMsg.data.size = written > 0 ? min(static_cast<size_t>(written), sizeof(statusBuffer) - 1) : 0;

  const int faultWritten = snprintf(
      faultBuffer,
      sizeof(faultBuffer),
      "%s:%s",
      safetyFaultIsActive() ? "FAULT" : "OK",
      safetyFaultText());
  faultMsg.data.size = faultWritten > 0 ? min(static_cast<size_t>(faultWritten), sizeof(faultBuffer) - 1) : 0;
}

static void publishTelemetry(rcl_timer_t *, int64_t) {
  fillTelemetryMessages();
  rcl_publish(&jointStatePublisher, &jointStateMsg, nullptr);
  rcl_publish(&axisPositionPublisher, &axisPositionMsg, nullptr);
  rcl_publish(&motorRpmPublisher, &motorRpmMsg, nullptr);
  rcl_publish(&statusPublisher, &statusMsg, nullptr);
  rcl_publish(&faultPublisher, &faultMsg, nullptr);
}

static void onEmergencyStop(const void *msgIn) {
  const std_msgs__msg__Bool *msg = static_cast<const std_msgs__msg__Bool *>(msgIn);
  if (msg->data) {
    stopAllMotors();
  }
}

static void initMessages() {
  setString(jointStateMsg.header.frame_id, frameIdBuffer, sizeof(frameIdBuffer), strlen(frameIdBuffer));

  jointStateMsg.name.data = jointNames;
  jointStateMsg.name.size = 4;
  jointStateMsg.name.capacity = 4;
  for (size_t i = 0; i < 4; i++) {
    setString(jointNames[i], jointNameBuffers[i], sizeof(jointNameBuffers[i]), strlen(jointNameBuffers[i]));
  }

  jointStateMsg.position.data = jointPositions;
  jointStateMsg.position.size = 4;
  jointStateMsg.position.capacity = 4;
  jointStateMsg.velocity.data = jointVelocities;
  jointStateMsg.velocity.size = 4;
  jointStateMsg.velocity.capacity = 4;
  jointStateMsg.effort.data = jointEfforts;
  jointStateMsg.effort.size = 4;
  jointStateMsg.effort.capacity = 4;

  axisPositionMsg.layout.dim.size = 0;
  axisPositionMsg.layout.dim.capacity = 0;
  axisPositionMsg.layout.dim.data = nullptr;
  axisPositionMsg.layout.data_offset = 0;
  axisPositionMsg.data.data = axisPositionsMm;
  axisPositionMsg.data.size = 3;
  axisPositionMsg.data.capacity = 3;

  motorRpmMsg.layout.dim.size = 0;
  motorRpmMsg.layout.dim.capacity = 0;
  motorRpmMsg.layout.dim.data = nullptr;
  motorRpmMsg.layout.data_offset = 0;
  motorRpmMsg.data.data = motorRpms;
  motorRpmMsg.data.size = 4;
  motorRpmMsg.data.capacity = 4;

  setString(statusMsg.data, statusBuffer, sizeof(statusBuffer));
  setString(faultMsg.data, faultBuffer, sizeof(faultBuffer));
}

bool beginRosBridge() {
  if (rosReady) return true;

  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();

  if (rmw_uros_ping_agent(100, 3) != RCL_RET_OK) {
    return false;
  }

  initOptions = rcl_get_zero_initialized_init_options();
  if (!check(rcl_init_options_init(&initOptions, allocator))) return false;
  if (!check(rcl_init_options_set_domain_id(&initOptions, MICRO_ROS_DOMAIN_ID))) return false;
  if (!check(rclc_support_init_with_options(&support, 0, nullptr, &initOptions, &allocator))) return false;
  if (!check(rclc_node_init_default(&node, "palletizer_controller", "", &support))) return false;

  if (!check(rclc_publisher_init_default(
          &jointStatePublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          "/joint_states"))) return false;
  if (!check(rclc_publisher_init_default(
          &axisPositionPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/palletizer/axis_position_mm"))) return false;
  if (!check(rclc_publisher_init_default(
          &motorRpmPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/palletizer/motor_rpm"))) return false;
  if (!check(rclc_publisher_init_default(
          &statusPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/palletizer/status"))) return false;
  if (!check(rclc_publisher_init_default(
          &faultPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/palletizer/fault_state"))) return false;
  if (!check(rclc_subscription_init_best_effort(
          &emergencyStopSubscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
          "/palletizer/emergency_stop"))) return false;

  initMessages();

  if (!check(rclc_timer_init_default(
          &telemetryTimer,
          &support,
          RCL_MS_TO_NS(50),
          publishTelemetry))) return false;

  if (!check(rclc_executor_init(&executor, &support.context, 2, &allocator))) return false;
  if (!check(rclc_executor_add_timer(&executor, &telemetryTimer))) return false;
  if (!check(rclc_executor_add_subscription(
          &executor,
          &emergencyStopSubscriber,
          &emergencyStopMsg,
          onEmergencyStop,
          ON_NEW_DATA))) return false;

  rmw_uros_sync_session(1000);
  rosReady = true;
  return true;
}

bool rosBridgeReady() {
  return rosReady;
}

void spinRosBridge() {
  if (!rosReady) return;
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}

#else

bool beginRosBridge() {
  return true;
}

bool rosBridgeReady() {
  return true;
}

void spinRosBridge() {}

#endif
