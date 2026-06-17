#include "ros_bridge.h"

#if ENABLE_MICRO_ROS

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rclc/action_goal_handle.h>
#include <rclc/action_server.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <palletizer_msgs/action/move_xyz.h>
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
static rclc_action_server_t moveActionServer;

static sensor_msgs__msg__JointState jointStateMsg;
static std_msgs__msg__Float32MultiArray axisPositionMsg;
static std_msgs__msg__Float32MultiArray motorRpmMsg;
static std_msgs__msg__String statusMsg;
static std_msgs__msg__String faultMsg;
static std_msgs__msg__Bool emergencyStopMsg;
static palletizer_msgs__action__MoveXYZ_SendGoal_Request moveGoalRequest;
static palletizer_msgs__action__MoveXYZ_FeedbackMessage moveFeedbackMsg;
static palletizer_msgs__action__MoveXYZ_GetResult_Response moveResultResponse;

static rosidl_runtime_c__String jointNames[4];
static double jointPositions[4];
static double jointVelocities[4];
static double jointEfforts[4];
static float axisPositionsMm[3];
static float motorRpms[4];
static char statusBuffer[512];
static char faultBuffer[96];
static char moveFeedbackStateBuffer[96];
static char moveResultMessageBuffer[96];
static char frameIdBuffer[] = "palletizer_base";
static char jointNameBuffers[4][4] = {"X1", "X2", "Y", "Z"};

static bool rosReady = false;
static rclc_action_goal_handle_t *activeMoveGoal = nullptr;
static float moveTargetX = 0.0f;
static float moveTargetY = 0.0f;
static float moveTargetZ = 0.0f;
static float moveStartX = 0.0f;
static float moveStartY = 0.0f;
static float moveStartZ = 0.0f;
static float moveToleranceMm = 1.0f;
static uint32_t moveStartedMs = 0;
static uint32_t moveTimeoutMs = 30000;
static bool moveResultPending = false;
static rcl_action_goal_state_t movePendingResultState = GOAL_STATE_UNKNOWN;
static bool movePendingResultSuccess = false;
static const char *movePendingResultMessage = "";

static bool check(rcl_ret_t result) {
  return result == RCL_RET_OK;
}

static void setString(rosidl_runtime_c__String &text, char *buffer, size_t capacity, size_t size = 0) {
  text.data = buffer;
  text.capacity = capacity;
  text.size = size;
}

static void setMoveFeedbackState(const char *state) {
  const int written = snprintf(moveFeedbackStateBuffer, sizeof(moveFeedbackStateBuffer), "%s", state);
  moveFeedbackMsg.feedback.state.size =
      written > 0 ? min(static_cast<size_t>(written), sizeof(moveFeedbackStateBuffer) - 1) : 0;
}

static void setMoveResultMessage(const char *message) {
  const int written = snprintf(moveResultMessageBuffer, sizeof(moveResultMessageBuffer), "%s", message);
  moveResultResponse.result.message.size =
      written > 0 ? min(static_cast<size_t>(written), sizeof(moveResultMessageBuffer) - 1) : 0;
}

static float distance3(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

static float commandablePositionMm(float mm) {
  return encoderCountsToMm(mmToEncoderCounts(mm));
}

static void fillMoveFeedback(const char *state) {
  float currentX = 0.0f;
  float currentY = 0.0f;
  float currentZ = 0.0f;
  getAxisPositionsMm(currentX, currentY, currentZ);

  moveFeedbackMsg.feedback.current_x_mm = currentX;
  moveFeedbackMsg.feedback.current_y_mm = currentY;
  moveFeedbackMsg.feedback.current_z_mm = currentZ;
  moveFeedbackMsg.feedback.error_x_mm = moveTargetX - currentX;
  moveFeedbackMsg.feedback.error_y_mm = moveTargetY - currentY;
  moveFeedbackMsg.feedback.error_z_mm = moveTargetZ - currentZ;

  const float totalDistance = distance3(moveTargetX - moveStartX, moveTargetY - moveStartY, moveTargetZ - moveStartZ);
  const float remainingDistance = distance3(
      moveFeedbackMsg.feedback.error_x_mm,
      moveFeedbackMsg.feedback.error_y_mm,
      moveFeedbackMsg.feedback.error_z_mm);
  if (totalDistance <= 0.001f) {
    moveFeedbackMsg.feedback.progress = isAtXYZMm(moveTargetX, moveTargetY, moveTargetZ, moveToleranceMm) ? 1.0f : 0.0f;
  } else {
    moveFeedbackMsg.feedback.progress = 1.0f - (remainingDistance / totalDistance);
    if (moveFeedbackMsg.feedback.progress < 0.0f) moveFeedbackMsg.feedback.progress = 0.0f;
    if (moveFeedbackMsg.feedback.progress > 1.0f) moveFeedbackMsg.feedback.progress = 1.0f;
  }

  setMoveFeedbackState(state);
}

static rcl_ret_t finishMoveGoal(rcl_action_goal_state_t state, bool success, const char *message) {
  if (!activeMoveGoal) return RCL_RET_ERROR;

  float finalX = 0.0f;
  float finalY = 0.0f;
  float finalZ = 0.0f;
  getAxisPositionsMm(finalX, finalY, finalZ);

  moveResultResponse.status = state;
  moveResultResponse.result.success = success;
  moveResultResponse.result.final_x_mm = finalX;
  moveResultResponse.result.final_y_mm = finalY;
  moveResultResponse.result.final_z_mm = finalZ;
  setMoveResultMessage(message);

  const rcl_ret_t result = rclc_action_send_result(activeMoveGoal, state, &moveResultResponse);
  if (result == RCL_RET_OK) {
    activeMoveGoal = nullptr;
    moveResultPending = false;
  }
  return result;
}

static void queueMoveResult(rcl_action_goal_state_t state, bool success, const char *message) {
  moveResultPending = true;
  movePendingResultState = state;
  movePendingResultSuccess = success;
  movePendingResultMessage = message;
  finishMoveGoal(state, success, message);
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
      const float mm = encoderCountsToMm(motor.encoder);
      jointPositions[i] = static_cast<double>(mm) / 1000.0;
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
                             ? static_cast<double>(motor.rpm) * LEADSCREW_MM_PER_REV / 60000.0
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

  if (activeMoveGoal) {
    if (moveResultPending) {
      fillMoveFeedback(movePendingResultSuccess ? "result_pending" : "terminal_result_pending");
      if (movePendingResultSuccess) moveFeedbackMsg.feedback.progress = 1.0f;
      rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
      finishMoveGoal(movePendingResultState, movePendingResultSuccess, movePendingResultMessage);
      return;
    }

    if (activeMoveGoal->goal_cancelled) {
      stopAllMotors();
      fillMoveFeedback("canceled");
      rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
      queueMoveResult(GOAL_STATE_CANCELED, false, "goal canceled");
      return;
    }

    if (safetyFaultIsActive()) {
      fillMoveFeedback("fault");
      rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
      queueMoveResult(GOAL_STATE_ABORTED, false, safetyFaultText());
      return;
    }

    if (millis() - moveStartedMs > moveTimeoutMs) {
      stopAllMotors();
      fillMoveFeedback("timeout");
      rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
      queueMoveResult(GOAL_STATE_ABORTED, false, "timeout before reaching target");
      return;
    }

    if (isAtXYZMm(moveTargetX, moveTargetY, moveTargetZ, moveToleranceMm)) {
      fillMoveFeedback("arrived");
      moveFeedbackMsg.feedback.progress = 1.0f;
      rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
      queueMoveResult(GOAL_STATE_SUCCEEDED, true, "target reached");
      return;
    }

    fillMoveFeedback(axisPositionsAreValid() ? "moving" : "waiting_for_encoders");
    rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
  }
}

static void onEmergencyStop(const void *msgIn) {
  const std_msgs__msg__Bool *msg = static_cast<const std_msgs__msg__Bool *>(msgIn);
  if (msg->data) {
    stopAllMotors();
  }
}

static rcl_ret_t onMoveGoal(rclc_action_goal_handle_t *goalHandle, void *) {
  if (activeMoveGoal) {
    return RCL_RET_ACTION_GOAL_REJECTED;
  }

  palletizer_msgs__action__MoveXYZ_SendGoal_Request *request =
      reinterpret_cast<palletizer_msgs__action__MoveXYZ_SendGoal_Request *>(goalHandle->ros_goal_request);

  const float requestedX = request->goal.x_mm;
  const float requestedY = request->goal.y_mm;
  const float requestedZ = request->goal.z_mm;

  moveTargetX = commandablePositionMm(requestedX);
  moveTargetY = commandablePositionMm(requestedY);
  moveTargetZ = commandablePositionMm(requestedZ);
  moveToleranceMm = request->goal.tolerance_mm > 0.0f ? request->goal.tolerance_mm : 1.0f;
  moveTimeoutMs = request->goal.timeout_ms > 0 ? request->goal.timeout_ms : 30000;
  moveStartedMs = millis();
  moveResultPending = false;
  getAxisPositionsMm(moveStartX, moveStartY, moveStartZ);

  const bool commandAccepted = commandMoveXYZMm(
      requestedX,
      requestedY,
      requestedZ,
      request->goal.speed_mm_s,
      request->goal.accel_mm_s2);

  if (!commandAccepted) {
    return RCL_RET_ACTION_GOAL_REJECTED;
  }

  activeMoveGoal = goalHandle;
  fillMoveFeedback("accepted");
  rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

static bool onMoveCancel(rclc_action_goal_handle_t *goalHandle, void *) {
  if (goalHandle != activeMoveGoal) return false;
  stopAllMotors();
  return true;
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
  setString(moveFeedbackMsg.feedback.state, moveFeedbackStateBuffer, sizeof(moveFeedbackStateBuffer));
  setString(moveResultResponse.result.message, moveResultMessageBuffer, sizeof(moveResultMessageBuffer));
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
  if (!check(rclc_action_server_init_default(
          &moveActionServer,
          &node,
          &support,
          ROSIDL_GET_ACTION_TYPE_SUPPORT(palletizer_msgs, MoveXYZ),
          "/palletizer/move_xyz"))) return false;

  initMessages();

  if (!check(rclc_timer_init_default(
          &telemetryTimer,
          &support,
          RCL_MS_TO_NS(50),
          publishTelemetry))) return false;

  if (!check(rclc_executor_init(&executor, &support.context, 3, &allocator))) return false;
  if (!check(rclc_executor_add_timer(&executor, &telemetryTimer))) return false;
  if (!check(rclc_executor_add_subscription(
          &executor,
          &emergencyStopSubscriber,
          &emergencyStopMsg,
          onEmergencyStop,
          ON_NEW_DATA))) return false;
  if (!check(rclc_executor_add_action_server(
          &executor,
          &moveActionServer,
          1,
          &moveGoalRequest,
          sizeof(moveGoalRequest),
          onMoveGoal,
          onMoveCancel,
          nullptr))) return false;

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
