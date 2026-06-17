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
#include "robot_tasks.h"

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
static bool initOptionsInitialized = false;
static bool supportInitialized = false;
static bool nodeInitialized = false;
static bool executorInitialized = false;
static bool telemetryTimerInitialized = false;
static bool jointStatePublisherInitialized = false;
static bool axisPositionPublisherInitialized = false;
static bool motorRpmPublisherInitialized = false;
static bool statusPublisherInitialized = false;
static bool faultPublisherInitialized = false;
static bool emergencyStopSubscriberInitialized = false;
static bool moveActionServerInitialized = false;
static uint32_t lastRosReconnectAttemptMs = 0;
static uint32_t lastRosHealthCheckMs = 0;
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
static uint32_t movePositionStableSinceMs = 0;
static bool moveResultPending = false;
static rcl_action_goal_state_t movePendingResultState = GOAL_STATE_UNKNOWN;
static bool movePendingResultSuccess = false;
static const char *movePendingResultMessage = "";
static uint32_t moveControlCommandId = 0;
static bool moveControlCommandAccepted = false;

static bool check(rcl_ret_t result) {
  return result == RCL_RET_OK;
}

static void ignoreRclRet(rcl_ret_t result) {
  (void)result;
}

static void zeroRosHandles() {
  initOptions = rcl_get_zero_initialized_init_options();
  support = {};
  node = rcl_get_zero_initialized_node();
  telemetryTimer = rcl_get_zero_initialized_timer();
  executor = rclc_executor_get_zero_initialized_executor();
  jointStatePublisher = rcl_get_zero_initialized_publisher();
  axisPositionPublisher = rcl_get_zero_initialized_publisher();
  motorRpmPublisher = rcl_get_zero_initialized_publisher();
  statusPublisher = rcl_get_zero_initialized_publisher();
  faultPublisher = rcl_get_zero_initialized_publisher();
  emergencyStopSubscriber = rcl_get_zero_initialized_subscription();
  moveActionServer = {};
}

static void resetRosRuntimeState() {
  activeMoveGoal = nullptr;
  moveResultPending = false;
  movePositionStableSinceMs = 0;
  moveControlCommandId = 0;
  moveControlCommandAccepted = false;
}

static void resetRosInitFlags() {
  initOptionsInitialized = false;
  supportInitialized = false;
  nodeInitialized = false;
  executorInitialized = false;
  telemetryTimerInitialized = false;
  jointStatePublisherInitialized = false;
  axisPositionPublisherInitialized = false;
  motorRpmPublisherInitialized = false;
  statusPublisherInitialized = false;
  faultPublisherInitialized = false;
  emergencyStopSubscriberInitialized = false;
  moveActionServerInitialized = false;
}

static void disconnectRosBridge() {
  rosReady = false;
  resetRosRuntimeState();

  if (executorInitialized) ignoreRclRet(rclc_executor_fini(&executor));
  if (moveActionServerInitialized) ignoreRclRet(rclc_action_server_fini(&moveActionServer, &node));
  if (emergencyStopSubscriberInitialized) ignoreRclRet(rcl_subscription_fini(&emergencyStopSubscriber, &node));
  if (faultPublisherInitialized) ignoreRclRet(rcl_publisher_fini(&faultPublisher, &node));
  if (statusPublisherInitialized) ignoreRclRet(rcl_publisher_fini(&statusPublisher, &node));
  if (motorRpmPublisherInitialized) ignoreRclRet(rcl_publisher_fini(&motorRpmPublisher, &node));
  if (axisPositionPublisherInitialized) ignoreRclRet(rcl_publisher_fini(&axisPositionPublisher, &node));
  if (jointStatePublisherInitialized) ignoreRclRet(rcl_publisher_fini(&jointStatePublisher, &node));
  if (telemetryTimerInitialized) ignoreRclRet(rcl_timer_fini(&telemetryTimer));
  if (nodeInitialized) ignoreRclRet(rcl_node_fini(&node));
  if (supportInitialized) ignoreRclRet(rclc_support_fini(&support));
  if (initOptionsInitialized) ignoreRclRet(rcl_init_options_fini(&initOptions));

  rcl_reset_error();
  resetRosInitFlags();
  zeroRosHandles();
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
  return robotCommandablePositionMm(mm);
}

static float maxMoveErrorMm() {
  RobotStateSnapshot state;
  if (!getRobotStateSnapshot(state) || !state.axisPositionsValid) return INFINITY;

  float maxError = fabsf(moveTargetX - state.axisPositionMm[0]);
  const float yError = fabsf(moveTargetY - state.axisPositionMm[1]);
  const float zError = fabsf(moveTargetZ - state.axisPositionMm[2]);
  if (yError > maxError) maxError = yError;
  if (zError > maxError) maxError = zError;
  return maxError;
}

static bool measuredPositionIsStableAtTarget() {
  RobotStateSnapshot state;
  if (!getRobotStateSnapshot(state) || !state.axisPositionsValid) {
    movePositionStableSinceMs = 0;
    return false;
  }

  const float settleTolerance = moveToleranceMm > ACTION_RESULT_MIN_TOLERANCE_MM
                                    ? moveToleranceMm
                                    : ACTION_RESULT_MIN_TOLERANCE_MM;
  if (maxMoveErrorMm() > settleTolerance) {
    movePositionStableSinceMs = 0;
    return false;
  }

  const uint32_t now = millis();
  if (movePositionStableSinceMs == 0) {
    movePositionStableSinceMs = now;
    return false;
  }
  return now - movePositionStableSinceMs >= ACTION_RESULT_POSITION_STABLE_MS;
}

static void fillMoveFeedback(const char *state) {
  RobotStateSnapshot snapshot;
  const bool haveSnapshot = getRobotStateSnapshot(snapshot);
  const float currentX = haveSnapshot ? snapshot.axisPositionMm[0] : 0.0f;
  const float currentY = haveSnapshot ? snapshot.axisPositionMm[1] : 0.0f;
  const float currentZ = haveSnapshot ? snapshot.axisPositionMm[2] : 0.0f;

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
    moveFeedbackMsg.feedback.progress = maxMoveErrorMm() <= moveToleranceMm ? 1.0f : 0.0f;
  } else {
    moveFeedbackMsg.feedback.progress = 1.0f - (remainingDistance / totalDistance);
    if (moveFeedbackMsg.feedback.progress < 0.0f) moveFeedbackMsg.feedback.progress = 0.0f;
    if (moveFeedbackMsg.feedback.progress > 1.0f) moveFeedbackMsg.feedback.progress = 1.0f;
  }

  setMoveFeedbackState(state);
}

static rcl_ret_t finishMoveGoal(rcl_action_goal_state_t state, bool success, const char *message) {
  if (!activeMoveGoal) return RCL_RET_ERROR;

  RobotStateSnapshot snapshot;
  getRobotStateSnapshot(snapshot);

  moveResultResponse.status = state;
  moveResultResponse.result.success = success;
  moveResultResponse.result.final_x_mm = snapshot.axisPositionMm[0];
  moveResultResponse.result.final_y_mm = snapshot.axisPositionMm[1];
  moveResultResponse.result.final_z_mm = snapshot.axisPositionMm[2];
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

  RobotStateSnapshot snapshot;
  getRobotStateSnapshot(snapshot);

  for (size_t i = 0; i < 4; i++) {
    jointPositions[i] = snapshot.encoderOk[i]
                            ? static_cast<double>(snapshot.motorPositionMm[i]) / 1000.0
                            : 0.0;
    jointVelocities[i] = snapshot.rpmOk[i]
                             ? static_cast<double>(snapshot.motorRpm[i]) * LEADSCREW_MM_PER_REV / 60000.0
                             : 0.0;
    jointEfforts[i] = 0.0;
    motorRpms[i] = snapshot.rpmOk[i] ? static_cast<float>(snapshot.motorRpm[i]) : 0.0f;
  }

  axisPositionsMm[0] = snapshot.axisPositionsValid ? snapshot.axisPositionMm[0] : 0.0f;
  axisPositionsMm[1] = snapshot.axisPositionsValid ? snapshot.axisPositionMm[1] : 0.0f;
  axisPositionsMm[2] = snapshot.axisPositionsValid ? snapshot.axisPositionMm[2] : 0.0f;

  const int written = snprintf(
      statusBuffer,
      sizeof(statusBuffer),
      "{\"fault\":%u,\"reason\":\"%s\",\"online\":[%u,%u,%u,%u],\"enc\":[%lld,%lld,%lld,%lld],\"mm\":[%.3f,%.3f,%.3f,%.3f],\"rpm\":[%d,%d,%d,%d],\"acc\":[%u,%u,%u,%u],\"moveStatus\":[%u,%u,%u,%u],\"homeStatus\":[%u,%u,%u,%u]}",
      snapshot.safetyFault ? 1 : 0,
      snapshot.safetyReason,
      snapshot.motorOnline[0] ? 1 : 0,
      snapshot.motorOnline[1] ? 1 : 0,
      snapshot.motorOnline[2] ? 1 : 0,
      snapshot.motorOnline[3] ? 1 : 0,
      static_cast<long long>(snapshot.encoder[0]),
      static_cast<long long>(snapshot.encoder[1]),
      static_cast<long long>(snapshot.encoder[2]),
      static_cast<long long>(snapshot.encoder[3]),
      snapshot.motorPositionMm[0],
      snapshot.motorPositionMm[1],
      snapshot.motorPositionMm[2],
      snapshot.motorPositionMm[3],
      snapshot.motorRpm[0],
      snapshot.motorRpm[1],
      snapshot.motorRpm[2],
      snapshot.motorRpm[3],
      snapshot.lastAcc[0],
      snapshot.lastAcc[1],
      snapshot.lastAcc[2],
      snapshot.lastAcc[3],
      snapshot.moveStatus[0],
      snapshot.moveStatus[1],
      snapshot.moveStatus[2],
      snapshot.moveStatus[3],
      snapshot.homeStatus[0],
      snapshot.homeStatus[1],
      snapshot.homeStatus[2],
      snapshot.homeStatus[3]);
  statusMsg.data.size = written > 0 ? min(static_cast<size_t>(written), sizeof(statusBuffer) - 1) : 0;

  const int faultWritten = snprintf(
      faultBuffer,
      sizeof(faultBuffer),
      "%s:%s",
      snapshot.safetyFault ? "FAULT" : "OK",
      snapshot.safetyReason);
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
      robotRequestStop();
      fillMoveFeedback("canceled");
      rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
      queueMoveResult(GOAL_STATE_CANCELED, false, "goal canceled");
      return;
    }

    RobotStateSnapshot snapshot;
    getRobotStateSnapshot(snapshot);

    if (snapshot.safetyFault) {
      fillMoveFeedback("fault");
      rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
      queueMoveResult(GOAL_STATE_ABORTED, false, snapshot.safetyReason);
      return;
    }

    if (!moveControlCommandAccepted) {
      bool known = false;
      bool accepted = false;
      robotGetMoveCommandStatus(moveControlCommandId, known, accepted);
      if (!known) {
        fillMoveFeedback("queued");
        rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
        return;
      }
      if (!accepted) {
        fillMoveFeedback("rejected_by_control");
        rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
        queueMoveResult(GOAL_STATE_ABORTED, false, "control rejected move command");
        return;
      }
      moveControlCommandAccepted = true;
    }

    if (millis() - moveStartedMs > moveTimeoutMs) {
      robotRequestStop();
      fillMoveFeedback("timeout");
      rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
      queueMoveResult(GOAL_STATE_ABORTED, false, "timeout before reaching target");
      return;
    }

    if (measuredPositionIsStableAtTarget()) {
      fillMoveFeedback("arrived");
      moveFeedbackMsg.feedback.progress = 1.0f;
      rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
      queueMoveResult(GOAL_STATE_SUCCEEDED, true, "target reached");
      return;
    }

    fillMoveFeedback(snapshot.axisPositionsValid ? "moving" : "waiting_for_encoders");
    rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
  }
}

static void onEmergencyStop(const void *msgIn) {
  const std_msgs__msg__Bool *msg = static_cast<const std_msgs__msg__Bool *>(msgIn);
  if (msg->data) {
    robotRequestEmergencyStop();
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
  movePositionStableSinceMs = 0;
  moveResultPending = false;
  moveControlCommandId = 0;
  moveControlCommandAccepted = false;

  RobotStateSnapshot snapshot;
  getRobotStateSnapshot(snapshot);
  moveStartX = snapshot.axisPositionMm[0];
  moveStartY = snapshot.axisPositionMm[1];
  moveStartZ = snapshot.axisPositionMm[2];

  const bool commandQueued = robotRequestMoveXYZMm(
      requestedX,
      requestedY,
      requestedZ,
      request->goal.speed_mm_s,
      request->goal.accel_mm_s2,
      moveControlCommandId);

  if (!commandQueued) {
    return RCL_RET_ACTION_GOAL_REJECTED;
  }

  activeMoveGoal = goalHandle;
  fillMoveFeedback("accepted");
  rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg);
  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

static bool onMoveCancel(rclc_action_goal_handle_t *goalHandle, void *) {
  if (goalHandle != activeMoveGoal) return false;
  robotRequestStop();
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

  if (rmw_uros_ping_agent(MICRO_ROS_PING_TIMEOUT_MS, MICRO_ROS_PING_ATTEMPTS) != RCL_RET_OK) {
    return false;
  }

  zeroRosHandles();
  resetRosInitFlags();
  resetRosRuntimeState();

  initOptions = rcl_get_zero_initialized_init_options();
  if (!check(rcl_init_options_init(&initOptions, allocator))) {
    disconnectRosBridge();
    return false;
  }
  initOptionsInitialized = true;
  if (!check(rcl_init_options_set_domain_id(&initOptions, MICRO_ROS_DOMAIN_ID))) {
    disconnectRosBridge();
    return false;
  }
  if (!check(rclc_support_init_with_options(&support, 0, nullptr, &initOptions, &allocator))) {
    disconnectRosBridge();
    return false;
  }
  supportInitialized = true;
  if (!check(rclc_node_init_default(&node, "palletizer_controller", "", &support))) {
    disconnectRosBridge();
    return false;
  }
  nodeInitialized = true;

  if (!check(rclc_publisher_init_default(
          &jointStatePublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          "/joint_states"))) {
    disconnectRosBridge();
    return false;
  }
  jointStatePublisherInitialized = true;
  if (!check(rclc_publisher_init_default(
          &axisPositionPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/palletizer/axis_position_mm"))) {
    disconnectRosBridge();
    return false;
  }
  axisPositionPublisherInitialized = true;
  if (!check(rclc_publisher_init_default(
          &motorRpmPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/palletizer/motor_rpm"))) {
    disconnectRosBridge();
    return false;
  }
  motorRpmPublisherInitialized = true;
  if (!check(rclc_publisher_init_default(
          &statusPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/palletizer/status"))) {
    disconnectRosBridge();
    return false;
  }
  statusPublisherInitialized = true;
  if (!check(rclc_publisher_init_default(
          &faultPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/palletizer/fault_state"))) {
    disconnectRosBridge();
    return false;
  }
  faultPublisherInitialized = true;
  if (!check(rclc_subscription_init_best_effort(
          &emergencyStopSubscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
          "/palletizer/emergency_stop"))) {
    disconnectRosBridge();
    return false;
  }
  emergencyStopSubscriberInitialized = true;
  if (!check(rclc_action_server_init_default(
          &moveActionServer,
          &node,
          &support,
          ROSIDL_GET_ACTION_TYPE_SUPPORT(palletizer_msgs, MoveXYZ),
          "/palletizer/move_xyz"))) {
    disconnectRosBridge();
    return false;
  }
  moveActionServerInitialized = true;

  initMessages();

  if (!check(rclc_timer_init_default(
          &telemetryTimer,
          &support,
          RCL_MS_TO_NS(50),
          publishTelemetry))) {
    disconnectRosBridge();
    return false;
  }
  telemetryTimerInitialized = true;

  if (!check(rclc_executor_init(&executor, &support.context, 3, &allocator))) {
    disconnectRosBridge();
    return false;
  }
  executorInitialized = true;
  if (!check(rclc_executor_add_timer(&executor, &telemetryTimer))) {
    disconnectRosBridge();
    return false;
  }
  if (!check(rclc_executor_add_subscription(
          &executor,
          &emergencyStopSubscriber,
          &emergencyStopMsg,
          onEmergencyStop,
          ON_NEW_DATA))) {
    disconnectRosBridge();
    return false;
  }
  if (!check(rclc_executor_add_action_server(
          &executor,
          &moveActionServer,
          1,
          &moveGoalRequest,
          sizeof(moveGoalRequest),
          onMoveGoal,
          onMoveCancel,
          nullptr))) {
    disconnectRosBridge();
    return false;
  }

  rmw_uros_sync_session(1000);
  lastRosHealthCheckMs = millis();
  rosReady = true;
  return true;
}

bool rosBridgeReady() {
  return rosReady;
}

void manageRosBridge() {
  const uint32_t now = millis();
  if (!rosReady) {
    if (now - lastRosReconnectAttemptMs >= MICRO_ROS_RECONNECT_PERIOD_MS) {
      lastRosReconnectAttemptMs = now;
      beginRosBridge();
    }
    return;
  }

  if (now - lastRosHealthCheckMs < MICRO_ROS_HEALTH_CHECK_PERIOD_MS) return;
  lastRosHealthCheckMs = now;

  if (rmw_uros_ping_agent(MICRO_ROS_PING_TIMEOUT_MS, MICRO_ROS_PING_ATTEMPTS) != RCL_RET_OK) {
    disconnectRosBridge();
    lastRosReconnectAttemptMs = now;
  }
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

void manageRosBridge() {}

void spinRosBridge() {}

#endif
