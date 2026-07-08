#include "ros_bridge.h"

#if ENABLE_MICRO_ROS

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/context.h>
#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rclc/action_goal_handle.h>
#include <rclc/action_server.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/timing.h>
#include <palletizer_msgs/action/go_origin.h>
#include <palletizer_msgs/action/home_axis.h>
#include <palletizer_msgs/action/move_xyz.h>
#include <palletizer_msgs/srv/clear_fault.h>
#include <palletizer_msgs/srv/enable_axis.h>
#include <palletizer_msgs/srv/get_driver_status.h>
#include <palletizer_msgs/srv/release_stall.h>
#include <palletizer_msgs/srv/set_axis_limits.h>
#include <palletizer_msgs/srv/set_gripper.h>
#include <palletizer_msgs/srv/set_zero.h>
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
static rcl_timer_t statusTelemetryTimer;
static rcl_timer_t actionTimer;
static rclc_executor_t executor;

static rcl_publisher_t jointStatePublisher;
static rcl_publisher_t axisPositionPublisher;
static rcl_publisher_t motorRpmPublisher;
static rcl_publisher_t statusPublisher;
static rcl_publisher_t faultPublisher;
static rcl_subscription_t emergencyStopSubscriber;
static rcl_subscription_t commandSubscriber;
static rcl_subscription_t fastMoveSubscriber;
static rcl_service_t enableAxisService;
static rcl_service_t setZeroService;
static rcl_service_t setAxisLimitsService;
static rcl_service_t clearFaultService;
static rcl_service_t releaseStallService;
static rcl_service_t getDriverStatusService;
static rcl_service_t setGripperService;
static rclc_action_server_t moveActionServer;
static rclc_action_server_t homeActionServer;
static rclc_action_server_t originActionServer;

static sensor_msgs__msg__JointState jointStateMsg;
static std_msgs__msg__Float32MultiArray axisPositionMsg;
static std_msgs__msg__Float32MultiArray motorRpmMsg;
static std_msgs__msg__String statusMsg;
static std_msgs__msg__String faultMsg;
static std_msgs__msg__Bool emergencyStopMsg;
static std_msgs__msg__String commandMsg;
static std_msgs__msg__Float32MultiArray fastMoveMsg;
static palletizer_msgs__srv__EnableAxis_Request enableAxisRequest;
static palletizer_msgs__srv__EnableAxis_Response enableAxisResponse;
static palletizer_msgs__srv__SetZero_Request setZeroRequest;
static palletizer_msgs__srv__SetZero_Response setZeroResponse;
static palletizer_msgs__srv__SetAxisLimits_Request setAxisLimitsRequest;
static palletizer_msgs__srv__SetAxisLimits_Response setAxisLimitsResponse;
static palletizer_msgs__srv__ClearFault_Request clearFaultRequest;
static palletizer_msgs__srv__ClearFault_Response clearFaultResponse;
static palletizer_msgs__srv__ReleaseStall_Request releaseStallRequest;
static palletizer_msgs__srv__ReleaseStall_Response releaseStallResponse;
static palletizer_msgs__srv__GetDriverStatus_Request getDriverStatusRequest;
static palletizer_msgs__srv__GetDriverStatus_Response getDriverStatusResponse;
static palletizer_msgs__srv__SetGripper_Request setGripperRequest;
static palletizer_msgs__srv__SetGripper_Response setGripperResponse;
static palletizer_msgs__action__MoveXYZ_SendGoal_Request moveGoalRequest;
static palletizer_msgs__action__MoveXYZ_FeedbackMessage moveFeedbackMsg;
static palletizer_msgs__action__MoveXYZ_GetResult_Response moveResultResponse;
static palletizer_msgs__action__HomeAxis_SendGoal_Request homeGoalRequest;
static palletizer_msgs__action__HomeAxis_FeedbackMessage homeFeedbackMsg;
static palletizer_msgs__action__HomeAxis_GetResult_Response homeResultResponse;
static palletizer_msgs__action__GoOrigin_SendGoal_Request originGoalRequest;
static palletizer_msgs__action__GoOrigin_FeedbackMessage originFeedbackMsg;
static palletizer_msgs__action__GoOrigin_GetResult_Response originResultResponse;

static rosidl_runtime_c__String jointNames[ROBOT_MOTOR_COUNT];
static double jointPositions[ROBOT_MOTOR_COUNT];
static double jointVelocities[ROBOT_MOTOR_COUNT];
static double jointEfforts[ROBOT_MOTOR_COUNT];
static float axisPositionsMm[ROBOT_LINEAR_AXIS_COUNT];
static float motorRpms[ROBOT_MOTOR_COUNT];
static float fastMoveData[5];
static char statusBuffer[2300];
static char faultBuffer[96];
static char commandBuffer[128];
static char serviceMessageBuffers[7][96];
static char driverStatusMessageBuffer[96];
static char driverStatusSafetyReasonBuffer[96];
static char driverStatusHomingStateBuffer[48];
static char moveFeedbackStateBuffer[96];
static char moveResultMessageBuffer[96];
static char homeFeedbackStateBuffer[96];
static char homeResultMessageBuffer[96];
static char originFeedbackStateBuffer[96];
static char originResultMessageBuffer[96];
static char frameIdBuffer[] = "palletizer_base";
static char jointNameBuffers[ROBOT_MOTOR_COUNT][4] = {"X1", "X2", "Y", "Z", "A"};

static constexpr size_t ROS_EXECUTOR_TIMERS = 3;
// Keep the entity set small enough for reliable creation over USB CDC.
// Absolute movements remain available through /palletizer/move_xyz.
static constexpr bool ROS_ENABLE_COMMAND_SUBSCRIBER = false;
static constexpr bool ROS_ENABLE_FAST_MOVE_SUBSCRIBER = false;
static constexpr size_t ROS_EXECUTOR_SUBSCRIPTIONS = 1 +
                                                       (ROS_ENABLE_COMMAND_SUBSCRIBER ? 1 : 0) +
                                                       (ROS_ENABLE_FAST_MOVE_SUBSCRIBER ? 1 : 0);
static constexpr bool ROS_ENABLE_SERVICE_SERVERS = false;
// Keep the stable micro-ROS profile below the ESP32/XRCE entity limit.
// The UI reports the auxiliary servo control as unavailable in this profile.
static constexpr bool ROS_ENABLE_GRIPPER_SERVICE = false;
static constexpr bool ROS_ENABLE_EXTENDED_SERVICE_SERVERS = false;
static constexpr size_t ROS_EXECUTOR_SERVICES = (ROS_ENABLE_SERVICE_SERVERS
                                                   ? (ROS_ENABLE_EXTENDED_SERVICE_SERVERS ? 6 : 2)
                                                   : 0) +
                                                   (ROS_ENABLE_GRIPPER_SERVICE ? 1 : 0);
static constexpr bool ROS_ENABLE_HOME_ORIGIN_ACTIONS = false;
static constexpr size_t ROS_EXECUTOR_ACTIONS = ROS_ENABLE_HOME_ORIGIN_ACTIONS ? 3 : 1;
static constexpr size_t ROS_EXECUTOR_SPARES = 2;
static constexpr size_t ROS_EXECUTOR_HANDLES =
    ROS_EXECUTOR_TIMERS +
    ROS_EXECUTOR_SUBSCRIPTIONS +
    ROS_EXECUTOR_SERVICES +
    ROS_EXECUTOR_ACTIONS +
    ROS_EXECUTOR_SPARES;

static bool rosReady = false;
static bool initOptionsInitialized = false;
static bool supportInitialized = false;
static bool nodeInitialized = false;
static bool executorInitialized = false;
static bool telemetryTimerInitialized = false;
static bool statusTelemetryTimerInitialized = false;
static bool actionTimerInitialized = false;
static bool jointStatePublisherInitialized = false;
static bool axisPositionPublisherInitialized = false;
static bool motorRpmPublisherInitialized = false;
static bool statusPublisherInitialized = false;
static bool faultPublisherInitialized = false;
static bool emergencyStopSubscriberInitialized = false;
static bool commandSubscriberInitialized = false;
static bool fastMoveSubscriberInitialized = false;
static bool enableAxisServiceInitialized = false;
static bool setZeroServiceInitialized = false;
static bool setAxisLimitsServiceInitialized = false;
static bool clearFaultServiceInitialized = false;
static bool releaseStallServiceInitialized = false;
static bool getDriverStatusServiceInitialized = false;
static bool setGripperServiceInitialized = false;
static bool moveActionServerInitialized = false;
static bool homeActionServerInitialized = false;
static bool originActionServerInitialized = false;
static uint32_t lastRosReconnectAttemptMs = 0;
static uint32_t lastRosHealthCheckMs = 0;
static rclc_action_goal_handle_t *activeMoveGoal = nullptr;
static float moveTargetX = 0.0f;
static float moveTargetY = 0.0f;
static float moveTargetZ = 0.0f;
static float moveTargetA = 0.0f;
static float moveStartX = 0.0f;
static float moveStartY = 0.0f;
static float moveStartZ = 0.0f;
static float moveStartA = 0.0f;
static float moveToleranceMm = 1.0f;
static float moveAngularToleranceDeg = 1.0f;
static bool moveUseA = false;
static uint32_t moveStartedMs = 0;
static uint32_t moveTimeoutMs = 30000;
static uint32_t movePositionStableSinceMs = 0;
static uint32_t lastMoveFeedbackMs = 0;
static bool moveResultPending = false;
static rcl_action_goal_state_t movePendingResultState = GOAL_STATE_UNKNOWN;
static bool movePendingResultSuccess = false;
static char movePendingResultMessage[96] = "";
static uint32_t moveControlCommandId = 0;
static bool moveControlCommandAccepted = false;
static rclc_action_goal_handle_t *activeHomeGoal = nullptr;
static uint8_t homeAxis = 0;
static uint32_t homeStartedMs = 0;
static uint32_t homeTimeoutMs = 60000;
static uint32_t homeControlCommandId = 0;
static bool homeControlCommandAccepted = false;
static bool homeResultPending = false;
static rcl_action_goal_state_t homePendingResultState = GOAL_STATE_UNKNOWN;
static bool homePendingResultSuccess = false;
static char homePendingResultMessage[96] = "";
static rclc_action_goal_handle_t *activeOriginGoal = nullptr;
static uint8_t originAxis = 0;
static float originToleranceMm = 1.0f;
static uint32_t originStartedMs = 0;
static uint32_t originTimeoutMs = 30000;
static uint32_t originStableSinceMs = 0;
static uint32_t originControlCommandId = 0;
static bool originControlCommandAccepted = false;
static float originStartErrorMm = 0.0f;
static bool originResultPending = false;
static rcl_action_goal_state_t originPendingResultState = GOAL_STATE_UNKNOWN;
static bool originPendingResultSuccess = false;
static char originPendingResultMessage[96] = "";

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
  statusTelemetryTimer = rcl_get_zero_initialized_timer();
  actionTimer = rcl_get_zero_initialized_timer();
  executor = rclc_executor_get_zero_initialized_executor();
  jointStatePublisher = rcl_get_zero_initialized_publisher();
  axisPositionPublisher = rcl_get_zero_initialized_publisher();
  motorRpmPublisher = rcl_get_zero_initialized_publisher();
  statusPublisher = rcl_get_zero_initialized_publisher();
  faultPublisher = rcl_get_zero_initialized_publisher();
  emergencyStopSubscriber = rcl_get_zero_initialized_subscription();
  commandSubscriber = rcl_get_zero_initialized_subscription();
  fastMoveSubscriber = rcl_get_zero_initialized_subscription();
  enableAxisService = rcl_get_zero_initialized_service();
  setZeroService = rcl_get_zero_initialized_service();
  setAxisLimitsService = rcl_get_zero_initialized_service();
  clearFaultService = rcl_get_zero_initialized_service();
  releaseStallService = rcl_get_zero_initialized_service();
  getDriverStatusService = rcl_get_zero_initialized_service();
  setGripperService = rcl_get_zero_initialized_service();
  moveActionServer = {};
  homeActionServer = {};
  originActionServer = {};
}

static void resetRosRuntimeState() {
  activeMoveGoal = nullptr;
  activeHomeGoal = nullptr;
  activeOriginGoal = nullptr;
  moveResultPending = false;
  homeResultPending = false;
  originResultPending = false;
  movePositionStableSinceMs = 0;
  lastMoveFeedbackMs = 0;
  moveControlCommandId = 0;
  moveControlCommandAccepted = false;
  homeControlCommandId = 0;
  homeControlCommandAccepted = false;
  originControlCommandId = 0;
  originControlCommandAccepted = false;
  originStableSinceMs = 0;
}

static void resetRosInitFlags() {
  initOptionsInitialized = false;
  supportInitialized = false;
  nodeInitialized = false;
  executorInitialized = false;
  telemetryTimerInitialized = false;
  statusTelemetryTimerInitialized = false;
  actionTimerInitialized = false;
  jointStatePublisherInitialized = false;
  axisPositionPublisherInitialized = false;
  motorRpmPublisherInitialized = false;
  statusPublisherInitialized = false;
  faultPublisherInitialized = false;
  emergencyStopSubscriberInitialized = false;
  commandSubscriberInitialized = false;
  fastMoveSubscriberInitialized = false;
  enableAxisServiceInitialized = false;
  setZeroServiceInitialized = false;
  setAxisLimitsServiceInitialized = false;
  clearFaultServiceInitialized = false;
  releaseStallServiceInitialized = false;
  getDriverStatusServiceInitialized = false;
  setGripperServiceInitialized = false;
  moveActionServerInitialized = false;
  homeActionServerInitialized = false;
  originActionServerInitialized = false;
}

static void disconnectRosBridge() {
  rosReady = false;
  resetRosRuntimeState();

  if (executorInitialized) ignoreRclRet(rclc_executor_fini(&executor));
  if (originActionServerInitialized) ignoreRclRet(rclc_action_server_fini(&originActionServer, &node));
  if (homeActionServerInitialized) ignoreRclRet(rclc_action_server_fini(&homeActionServer, &node));
  if (moveActionServerInitialized) ignoreRclRet(rclc_action_server_fini(&moveActionServer, &node));
  if (setGripperServiceInitialized) ignoreRclRet(rcl_service_fini(&setGripperService, &node));
  if (getDriverStatusServiceInitialized) ignoreRclRet(rcl_service_fini(&getDriverStatusService, &node));
  if (releaseStallServiceInitialized) ignoreRclRet(rcl_service_fini(&releaseStallService, &node));
  if (clearFaultServiceInitialized) ignoreRclRet(rcl_service_fini(&clearFaultService, &node));
  if (setAxisLimitsServiceInitialized) ignoreRclRet(rcl_service_fini(&setAxisLimitsService, &node));
  if (setZeroServiceInitialized) ignoreRclRet(rcl_service_fini(&setZeroService, &node));
  if (enableAxisServiceInitialized) ignoreRclRet(rcl_service_fini(&enableAxisService, &node));
  if (fastMoveSubscriberInitialized) ignoreRclRet(rcl_subscription_fini(&fastMoveSubscriber, &node));
  if (commandSubscriberInitialized) ignoreRclRet(rcl_subscription_fini(&commandSubscriber, &node));
  if (emergencyStopSubscriberInitialized) ignoreRclRet(rcl_subscription_fini(&emergencyStopSubscriber, &node));
  if (faultPublisherInitialized) ignoreRclRet(rcl_publisher_fini(&faultPublisher, &node));
  if (statusPublisherInitialized) ignoreRclRet(rcl_publisher_fini(&statusPublisher, &node));
  if (motorRpmPublisherInitialized) ignoreRclRet(rcl_publisher_fini(&motorRpmPublisher, &node));
  if (axisPositionPublisherInitialized) ignoreRclRet(rcl_publisher_fini(&axisPositionPublisher, &node));
  if (jointStatePublisherInitialized) ignoreRclRet(rcl_publisher_fini(&jointStatePublisher, &node));
  if (actionTimerInitialized) ignoreRclRet(rcl_timer_fini(&actionTimer));
  if (statusTelemetryTimerInitialized) ignoreRclRet(rcl_timer_fini(&statusTelemetryTimer));
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

static void writeString(rosidl_runtime_c__String &text, char *buffer, size_t capacity, const char *message) {
  const int written = snprintf(buffer, capacity, "%s", message);
  text.data = buffer;
  text.capacity = capacity;
  text.size = written > 0 ? min(static_cast<size_t>(written), capacity - 1) : 0;
}

static bool waitControlCommand(uint32_t commandId, bool &accepted, uint32_t timeoutMs = ROS_SERVICE_ACCEPT_WAIT_MS) {
  const uint32_t start = millis();
  do {
    bool known = false;
    if (robotGetControlCommandStatus(commandId, known, accepted) && known) return true;
    delay(1);
  } while (millis() - start < timeoutMs);
  accepted = false;
  return false;
}

static float axisPositionFromSnapshot(const RobotStateSnapshot &snapshot, uint8_t axis) {
  switch (axis) {
    case 0:
      return snapshot.axisPositionMm[0];
    case 1:
      return snapshot.axisPositionMm[1];
    case 2:
      return snapshot.axisPositionMm[2];
    case 3: {
      float maxAbs = fabsf(snapshot.axisPositionMm[0]);
      const float yAbs = fabsf(snapshot.axisPositionMm[1]);
      const float zAbs = fabsf(snapshot.axisPositionMm[2]);
      if (yAbs > maxAbs) maxAbs = yAbs;
      if (zAbs > maxAbs) maxAbs = zAbs;
      return maxAbs;
    }
    case 4:
      return snapshot.motorPositionMm[4];
    default:
      return 0.0f;
  }
}

static float axisOriginError(const RobotStateSnapshot &snapshot, uint8_t axis) {
  switch (axis) {
    case 0:
      return fabsf(snapshot.axisPositionMm[0]);
    case 1:
      return fabsf(snapshot.axisPositionMm[1]);
    case 2:
      return fabsf(snapshot.axisPositionMm[2]);
    case 3: {
      float maxError = fabsf(snapshot.axisPositionMm[0]);
      const float yError = fabsf(snapshot.axisPositionMm[1]);
      const float zError = fabsf(snapshot.axisPositionMm[2]);
      if (yError > maxError) maxError = yError;
      if (zError > maxError) maxError = zError;
      return maxError;
    }
    case 4:
      return fabsf(snapshot.motorPositionMm[4]);
    default:
      return INFINITY;
  }
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

static void setHomeFeedbackState(const char *state) {
  const int written = snprintf(homeFeedbackStateBuffer, sizeof(homeFeedbackStateBuffer), "%s", state);
  homeFeedbackMsg.feedback.state.size =
      written > 0 ? min(static_cast<size_t>(written), sizeof(homeFeedbackStateBuffer) - 1) : 0;
}

static void setHomeResultMessage(const char *message) {
  const int written = snprintf(homeResultMessageBuffer, sizeof(homeResultMessageBuffer), "%s", message);
  homeResultResponse.result.message.size =
      written > 0 ? min(static_cast<size_t>(written), sizeof(homeResultMessageBuffer) - 1) : 0;
}

static void setOriginFeedbackState(const char *state) {
  const int written = snprintf(originFeedbackStateBuffer, sizeof(originFeedbackStateBuffer), "%s", state);
  originFeedbackMsg.feedback.state.size =
      written > 0 ? min(static_cast<size_t>(written), sizeof(originFeedbackStateBuffer) - 1) : 0;
}

static void setOriginResultMessage(const char *message) {
  const int written = snprintf(originResultMessageBuffer, sizeof(originResultMessageBuffer), "%s", message);
  originResultResponse.result.message.size =
      written > 0 ? min(static_cast<size_t>(written), sizeof(originResultMessageBuffer) - 1) : 0;
}

static float distance3(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

static float commandablePositionMm(float mm) {
  return robotCommandablePositionMm(mm);
}

static float maxMoveLinearErrorMm(const RobotStateSnapshot &state) {
  if (!state.axisPositionsValid) return INFINITY;

  float maxError = fabsf(moveTargetX - state.axisPositionMm[0]);
  const float yError = fabsf(moveTargetY - state.axisPositionMm[1]);
  const float zError = fabsf(moveTargetZ - state.axisPositionMm[2]);
  if (yError > maxError) maxError = yError;
  if (zError > maxError) maxError = zError;
  return maxError;
}

static float maxMoveErrorMm() {
  RobotStateSnapshot state;
  if (!getRobotStateSnapshot(state)) return INFINITY;
  return maxMoveLinearErrorMm(state);
}

static float moveAngularErrorDeg(const RobotStateSnapshot &state) {
  if (!moveUseA) return 0.0f;
  if (!state.encoderOk[4]) return INFINITY;
  return fabsf(moveTargetA - state.motorPositionMm[4]);
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
  if (maxMoveLinearErrorMm(state) > settleTolerance) {
    movePositionStableSinceMs = 0;
    return false;
  }

  const float angularSettleTolerance = moveAngularToleranceDeg > 0.0f ? moveAngularToleranceDeg : 1.0f;
  if (moveAngularErrorDeg(state) > angularSettleTolerance) {
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
  const float currentA = haveSnapshot && snapshot.encoderOk[4] ? snapshot.motorPositionMm[4] : 0.0f;

  moveFeedbackMsg.feedback.current_x_mm = currentX;
  moveFeedbackMsg.feedback.current_y_mm = currentY;
  moveFeedbackMsg.feedback.current_z_mm = currentZ;
  moveFeedbackMsg.feedback.current_a_deg = currentA;
  moveFeedbackMsg.feedback.error_x_mm = moveTargetX - currentX;
  moveFeedbackMsg.feedback.error_y_mm = moveTargetY - currentY;
  moveFeedbackMsg.feedback.error_z_mm = moveTargetZ - currentZ;
  moveFeedbackMsg.feedback.error_a_deg = moveUseA ? moveTargetA - currentA : 0.0f;

  const float totalDistance = distance3(moveTargetX - moveStartX, moveTargetY - moveStartY, moveTargetZ - moveStartZ);
  const float remainingDistance = distance3(
      moveFeedbackMsg.feedback.error_x_mm,
      moveFeedbackMsg.feedback.error_y_mm,
      moveFeedbackMsg.feedback.error_z_mm);
  float linearProgress = 0.0f;
  if (totalDistance <= 0.001f) {
    linearProgress = maxMoveErrorMm() <= moveToleranceMm ? 1.0f : 0.0f;
  } else {
    linearProgress = 1.0f - (remainingDistance / totalDistance);
    if (linearProgress < 0.0f) linearProgress = 0.0f;
    if (linearProgress > 1.0f) linearProgress = 1.0f;
  }

  float progress = linearProgress;
  if (moveUseA) {
    float angularProgress = 0.0f;
    const float totalAngularDistance = fabsf(moveTargetA - moveStartA);
    if (totalAngularDistance <= 0.001f) {
      angularProgress = fabsf(moveFeedbackMsg.feedback.error_a_deg) <= moveAngularToleranceDeg ? 1.0f : 0.0f;
    } else {
      angularProgress = 1.0f - (fabsf(moveFeedbackMsg.feedback.error_a_deg) / totalAngularDistance);
      if (angularProgress < 0.0f) angularProgress = 0.0f;
      if (angularProgress > 1.0f) angularProgress = 1.0f;
    }
    if (angularProgress < progress) progress = angularProgress;
  }
  moveFeedbackMsg.feedback.progress = progress;

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
  moveResultResponse.result.final_a_deg = snapshot.encoderOk[4] ? snapshot.motorPositionMm[4] : 0.0f;
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
  snprintf(movePendingResultMessage, sizeof(movePendingResultMessage), "%s", message);
  finishMoveGoal(state, success, movePendingResultMessage);
}

static bool moveFeedbackDue() {
  const uint32_t now = millis();
  if (lastMoveFeedbackMs != 0 && now - lastMoveFeedbackMs < ACTION_FEEDBACK_PERIOD_MS) {
    return false;
  }
  lastMoveFeedbackMs = now;
  return true;
}

static void publishMoveFeedback(const char *state, bool force = false) {
  if (!activeMoveGoal) return;
  if (!force && !moveFeedbackDue()) return;
  fillMoveFeedback(state);
  ignoreRclRet(rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg));
}

static void publishMoveFeedbackComplete(const char *state) {
  if (!activeMoveGoal) return;
  fillMoveFeedback(state);
  moveFeedbackMsg.feedback.progress = 1.0f;
  ignoreRclRet(rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg));
}

static void fillHomeFeedback(const RobotStateSnapshot &snapshot, const char *state) {
  homeFeedbackMsg.feedback.axis = homeAxis;
  homeFeedbackMsg.feedback.current_position_mm = axisPositionFromSnapshot(snapshot, homeAxis);
  homeFeedbackMsg.feedback.progress = 0.0f;
  if (strncmp(state, "wait_slow", 9) == 0) homeFeedbackMsg.feedback.progress = 0.75f;
  else if (strncmp(state, "config_slow", 11) == 0 || strncmp(state, "start_slow", 10) == 0) homeFeedbackMsg.feedback.progress = 0.5f;
  else if (strncmp(state, "wait_fast", 9) == 0) homeFeedbackMsg.feedback.progress = 0.25f;
  else if (strncmp(state, "complete", 8) == 0) homeFeedbackMsg.feedback.progress = 1.0f;
  setHomeFeedbackState(state);
}

static rcl_ret_t finishHomeGoal(rcl_action_goal_state_t state, bool success, const char *message) {
  if (!activeHomeGoal) return RCL_RET_ERROR;

  RobotStateSnapshot snapshot;
  getRobotStateSnapshot(snapshot);

  homeResultResponse.status = state;
  homeResultResponse.result.success = success;
  homeResultResponse.result.final_position_mm = axisPositionFromSnapshot(snapshot, homeAxis);
  setHomeResultMessage(message);

  const rcl_ret_t result = rclc_action_send_result(activeHomeGoal, state, &homeResultResponse);
  if (result == RCL_RET_OK) {
    activeHomeGoal = nullptr;
    homeResultPending = false;
  }
  return result;
}

static void queueHomeResult(rcl_action_goal_state_t state, bool success, const char *message) {
  homeResultPending = true;
  homePendingResultState = state;
  homePendingResultSuccess = success;
  snprintf(homePendingResultMessage, sizeof(homePendingResultMessage), "%s", message);
  finishHomeGoal(state, success, homePendingResultMessage);
}

static void processHomeAction(const RobotStateSnapshot &snapshot) {
  if (!activeHomeGoal) return;

  if (homeResultPending) {
    fillHomeFeedback(snapshot, homePendingResultSuccess ? "result_pending" : "terminal_result_pending");
    if (homePendingResultSuccess) homeFeedbackMsg.feedback.progress = 1.0f;
    rclc_action_publish_feedback(activeHomeGoal, &homeFeedbackMsg);
    finishHomeGoal(homePendingResultState, homePendingResultSuccess, homePendingResultMessage);
    return;
  }

  if (activeHomeGoal->goal_cancelled) {
    robotRequestStop();
    fillHomeFeedback(snapshot, "canceled");
    rclc_action_publish_feedback(activeHomeGoal, &homeFeedbackMsg);
    queueHomeResult(GOAL_STATE_CANCELED, false, "goal canceled");
    return;
  }

  if (snapshot.safetyFault) {
    fillHomeFeedback(snapshot, "fault");
    rclc_action_publish_feedback(activeHomeGoal, &homeFeedbackMsg);
    queueHomeResult(GOAL_STATE_ABORTED, false, snapshot.safetyReason);
    return;
  }

  if (!homeControlCommandAccepted) {
    bool known = false;
    bool accepted = false;
    robotGetControlCommandStatus(homeControlCommandId, known, accepted);
    if (!known) {
      fillHomeFeedback(snapshot, "queued");
      rclc_action_publish_feedback(activeHomeGoal, &homeFeedbackMsg);
      return;
    }
    if (!accepted) {
      fillHomeFeedback(snapshot, "rejected_by_control");
      rclc_action_publish_feedback(activeHomeGoal, &homeFeedbackMsg);
      queueHomeResult(GOAL_STATE_ABORTED, false, "control rejected home command");
      return;
    }
    homeControlCommandAccepted = true;
  }

  if (millis() - homeStartedMs > homeTimeoutMs) {
    robotRequestStop();
    fillHomeFeedback(snapshot, "timeout");
    rclc_action_publish_feedback(activeHomeGoal, &homeFeedbackMsg);
    queueHomeResult(GOAL_STATE_ABORTED, false, "timeout before homing complete");
    return;
  }

  fillHomeFeedback(snapshot, snapshot.homingState);
  rclc_action_publish_feedback(activeHomeGoal, &homeFeedbackMsg);

  if (!snapshot.homingActive) {
    if (strncmp(snapshot.homingState, "complete", 8) == 0) {
      homeFeedbackMsg.feedback.progress = 1.0f;
      rclc_action_publish_feedback(activeHomeGoal, &homeFeedbackMsg);
      queueHomeResult(GOAL_STATE_SUCCEEDED, true, "homing complete");
    } else if (strncmp(snapshot.homingState, "failed", 6) == 0 || strncmp(snapshot.homingState, "aborted", 7) == 0) {
      queueHomeResult(GOAL_STATE_ABORTED, false, snapshot.homingState);
    }
  }
}

static void fillOriginFeedback(const RobotStateSnapshot &snapshot, const char *state) {
  const float error = axisOriginError(snapshot, originAxis);
  originFeedbackMsg.feedback.axis = originAxis;
  originFeedbackMsg.feedback.current_position_mm = axisPositionFromSnapshot(snapshot, originAxis);
  originFeedbackMsg.feedback.error_mm = error;
  if (originStartErrorMm <= 0.001f) {
    originFeedbackMsg.feedback.progress = error <= originToleranceMm ? 1.0f : 0.0f;
  } else {
    originFeedbackMsg.feedback.progress = 1.0f - (error / originStartErrorMm);
    if (originFeedbackMsg.feedback.progress < 0.0f) originFeedbackMsg.feedback.progress = 0.0f;
    if (originFeedbackMsg.feedback.progress > 1.0f) originFeedbackMsg.feedback.progress = 1.0f;
  }
  setOriginFeedbackState(state);
}

static bool originIsStableAtTarget(const RobotStateSnapshot &snapshot) {
  const float settleTolerance = originToleranceMm > ACTION_RESULT_MIN_TOLERANCE_MM
                                    ? originToleranceMm
                                    : ACTION_RESULT_MIN_TOLERANCE_MM;
  if (axisOriginError(snapshot, originAxis) > settleTolerance) {
    originStableSinceMs = 0;
    return false;
  }

  const uint32_t now = millis();
  if (originStableSinceMs == 0) {
    originStableSinceMs = now;
    return false;
  }
  return now - originStableSinceMs >= ACTION_RESULT_POSITION_STABLE_MS;
}

static rcl_ret_t finishOriginGoal(rcl_action_goal_state_t state, bool success, const char *message) {
  if (!activeOriginGoal) return RCL_RET_ERROR;

  RobotStateSnapshot snapshot;
  getRobotStateSnapshot(snapshot);

  originResultResponse.status = state;
  originResultResponse.result.success = success;
  originResultResponse.result.final_position_mm = axisPositionFromSnapshot(snapshot, originAxis);
  setOriginResultMessage(message);

  const rcl_ret_t result = rclc_action_send_result(activeOriginGoal, state, &originResultResponse);
  if (result == RCL_RET_OK) {
    activeOriginGoal = nullptr;
    originResultPending = false;
  }
  return result;
}

static void queueOriginResult(rcl_action_goal_state_t state, bool success, const char *message) {
  originResultPending = true;
  originPendingResultState = state;
  originPendingResultSuccess = success;
  snprintf(originPendingResultMessage, sizeof(originPendingResultMessage), "%s", message);
  finishOriginGoal(state, success, originPendingResultMessage);
}

static void processOriginAction(const RobotStateSnapshot &snapshot) {
  if (!activeOriginGoal) return;

  if (originResultPending) {
    fillOriginFeedback(snapshot, originPendingResultSuccess ? "result_pending" : "terminal_result_pending");
    if (originPendingResultSuccess) originFeedbackMsg.feedback.progress = 1.0f;
    rclc_action_publish_feedback(activeOriginGoal, &originFeedbackMsg);
    finishOriginGoal(originPendingResultState, originPendingResultSuccess, originPendingResultMessage);
    return;
  }

  if (activeOriginGoal->goal_cancelled) {
    robotRequestStop();
    fillOriginFeedback(snapshot, "canceled");
    rclc_action_publish_feedback(activeOriginGoal, &originFeedbackMsg);
    queueOriginResult(GOAL_STATE_CANCELED, false, "goal canceled");
    return;
  }

  if (snapshot.safetyFault) {
    fillOriginFeedback(snapshot, "fault");
    rclc_action_publish_feedback(activeOriginGoal, &originFeedbackMsg);
    queueOriginResult(GOAL_STATE_ABORTED, false, snapshot.safetyReason);
    return;
  }

  if (!originControlCommandAccepted) {
    bool known = false;
    bool accepted = false;
    robotGetControlCommandStatus(originControlCommandId, known, accepted);
    if (!known) {
      fillOriginFeedback(snapshot, "queued");
      rclc_action_publish_feedback(activeOriginGoal, &originFeedbackMsg);
      return;
    }
    if (!accepted) {
      fillOriginFeedback(snapshot, "rejected_by_control");
      rclc_action_publish_feedback(activeOriginGoal, &originFeedbackMsg);
      queueOriginResult(GOAL_STATE_ABORTED, false, "control rejected origin command");
      return;
    }
    originControlCommandAccepted = true;
  }

  if (millis() - originStartedMs > originTimeoutMs) {
    robotRequestStop();
    fillOriginFeedback(snapshot, "timeout");
    rclc_action_publish_feedback(activeOriginGoal, &originFeedbackMsg);
    queueOriginResult(GOAL_STATE_ABORTED, false, "timeout before reaching origin");
    return;
  }

  if (originIsStableAtTarget(snapshot)) {
    fillOriginFeedback(snapshot, "arrived");
    originFeedbackMsg.feedback.progress = 1.0f;
    rclc_action_publish_feedback(activeOriginGoal, &originFeedbackMsg);
    queueOriginResult(GOAL_STATE_SUCCEEDED, true, "origin reached");
    return;
  }

  fillOriginFeedback(snapshot, snapshot.axisPositionsValid ? "moving" : "waiting_for_encoders");
  rclc_action_publish_feedback(activeOriginGoal, &originFeedbackMsg);
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

static void appendStatus(const char *fmt, ...) {
  const size_t used = strlen(statusBuffer);
  if (used >= sizeof(statusBuffer) - 1) return;
  va_list args;
  va_start(args, fmt);
  vsnprintf(statusBuffer + used, sizeof(statusBuffer) - used, fmt, args);
  va_end(args);
}

static void appendBoolArray(const char *key, const bool *values, size_t count) {
  appendStatus(",\"%s\":[", key);
  for (size_t i = 0; i < count; i++) {
    appendStatus("%s%u", i ? "," : "", values[i] ? 1 : 0);
  }
  appendStatus("]");
}

static void appendUint8Array(const char *key, const uint8_t *values, size_t count) {
  appendStatus(",\"%s\":[", key);
  for (size_t i = 0; i < count; i++) {
    appendStatus("%s%u", i ? "," : "", values[i]);
  }
  appendStatus("]");
}

static void appendInt16Array(const char *key, const int16_t *values, size_t count) {
  appendStatus(",\"%s\":[", key);
  for (size_t i = 0; i < count; i++) {
    appendStatus("%s%d", i ? "," : "", values[i]);
  }
  appendStatus("]");
}

static void appendInt32Array(const char *key, const int32_t *values, size_t count) {
  appendStatus(",\"%s\":[", key);
  for (size_t i = 0; i < count; i++) {
    appendStatus("%s%ld", i ? "," : "", static_cast<long>(values[i]));
  }
  appendStatus("]");
}

static void appendInt64Array(const char *key, const int64_t *values, size_t count) {
  appendStatus(",\"%s\":[", key);
  for (size_t i = 0; i < count; i++) {
    appendStatus("%s%lld", i ? "," : "", static_cast<long long>(values[i]));
  }
  appendStatus("]");
}

static void appendFloatArray(const char *key, const float *values, size_t count) {
  appendStatus(",\"%s\":[", key);
  for (size_t i = 0; i < count; i++) {
    appendStatus("%s%.3f", i ? "," : "", values[i]);
  }
  appendStatus("]");
}

static void appendHome3BArray(const RobotStateSnapshot &snapshot) {
  appendStatus(",\"home3B\":[");
  for (size_t i = 0; i < ROBOT_MOTOR_COUNT; i++) {
    appendStatus("%s[%u,%u]", i ? "," : "", snapshot.homeStatusSingleTurn[i], snapshot.homeStatusOrigin[i]);
  }
  appendStatus("]");
}

static void fillTelemetryMessages() {
  stampJointState();

  RobotStateSnapshot snapshot;
  getRobotStateSnapshot(snapshot);

  for (size_t i = 0; i < ROBOT_MOTOR_COUNT; i++) {
    const bool rotary = i == 4;
    jointPositions[i] = snapshot.encoderOk[i]
                            ? (rotary ? static_cast<double>(snapshot.motorPositionMm[i]) * PI / 180.0
                                      : static_cast<double>(snapshot.motorPositionMm[i]) / 1000.0)
                            : 0.0;
    jointVelocities[i] = snapshot.encoderOk[i]
                             ? (rotary ? static_cast<double>(snapshot.motorVelocityMmS[i]) * PI / 180.0
                                       : static_cast<double>(snapshot.motorVelocityMmS[i]) / 1000.0)
                             : 0.0;
    jointEfforts[i] = 0.0;
    motorRpms[i] = snapshot.rpmOk[i] ? static_cast<float>(snapshot.motorRpm[i]) : 0.0f;
  }

  axisPositionsMm[0] = snapshot.axisPositionsValid ? snapshot.axisPositionMm[0] : 0.0f;
  axisPositionsMm[1] = snapshot.axisPositionsValid ? snapshot.axisPositionMm[1] : 0.0f;
  axisPositionsMm[2] = snapshot.axisPositionsValid ? snapshot.axisPositionMm[2] : 0.0f;

  statusBuffer[0] = '\0';
  appendStatus(
      "{\"fault\":%u,\"reason\":\"%s\",\"homing\":\"%s\"",
      snapshot.safetyFault ? 1 : 0,
      snapshot.safetyReason,
      snapshot.homingState);
  appendBoolArray("online", snapshot.motorOnline, ROBOT_MOTOR_COUNT);
  appendBoolArray("enabledOk", snapshot.enabledOk, ROBOT_MOTOR_COUNT);
  appendBoolArray("enabled", snapshot.enabled, ROBOT_MOTOR_COUNT);
  appendBoolArray("stalled", snapshot.stalled, ROBOT_MOTOR_COUNT);
  appendBoolArray("raw35Ok", snapshot.rawEncoderOk, ROBOT_MOTOR_COUNT);
  appendBoolArray("angleOk", snapshot.angleErrorOk, ROBOT_MOTOR_COUNT);
  appendInt64Array("enc31", snapshot.encoder, ROBOT_MOTOR_COUNT);
  appendInt64Array("raw35", snapshot.rawDiagnosticEncoder, ROBOT_MOTOR_COUNT);
  appendFloatArray("mm", snapshot.motorPositionMm, ROBOT_MOTOR_COUNT);
  appendFloatArray("vel_mm_s", snapshot.motorVelocityMmS, ROBOT_MOTOR_COUNT);
  appendInt16Array("rpm", snapshot.motorRpm, ROBOT_MOTOR_COUNT);
  appendInt32Array("angleError", snapshot.angleError, ROBOT_MOTOR_COUNT);
  appendStatus(
      ",\"units\":[\"mm\",\"mm\",\"mm\",\"mm\",\"deg\"],\"limits\":[[%u,%.3f,%.3f],[%u,%.3f,%.3f],[%u,%.3f,%.3f]]",
      snapshot.axisLimitsConfigured[0] ? 1 : 0,
      snapshot.axisMinMm[0],
      snapshot.axisMaxMm[0],
      snapshot.axisLimitsConfigured[1] ? 1 : 0,
      snapshot.axisMinMm[1],
      snapshot.axisMaxMm[1],
      snapshot.axisLimitsConfigured[2] ? 1 : 0,
      snapshot.axisMinMm[2],
      snapshot.axisMaxMm[2]);
  appendUint8Array("moveStatus", snapshot.moveStatus, ROBOT_MOTOR_COUNT);
  appendUint8Array("home91", snapshot.homeStatus, ROBOT_MOTOR_COUNT);
  appendHome3BArray(snapshot);
  appendStatus(
      ",\"a_deg\":%.3f,\"aux_servo\":{\"enabled\":%u,\"pin\":%d,\"pulse_us\":%u,\"angle_deg\":%.1f}}",
      snapshot.motorPositionMm[4],
      snapshot.auxServoEnabled ? 1 : 0,
      static_cast<int>(AUX_SERVO_PWM_PIN),
      snapshot.auxServoPulseUs,
      snapshot.auxServoAngleDeg);
  statusMsg.data.size = strlen(statusBuffer);

  const int faultWritten = snprintf(
      faultBuffer,
      sizeof(faultBuffer),
      "%s:%s",
      snapshot.safetyFault ? "FAULT" : "OK",
      snapshot.safetyReason);
  faultMsg.data.size = faultWritten > 0 ? min(static_cast<size_t>(faultWritten), sizeof(faultBuffer) - 1) : 0;
}

static void processActions();

static void publishFastTelemetry(rcl_timer_t *, int64_t) {
  fillTelemetryMessages();
  ignoreRclRet(rcl_publish(&jointStatePublisher, &jointStateMsg, nullptr));
  ignoreRclRet(rcl_publish(&axisPositionPublisher, &axisPositionMsg, nullptr));
}

static void publishStatusTelemetry(rcl_timer_t *, int64_t) {
  fillTelemetryMessages();
  ignoreRclRet(rcl_publish(&motorRpmPublisher, &motorRpmMsg, nullptr));
  ignoreRclRet(rcl_publish(&statusPublisher, &statusMsg, nullptr));
  ignoreRclRet(rcl_publish(&faultPublisher, &faultMsg, nullptr));
}

static void processActionTimer(rcl_timer_t *, int64_t) {
  processActions();
}

static void processActions() {
  if (activeMoveGoal) {
    if (moveResultPending) {
      if (movePendingResultSuccess) {
        publishMoveFeedbackComplete("result_pending");
      } else {
        publishMoveFeedback("terminal_result_pending", true);
      }
      finishMoveGoal(movePendingResultState, movePendingResultSuccess, movePendingResultMessage);
      return;
    }

    if (activeMoveGoal->goal_cancelled) {
      robotRequestStop();
      publishMoveFeedback("canceled", true);
      queueMoveResult(GOAL_STATE_CANCELED, false, "goal canceled");
      return;
    }

    RobotStateSnapshot snapshot;
    getRobotStateSnapshot(snapshot);

    if (snapshot.safetyFault) {
      publishMoveFeedback("fault", true);
      queueMoveResult(GOAL_STATE_ABORTED, false, snapshot.safetyReason);
      return;
    }

    if (!moveControlCommandAccepted) {
      bool known = false;
      bool accepted = false;
      robotGetMoveCommandStatus(moveControlCommandId, known, accepted);
      if (!known) {
        publishMoveFeedback("queued");
        return;
      }
      if (!accepted) {
        publishMoveFeedback("rejected_by_control", true);
        queueMoveResult(GOAL_STATE_ABORTED, false, "control rejected move command");
        return;
      }
      moveControlCommandAccepted = true;
    }

    if (millis() - moveStartedMs > moveTimeoutMs) {
      robotRequestStop();
      publishMoveFeedback("timeout", true);
      queueMoveResult(GOAL_STATE_ABORTED, false, "timeout before reaching target");
      return;
    }

    if (measuredPositionIsStableAtTarget()) {
      publishMoveFeedbackComplete("arrived");
      queueMoveResult(GOAL_STATE_SUCCEEDED, true, "target reached");
      return;
    }

    publishMoveFeedback(snapshot.axisPositionsValid ? "moving" : "waiting_for_encoders");
  }

  RobotStateSnapshot actionSnapshot;
  getRobotStateSnapshot(actionSnapshot);
  processHomeAction(actionSnapshot);
  processOriginAction(actionSnapshot);
}

static void onEmergencyStop(const void *msgIn) {
  const std_msgs__msg__Bool *msg = static_cast<const std_msgs__msg__Bool *>(msgIn);
  if (msg->data) {
    robotRequestEmergencyStop();
  }
}

static void onCommand(const void *msgIn) {
  const std_msgs__msg__String *msg = static_cast<const std_msgs__msg__String *>(msgIn);
  const size_t size = msg->data.size < sizeof(commandBuffer) - 1 ? msg->data.size : sizeof(commandBuffer) - 1;
  memcpy(commandBuffer, msg->data.data, size);
  commandBuffer[size] = '\0';
  robotRequestCommandText(commandBuffer);
}

static void onFastMove(const void *msgIn) {
  const auto *msg = static_cast<const std_msgs__msg__Float32MultiArray *>(msgIn);
  if (!msg->data.data || msg->data.size < 3) return;

  uint32_t commandId = 0;
  const float speedMmS = msg->data.size > 3 ? msg->data.data[3] : 0.0f;
  const float accelMmS2 = msg->data.size > 4 ? msg->data.data[4] : 0.0f;
  robotRequestMoveXYZMm(
      msg->data.data[0],
      msg->data.data[1],
      msg->data.data[2],
      speedMmS,
      accelMmS2,
      commandId);
}

static void finishCommandService(bool queued, uint32_t commandId, bool &success, rosidl_runtime_c__String &message, char *buffer, size_t capacity) {
  if (!queued) {
    success = false;
    writeString(message, buffer, capacity, "queue full");
    return;
  }

  bool accepted = false;
  if (waitControlCommand(commandId, accepted)) {
    success = accepted;
    writeString(message, buffer, capacity, accepted ? "accepted" : "rejected by control");
    return;
  }

  success = true;
  writeString(message, buffer, capacity, "queued");
}

static void onEnableAxisService(const void *requestIn, void *responseOut) {
  const auto *request = static_cast<const palletizer_msgs__srv__EnableAxis_Request *>(requestIn);
  auto *response = static_cast<palletizer_msgs__srv__EnableAxis_Response *>(responseOut);
  uint32_t commandId = 0;
  const bool queued = robotRequestSetAxisEnable(request->axis, request->enable, commandId);
  finishCommandService(queued, commandId, response->success, response->message, serviceMessageBuffers[0], sizeof(serviceMessageBuffers[0]));
}

static void onSetZeroService(const void *requestIn, void *responseOut) {
  const auto *request = static_cast<const palletizer_msgs__srv__SetZero_Request *>(requestIn);
  auto *response = static_cast<palletizer_msgs__srv__SetZero_Response *>(responseOut);
  uint32_t commandId = 0;
  const bool queued = robotRequestSetZero(request->axis, request->min_mm, request->max_mm, commandId);
  finishCommandService(queued, commandId, response->success, response->message, serviceMessageBuffers[1], sizeof(serviceMessageBuffers[1]));
}

static void onSetAxisLimitsService(const void *requestIn, void *responseOut) {
  const auto *request = static_cast<const palletizer_msgs__srv__SetAxisLimits_Request *>(requestIn);
  auto *response = static_cast<palletizer_msgs__srv__SetAxisLimits_Response *>(responseOut);
  uint32_t commandId = 0;
  const bool queued = robotRequestSetAxisLimits(request->axis, request->min_mm, request->max_mm, commandId);
  finishCommandService(queued, commandId, response->success, response->message, serviceMessageBuffers[2], sizeof(serviceMessageBuffers[2]));
}

static void onClearFaultService(const void *, void *responseOut) {
  auto *response = static_cast<palletizer_msgs__srv__ClearFault_Response *>(responseOut);
  uint32_t commandId = 0;
  const bool queued = robotRequestClearFault(commandId);
  finishCommandService(queued, commandId, response->success, response->message, serviceMessageBuffers[3], sizeof(serviceMessageBuffers[3]));
}

static void onReleaseStallService(const void *requestIn, void *responseOut) {
  const auto *request = static_cast<const palletizer_msgs__srv__ReleaseStall_Request *>(requestIn);
  auto *response = static_cast<palletizer_msgs__srv__ReleaseStall_Response *>(responseOut);
  uint32_t commandId = 0;
  const bool queued = robotRequestReleaseStall(request->axis, commandId);
  finishCommandService(queued, commandId, response->success, response->message, serviceMessageBuffers[4], sizeof(serviceMessageBuffers[4]));
}

static void onGetDriverStatusService(const void *, void *responseOut) {
  auto *response = static_cast<palletizer_msgs__srv__GetDriverStatus_Response *>(responseOut);
  RobotStateSnapshot snapshot{};
  const bool haveSnapshot = getRobotStateSnapshot(snapshot);

  response->success = haveSnapshot;
  response->safety_fault = snapshot.safetyFault;
  writeString(response->message, driverStatusMessageBuffer, sizeof(driverStatusMessageBuffer), haveSnapshot ? "ok" : "no snapshot");
  writeString(response->safety_reason, driverStatusSafetyReasonBuffer, sizeof(driverStatusSafetyReasonBuffer), snapshot.safetyReason);
  writeString(response->homing_state, driverStatusHomingStateBuffer, sizeof(driverStatusHomingStateBuffer), snapshot.homingState);

  for (size_t i = 0; i < ROBOT_MOTOR_COUNT; i++) {
    response->online[i] = snapshot.motorOnline[i];
    response->enabled_ok[i] = snapshot.enabledOk[i];
    response->enabled[i] = snapshot.enabled[i];
    response->stalled[i] = snapshot.stalled[i];
    response->raw35_ok[i] = snapshot.rawEncoderOk[i];
    response->angle_error_ok[i] = snapshot.angleErrorOk[i];
    response->enc31[i] = snapshot.encoder[i];
    response->raw35[i] = snapshot.rawDiagnosticEncoder[i];
    response->position_mm[i] = snapshot.motorPositionMm[i];
    response->velocity_mm_s[i] = snapshot.motorVelocityMmS[i];
    response->angle_error[i] = snapshot.angleError[i];
    response->home91[i] = snapshot.homeStatus[i];
    response->home3b_single[i] = snapshot.homeStatusSingleTurn[i];
    response->home3b_origin[i] = snapshot.homeStatusOrigin[i];
  }

  for (size_t i = 0; i < 3; i++) {
    response->limits_configured[i] = snapshot.axisLimitsConfigured[i];
    response->limit_min_mm[i] = snapshot.axisMinMm[i];
    response->limit_max_mm[i] = snapshot.axisMaxMm[i];
  }
}

static void onSetGripperService(const void *requestIn, void *responseOut) {
  const auto *request = static_cast<const palletizer_msgs__srv__SetGripper_Request *>(requestIn);
  auto *response = static_cast<palletizer_msgs__srv__SetGripper_Response *>(responseOut);

  const float targetAngleDeg = request->closed ? AUX_SERVO_MIN_DEG : AUX_SERVO_MAX_DEG;
  uint32_t commandId = 0;
  const bool queued = robotRequestAuxServoAngle(targetAngleDeg, commandId);
  finishCommandService(queued, commandId, response->success, response->message, serviceMessageBuffers[6], sizeof(serviceMessageBuffers[6]));

  response->closed = request->closed;
  response->angle_deg = targetAngleDeg;
  response->pulse_us = request->closed ? AUX_SERVO_MIN_US : AUX_SERVO_MAX_US;
}

static bool anyActionActive() {
  return activeMoveGoal || activeHomeGoal || activeOriginGoal;
}

static rcl_ret_t onMoveGoal(rclc_action_goal_handle_t *goalHandle, void *) {
  if (anyActionActive()) {
    return RCL_RET_ACTION_GOAL_REJECTED;
  }

  palletizer_msgs__action__MoveXYZ_SendGoal_Request *request =
      reinterpret_cast<palletizer_msgs__action__MoveXYZ_SendGoal_Request *>(goalHandle->ros_goal_request);

  const float requestedX = request->goal.x_mm;
  const float requestedY = request->goal.y_mm;
  const float requestedZ = request->goal.z_mm;
  const bool requestedUseA = request->goal.use_a;
  const float requestedA = request->goal.a_deg;

  moveTargetX = commandablePositionMm(requestedX);
  moveTargetY = commandablePositionMm(requestedY);
  moveTargetZ = commandablePositionMm(requestedZ);
  moveUseA = requestedUseA;
  moveTargetA = requestedA;
  moveToleranceMm = request->goal.tolerance_mm > 0.0f ? request->goal.tolerance_mm : 1.0f;
  moveAngularToleranceDeg = request->goal.angular_tolerance_deg > 0.0f ? request->goal.angular_tolerance_deg : 1.0f;
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
  moveStartA = snapshot.encoderOk[4] ? snapshot.motorPositionMm[4] : 0.0f;

  const bool commandQueued = robotRequestMoveXYZAMmDeg(
      requestedX,
      requestedY,
      requestedZ,
      requestedUseA,
      requestedA,
      request->goal.speed_mm_s,
      request->goal.accel_mm_s2,
      request->goal.angular_speed_deg_s,
      request->goal.angular_accel_deg_s2,
      moveControlCommandId);

  if (!commandQueued) {
    return RCL_RET_ACTION_GOAL_REJECTED;
  }

  activeMoveGoal = goalHandle;
  lastMoveFeedbackMs = millis();
  fillMoveFeedback("accepted");
  ignoreRclRet(rclc_action_publish_feedback(activeMoveGoal, &moveFeedbackMsg));
  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

static bool onMoveCancel(rclc_action_goal_handle_t *goalHandle, void *) {
  if (goalHandle != activeMoveGoal) return false;
  robotRequestStop();
  return true;
}

static rcl_ret_t onHomeGoal(rclc_action_goal_handle_t *goalHandle, void *) {
  if (anyActionActive()) return RCL_RET_ACTION_GOAL_REJECTED;

  auto *request =
      reinterpret_cast<palletizer_msgs__action__HomeAxis_SendGoal_Request *>(goalHandle->ros_goal_request);

  homeAxis = request->goal.axis;
  homeStartedMs = millis();
  homeTimeoutMs = request->goal.timeout_ms > 0 ? request->goal.timeout_ms : HOME_PHASE_TIMEOUT_MS * 2;
  homeControlCommandId = 0;
  homeControlCommandAccepted = false;

  const uint16_t fastRpm = request->goal.fast_rpm > 0.0f
                               ? static_cast<uint16_t>(min(static_cast<int32_t>(roundf(request->goal.fast_rpm)), static_cast<int32_t>(MAX_RPM)))
                               : 0;
  const uint16_t slowRpm = request->goal.slow_rpm > 0.0f
                               ? static_cast<uint16_t>(min(static_cast<int32_t>(roundf(request->goal.slow_rpm)), static_cast<int32_t>(MAX_RPM)))
                               : 0;
  const bool commandQueued = robotRequestHomeAxis(
      homeAxis,
      request->goal.set_limits,
      request->goal.min_mm,
      request->goal.max_mm,
      fastRpm,
      slowRpm,
      homeControlCommandId);

  if (!commandQueued) return RCL_RET_ACTION_GOAL_REJECTED;

  RobotStateSnapshot snapshot;
  getRobotStateSnapshot(snapshot);
  activeHomeGoal = goalHandle;
  fillHomeFeedback(snapshot, "accepted");
  rclc_action_publish_feedback(activeHomeGoal, &homeFeedbackMsg);
  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

static bool onHomeCancel(rclc_action_goal_handle_t *goalHandle, void *) {
  if (goalHandle != activeHomeGoal) return false;
  robotRequestStop();
  return true;
}

static rcl_ret_t onOriginGoal(rclc_action_goal_handle_t *goalHandle, void *) {
  if (anyActionActive()) return RCL_RET_ACTION_GOAL_REJECTED;

  auto *request =
      reinterpret_cast<palletizer_msgs__action__GoOrigin_SendGoal_Request *>(goalHandle->ros_goal_request);

  RobotStateSnapshot snapshot;
  getRobotStateSnapshot(snapshot);
  originAxis = request->goal.axis;
  originToleranceMm = request->goal.tolerance_mm > 0.0f ? request->goal.tolerance_mm : 1.0f;
  originStartedMs = millis();
  originTimeoutMs = request->goal.timeout_ms > 0 ? request->goal.timeout_ms : 30000;
  originStableSinceMs = 0;
  originStartErrorMm = axisOriginError(snapshot, originAxis);
  originControlCommandId = 0;
  originControlCommandAccepted = false;

  const bool commandQueued = robotRequestGoOrigin(originAxis, originControlCommandId);
  if (!commandQueued) return RCL_RET_ACTION_GOAL_REJECTED;

  activeOriginGoal = goalHandle;
  fillOriginFeedback(snapshot, "accepted");
  rclc_action_publish_feedback(activeOriginGoal, &originFeedbackMsg);
  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

static bool onOriginCancel(rclc_action_goal_handle_t *goalHandle, void *) {
  if (goalHandle != activeOriginGoal) return false;
  robotRequestStop();
  return true;
}

static void initMessages() {
  setString(jointStateMsg.header.frame_id, frameIdBuffer, sizeof(frameIdBuffer), strlen(frameIdBuffer));

  jointStateMsg.name.data = jointNames;
  jointStateMsg.name.size = ROBOT_MOTOR_COUNT;
  jointStateMsg.name.capacity = ROBOT_MOTOR_COUNT;
  for (size_t i = 0; i < ROBOT_MOTOR_COUNT; i++) {
    setString(jointNames[i], jointNameBuffers[i], sizeof(jointNameBuffers[i]), strlen(jointNameBuffers[i]));
  }

  jointStateMsg.position.data = jointPositions;
  jointStateMsg.position.size = ROBOT_MOTOR_COUNT;
  jointStateMsg.position.capacity = ROBOT_MOTOR_COUNT;
  jointStateMsg.velocity.data = jointVelocities;
  jointStateMsg.velocity.size = ROBOT_MOTOR_COUNT;
  jointStateMsg.velocity.capacity = ROBOT_MOTOR_COUNT;
  jointStateMsg.effort.data = jointEfforts;
  jointStateMsg.effort.size = ROBOT_MOTOR_COUNT;
  jointStateMsg.effort.capacity = ROBOT_MOTOR_COUNT;

  axisPositionMsg.layout.dim.size = 0;
  axisPositionMsg.layout.dim.capacity = 0;
  axisPositionMsg.layout.dim.data = nullptr;
  axisPositionMsg.layout.data_offset = 0;
  axisPositionMsg.data.data = axisPositionsMm;
  axisPositionMsg.data.size = ROBOT_LINEAR_AXIS_COUNT;
  axisPositionMsg.data.capacity = ROBOT_LINEAR_AXIS_COUNT;

  motorRpmMsg.layout.dim.size = 0;
  motorRpmMsg.layout.dim.capacity = 0;
  motorRpmMsg.layout.dim.data = nullptr;
  motorRpmMsg.layout.data_offset = 0;
  motorRpmMsg.data.data = motorRpms;
  motorRpmMsg.data.size = ROBOT_MOTOR_COUNT;
  motorRpmMsg.data.capacity = ROBOT_MOTOR_COUNT;

  fastMoveMsg.layout.dim.size = 0;
  fastMoveMsg.layout.dim.capacity = 0;
  fastMoveMsg.layout.dim.data = nullptr;
  fastMoveMsg.layout.data_offset = 0;
  fastMoveMsg.data.data = fastMoveData;
  fastMoveMsg.data.size = 0;
  fastMoveMsg.data.capacity = 5;

  setString(statusMsg.data, statusBuffer, sizeof(statusBuffer));
  setString(faultMsg.data, faultBuffer, sizeof(faultBuffer));
  setString(commandMsg.data, commandBuffer, sizeof(commandBuffer));
  setString(enableAxisResponse.message, serviceMessageBuffers[0], sizeof(serviceMessageBuffers[0]));
  setString(setZeroResponse.message, serviceMessageBuffers[1], sizeof(serviceMessageBuffers[1]));
  setString(setAxisLimitsResponse.message, serviceMessageBuffers[2], sizeof(serviceMessageBuffers[2]));
  setString(setGripperResponse.message, serviceMessageBuffers[6], sizeof(serviceMessageBuffers[6]));
  setString(clearFaultResponse.message, serviceMessageBuffers[3], sizeof(serviceMessageBuffers[3]));
  setString(releaseStallResponse.message, serviceMessageBuffers[4], sizeof(serviceMessageBuffers[4]));
  setString(getDriverStatusResponse.message, driverStatusMessageBuffer, sizeof(driverStatusMessageBuffer));
  setString(getDriverStatusResponse.safety_reason, driverStatusSafetyReasonBuffer, sizeof(driverStatusSafetyReasonBuffer));
  setString(getDriverStatusResponse.homing_state, driverStatusHomingStateBuffer, sizeof(driverStatusHomingStateBuffer));
  setString(moveFeedbackMsg.feedback.state, moveFeedbackStateBuffer, sizeof(moveFeedbackStateBuffer));
  setString(moveResultResponse.result.message, moveResultMessageBuffer, sizeof(moveResultMessageBuffer));
  setString(homeFeedbackMsg.feedback.state, homeFeedbackStateBuffer, sizeof(homeFeedbackStateBuffer));
  setString(homeResultResponse.result.message, homeResultMessageBuffer, sizeof(homeResultMessageBuffer));
  setString(originFeedbackMsg.feedback.state, originFeedbackStateBuffer, sizeof(originFeedbackStateBuffer));
  setString(originResultResponse.result.message, originResultMessageBuffer, sizeof(originResultMessageBuffer));
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
  rmw_context_t *rmwContext = rcl_context_get_rmw_context(&support.context);
  if (rmwContext) {
    rmw_uros_set_context_entity_creation_session_timeout(rmwContext, MICRO_ROS_ENTITY_CREATION_TIMEOUT_MS);
    rmw_uros_set_context_entity_destroy_session_timeout(rmwContext, MICRO_ROS_ENTITY_DESTROY_TIMEOUT_MS);
  }
  if (!check(rclc_node_init_default(&node, "palletizer_controller", "", &support))) {
    disconnectRosBridge();
    return false;
  }
  nodeInitialized = true;

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
  if (ROS_ENABLE_HOME_ORIGIN_ACTIONS) {
    if (!check(rclc_action_server_init_default(
            &homeActionServer,
            &node,
            &support,
            ROSIDL_GET_ACTION_TYPE_SUPPORT(palletizer_msgs, HomeAxis),
            "/palletizer/home_axis"))) {
      disconnectRosBridge();
      return false;
    }
    homeActionServerInitialized = true;
    if (!check(rclc_action_server_init_default(
            &originActionServer,
            &node,
            &support,
            ROSIDL_GET_ACTION_TYPE_SUPPORT(palletizer_msgs, GoOrigin),
            "/palletizer/go_origin"))) {
      disconnectRosBridge();
      return false;
    }
    originActionServerInitialized = true;
  }

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
  if (ROS_ENABLE_COMMAND_SUBSCRIBER) {
    if (!check(rclc_subscription_init_best_effort(
            &commandSubscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/palletizer/command"))) {
      disconnectRosBridge();
      return false;
    }
    commandSubscriberInitialized = true;
  }
  if (ROS_ENABLE_FAST_MOVE_SUBSCRIBER) {
    if (!check(rclc_subscription_init_best_effort(
            &fastMoveSubscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/palletizer/fast_move_xyz"))) {
      disconnectRosBridge();
      return false;
    }
    fastMoveSubscriberInitialized = true;
  }

  if (ROS_ENABLE_GRIPPER_SERVICE) {
    if (!check(rclc_service_init_default(
            &setGripperService,
            &node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(palletizer_msgs, srv, SetGripper),
            "/palletizer/set_gripper"))) {
      disconnectRosBridge();
      return false;
    }
    setGripperServiceInitialized = true;
  }

  if (ROS_ENABLE_SERVICE_SERVERS) {
    if (check(rclc_service_init_default(
            &enableAxisService, &node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(palletizer_msgs, srv, EnableAxis),
            "/palletizer/enable_axis"))) {
      enableAxisServiceInitialized = true;
    } else {
      rcl_reset_error();
    }
    if (check(rclc_service_init_default(
            &setAxisLimitsService, &node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(palletizer_msgs, srv, SetAxisLimits),
            "/palletizer/set_axis_limits"))) {
      setAxisLimitsServiceInitialized = true;
    } else {
      rcl_reset_error();
    }
    if (ROS_ENABLE_EXTENDED_SERVICE_SERVERS) {
      if (check(rclc_service_init_default(
              &setZeroService, &node,
              ROSIDL_GET_SRV_TYPE_SUPPORT(palletizer_msgs, srv, SetZero),
              "/palletizer/set_zero"))) {
        setZeroServiceInitialized = true;
      } else {
        rcl_reset_error();
      }
      if (check(rclc_service_init_default(
              &clearFaultService, &node,
              ROSIDL_GET_SRV_TYPE_SUPPORT(palletizer_msgs, srv, ClearFault),
              "/palletizer/clear_fault"))) {
        clearFaultServiceInitialized = true;
      } else {
        rcl_reset_error();
      }
      if (check(rclc_service_init_default(
              &releaseStallService, &node,
              ROSIDL_GET_SRV_TYPE_SUPPORT(palletizer_msgs, srv, ReleaseStall),
              "/palletizer/release_stall"))) {
        releaseStallServiceInitialized = true;
      } else {
        rcl_reset_error();
      }
      if (check(rclc_service_init_default(
              &getDriverStatusService, &node,
              ROSIDL_GET_SRV_TYPE_SUPPORT(palletizer_msgs, srv, GetDriverStatus),
              "/palletizer/get_driver_status"))) {
        getDriverStatusServiceInitialized = true;
      } else {
        rcl_reset_error();
      }
    }
  }

  initMessages();

  if (!check(rclc_timer_init_default(
          &telemetryTimer,
          &support,
          RCL_MS_TO_NS(ROS_FAST_TELEMETRY_PERIOD_MS),
          publishFastTelemetry))) {
    disconnectRosBridge();
    return false;
  }
  telemetryTimerInitialized = true;
  if (!check(rclc_timer_init_default(
          &statusTelemetryTimer,
          &support,
          RCL_MS_TO_NS(ROS_STATUS_TELEMETRY_PERIOD_MS),
          publishStatusTelemetry))) {
    disconnectRosBridge();
    return false;
  }
  statusTelemetryTimerInitialized = true;
  if (!check(rclc_timer_init_default(
          &actionTimer,
          &support,
          RCL_MS_TO_NS(ACTION_PROCESS_PERIOD_MS),
          processActionTimer))) {
    disconnectRosBridge();
    return false;
  }
  actionTimerInitialized = true;

  if (!check(rclc_executor_init(&executor, &support.context, ROS_EXECUTOR_HANDLES, &allocator))) {
    disconnectRosBridge();
    return false;
  }
  executorInitialized = true;
  if (!check(rclc_executor_add_subscription(
          &executor,
          &emergencyStopSubscriber,
          &emergencyStopMsg,
          onEmergencyStop,
          ON_NEW_DATA))) {
    disconnectRosBridge();
    return false;
  }
  if (ROS_ENABLE_FAST_MOVE_SUBSCRIBER && !check(rclc_executor_add_subscription(
          &executor,
          &fastMoveSubscriber,
          &fastMoveMsg,
          onFastMove,
          ON_NEW_DATA))) {
    disconnectRosBridge();
    return false;
  }
  if (ROS_ENABLE_COMMAND_SUBSCRIBER && !check(rclc_executor_add_subscription(
          &executor,
          &commandSubscriber,
          &commandMsg,
          onCommand,
          ON_NEW_DATA))) {
    disconnectRosBridge();
    return false;
  }
  if (ROS_ENABLE_GRIPPER_SERVICE && setGripperServiceInitialized &&
      !check(rclc_executor_add_service(
          &executor,
          &setGripperService,
          &setGripperRequest,
          &setGripperResponse,
          onSetGripperService))) {
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
  if (ROS_ENABLE_HOME_ORIGIN_ACTIONS) {
    if (!check(rclc_executor_add_action_server(
            &executor,
            &homeActionServer,
            1,
            &homeGoalRequest,
            sizeof(homeGoalRequest),
            onHomeGoal,
            onHomeCancel,
            nullptr))) {
      disconnectRosBridge();
      return false;
    }
    if (!check(rclc_executor_add_action_server(
            &executor,
            &originActionServer,
            1,
            &originGoalRequest,
            sizeof(originGoalRequest),
            onOriginGoal,
            onOriginCancel,
            nullptr))) {
      disconnectRosBridge();
      return false;
    }
  }

  if (ROS_ENABLE_SERVICE_SERVERS && enableAxisServiceInitialized &&
      !check(rclc_executor_add_service(
          &executor,
          &enableAxisService,
          &enableAxisRequest,
          &enableAxisResponse,
          onEnableAxisService))) {
    disconnectRosBridge();
    return false;
  }
  if (ROS_ENABLE_EXTENDED_SERVICE_SERVERS && setZeroServiceInitialized &&
      !check(rclc_executor_add_service(
          &executor,
          &setZeroService,
          &setZeroRequest,
          &setZeroResponse,
          onSetZeroService))) {
    disconnectRosBridge();
    return false;
  }
  if (ROS_ENABLE_SERVICE_SERVERS && setAxisLimitsServiceInitialized &&
      !check(rclc_executor_add_service(
          &executor,
          &setAxisLimitsService,
          &setAxisLimitsRequest,
          &setAxisLimitsResponse,
          onSetAxisLimitsService))) {
    disconnectRosBridge();
    return false;
  }
  if (ROS_ENABLE_EXTENDED_SERVICE_SERVERS && clearFaultServiceInitialized &&
      !check(rclc_executor_add_service(
          &executor,
          &clearFaultService,
          &clearFaultRequest,
          &clearFaultResponse,
          onClearFaultService))) {
    disconnectRosBridge();
    return false;
  }
  if (ROS_ENABLE_EXTENDED_SERVICE_SERVERS && releaseStallServiceInitialized &&
      !check(rclc_executor_add_service(
          &executor,
          &releaseStallService,
          &releaseStallRequest,
          &releaseStallResponse,
          onReleaseStallService))) {
    disconnectRosBridge();
    return false;
  }
  if (ROS_ENABLE_EXTENDED_SERVICE_SERVERS && getDriverStatusServiceInitialized &&
      !check(rclc_executor_add_service(
          &executor,
          &getDriverStatusService,
          &getDriverStatusRequest,
          &getDriverStatusResponse,
          onGetDriverStatusService))) {
    disconnectRosBridge();
    return false;
  }

  if (!check(rclc_executor_add_timer(&executor, &actionTimer))) {
    disconnectRosBridge();
    return false;
  }
  if (!check(rclc_executor_add_timer(&executor, &telemetryTimer))) {
    disconnectRosBridge();
    return false;
  }
  if (!check(rclc_executor_add_timer(&executor, &statusTelemetryTimer))) {
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
