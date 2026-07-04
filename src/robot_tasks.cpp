#include "robot_tasks.h"

#include <string.h>

#include "config.h"
#include "machine.h"
#include "ros_bridge.h"
#include "serial_commands.h"

enum class RobotCommandType : uint8_t {
  MOVE_XYZ,
  STOP,
  TEXT,
  HOME_AXIS,
  GO_ORIGIN,
  SET_ZERO,
  SET_AXIS_LIMITS,
  SET_AXIS_ENABLE,
  CLEAR_FAULT,
  RELEASE_STALL,
  AUX_SERVO_ANGLE,
  AUX_SERVO_PULSE_US,
  AUX_SERVO_DISABLE,
};

struct RobotCommand {
  RobotCommandType type = RobotCommandType::STOP;
  uint32_t id = 0;
  float xMm = 0.0f;
  float yMm = 0.0f;
  float zMm = 0.0f;
  float speedMmS = 0.0f;
  float accelMmS2 = 0.0f;
  float aDeg = 0.0f;
  float angularSpeedDegS = 0.0f;
  float angularAccelDegS2 = 0.0f;
  bool useA = false;
  uint8_t axis = 0;
  bool flag = false;
  float minMm = 0.0f;
  float maxMm = 0.0f;
  uint16_t fastRpm = 0;
  uint16_t slowRpm = 0;
  uint16_t pulseUs = 0;
  char text[128] = "";
};

static QueueHandle_t robotCommandQueue = nullptr;
static TaskHandle_t controlTaskHandle = nullptr;
static TaskHandle_t rosTaskHandle = nullptr;
static portMUX_TYPE snapshotMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE commandStatusMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE emergencyMux = portMUX_INITIALIZER_UNLOCKED;
static RobotStateSnapshot latestSnapshot;
static volatile bool emergencyStopRequested = false;
static uint32_t nextCommandId = 1;
static uint32_t lastMoveCommandId = 0;
static bool lastMoveCommandKnown = false;
static bool lastMoveCommandAccepted = false;
static uint32_t lastControlCommandId = 0;
static bool lastControlCommandKnown = false;
static bool lastControlCommandAccepted = false;

static Axis axisFromCommand(uint8_t axis) {
  switch (axis) {
    case 0:
      return Axis::X;
    case 1:
      return Axis::Y;
    case 2:
      return Axis::Z;
    case 3:
      return Axis::ALL;
    case 4:
      return Axis::A;
    case 5:
      return Axis::X1;
    case 6:
      return Axis::X2;
    default:
      return Axis::UNKNOWN;
  }
}

static uint32_t allocateCommandId() {
  portENTER_CRITICAL(&commandStatusMux);
  const uint32_t commandId = nextCommandId++;
  if (nextCommandId == 0) nextCommandId = 1;
  portEXIT_CRITICAL(&commandStatusMux);
  return commandId;
}

static void setMoveCommandPending(uint32_t commandId) {
  portENTER_CRITICAL(&commandStatusMux);
  lastMoveCommandId = commandId;
  lastMoveCommandKnown = false;
  lastMoveCommandAccepted = false;
  portEXIT_CRITICAL(&commandStatusMux);
}

static void setMoveCommandStatus(uint32_t commandId, bool accepted) {
  portENTER_CRITICAL(&commandStatusMux);
  lastMoveCommandId = commandId;
  lastMoveCommandKnown = true;
  lastMoveCommandAccepted = accepted;
  portEXIT_CRITICAL(&commandStatusMux);
}

static void setControlCommandPending(uint32_t commandId) {
  portENTER_CRITICAL(&commandStatusMux);
  lastControlCommandId = commandId;
  lastControlCommandKnown = false;
  lastControlCommandAccepted = false;
  portEXIT_CRITICAL(&commandStatusMux);
}

static void setControlCommandStatus(uint32_t commandId, bool accepted) {
  portENTER_CRITICAL(&commandStatusMux);
  lastControlCommandId = commandId;
  lastControlCommandKnown = true;
  lastControlCommandAccepted = accepted;
  portEXIT_CRITICAL(&commandStatusMux);
}

static bool takeEmergencyStopRequest() {
  bool requested = false;
  portENTER_CRITICAL(&emergencyMux);
  requested = emergencyStopRequested;
  emergencyStopRequested = false;
  portEXIT_CRITICAL(&emergencyMux);
  return requested;
}

static void publishSnapshot() {
  RobotStateSnapshot next;
  float xMm = 0.0f;
  float yMm = 0.0f;
  float zMm = 0.0f;
  const uint32_t now = millis();

  getAxisPositionsMm(xMm, yMm, zMm);
  next.axisPositionMm[0] = xMm;
  next.axisPositionMm[1] = yMm;
  next.axisPositionMm[2] = zMm;
  next.axisPositionsValid = axisPositionsAreValid();
  next.axisLimitsConfigured[0] = getAxisLimits(Axis::X, next.axisMinMm[0], next.axisMaxMm[0]);
  next.axisLimitsConfigured[1] = getAxisLimits(Axis::Y, next.axisMinMm[1], next.axisMaxMm[1]);
  next.axisLimitsConfigured[2] = getAxisLimits(Axis::Z, next.axisMinMm[2], next.axisMaxMm[2]);
  next.homingActive = homingIsActive();
  snprintf(next.homingState, sizeof(next.homingState), "%s", homingStateText());
  next.safetyFault = safetyFaultIsActive();
  snprintf(next.safetyReason, sizeof(next.safetyReason), "%s", safetyFaultText());
  getAuxServoState(next.auxServoEnabled, next.auxServoPulseUs, next.auxServoAngleDeg);
  next.stampMs = now;

  for (size_t i = 0; i < ROBOT_MOTOR_COUNT; i++) {
    const MotorNode &motor = motors[i];
    next.encoder[i] = motor.encoderOk ? motor.encoder : 0;
    next.rawDiagnosticEncoder[i] = motor.rawEncoderOk ? motor.diagnosticRawEncoder : 0;
    next.motorPositionMm[i] = motor.encoderOk ? motorPositionUnits(motor) : 0.0f;
    next.motorVelocityMmS[i] = motor.encoderOk ? motorVelocityUnitsPerS(motor) : 0.0f;
    next.motorRpm[i] = motor.rpmOk ? motor.rpm : 0;
    next.angleError[i] = motor.angleErrorOk ? motor.angleError : 0;
    next.lastAcc[i] = motor.lastAcc;
    next.moveStatus[i] = motor.moveStatus;
    next.homeStatus[i] = motor.homeStatus;
    next.homeStatusSingleTurn[i] = motor.homeStatusSingleTurn;
    next.homeStatusOrigin[i] = motor.homeStatusOrigin;
    next.enabled[i] = motor.enabled;
    next.stalled[i] = motor.stalled;
    next.motorOnline[i] = motorIsOnline(motor, now);
    next.encoderOk[i] = motor.encoderOk;
    next.rawEncoderOk[i] = motor.rawEncoderOk;
    next.rpmOk[i] = motor.rpmOk;
    next.angleErrorOk[i] = motor.angleErrorOk;
    next.enabledOk[i] = motor.enabledOk;
    next.stallOk[i] = motor.stallOk;
    next.homeStatusOk[i] = motor.homeStatusOk;
  }

  portENTER_CRITICAL(&snapshotMux);
  latestSnapshot = next;
  portEXIT_CRITICAL(&snapshotMux);
}

static void processRobotCommand(const RobotCommand &command) {
  switch (command.type) {
    case RobotCommandType::MOVE_XYZ: {
      const bool accepted = commandMoveXYZAMmDeg(
          command.xMm,
          command.yMm,
          command.zMm,
          command.useA,
          command.aDeg,
          command.speedMmS,
          command.accelMmS2,
          command.angularSpeedDegS,
          command.angularAccelDegS2);
      setMoveCommandStatus(command.id, accepted);
      break;
    }
    case RobotCommandType::STOP:
      stopAllMotors();
      break;
    case RobotCommandType::TEXT:
      handleMachineCommand(String(command.text));
      break;
    case RobotCommandType::HOME_AXIS:
      setControlCommandStatus(
          command.id,
          startHomeAxis(
              axisFromCommand(command.axis),
              command.flag,
              command.minMm,
              command.maxMm,
              command.fastRpm,
              command.slowRpm));
      break;
    case RobotCommandType::GO_ORIGIN:
      setControlCommandStatus(command.id, commandGoOriginAxis(axisFromCommand(command.axis)));
      break;
    case RobotCommandType::SET_ZERO:
      setControlCommandStatus(
          command.id,
          commandSetZeroAxis(axisFromCommand(command.axis), true, command.minMm, command.maxMm));
      break;
    case RobotCommandType::SET_AXIS_LIMITS:
      setControlCommandStatus(
          command.id,
          commandSetAxisLimits(axisFromCommand(command.axis), command.minMm, command.maxMm));
      break;
    case RobotCommandType::SET_AXIS_ENABLE:
      setControlCommandStatus(command.id, commandSetAxisEnable(axisFromCommand(command.axis), command.flag));
      break;
    case RobotCommandType::CLEAR_FAULT:
      setControlCommandStatus(command.id, commandClearSafetyFault());
      break;
    case RobotCommandType::RELEASE_STALL:
      setControlCommandStatus(command.id, commandReleaseStallAxis(axisFromCommand(command.axis)));
      break;
    case RobotCommandType::AUX_SERVO_ANGLE:
      setControlCommandStatus(command.id, commandSetAuxServoAngle(command.xMm));
      break;
    case RobotCommandType::AUX_SERVO_PULSE_US:
      setControlCommandStatus(command.id, commandSetAuxServoPulseUs(command.pulseUs));
      break;
    case RobotCommandType::AUX_SERVO_DISABLE:
      setControlCommandStatus(command.id, commandDisableAuxServo());
      break;
  }
}

static void controlTask(void *) {
  for (;;) {
    if (takeEmergencyStopRequest()) {
      stopAllMotors();
    }

    RobotCommand command;
    while (robotCommandQueue && xQueueReceive(robotCommandQueue, &command, 0) == pdTRUE) {
      processRobotCommand(command);
    }

    pollEncoders();
    drainCanReplies();
    serviceMachine();

    publishSnapshot();
    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
  }
}

static void rosTask(void *) {
#if ENABLE_MICRO_ROS
  for (;;) {
    manageRosBridge();
    spinRosBridge();
    vTaskDelay(pdMS_TO_TICKS(ROS_TASK_PERIOD_MS));
  }
#else
  vTaskDelete(nullptr);
#endif
}

bool beginRobotTasks() {
  if (!robotCommandQueue) {
    robotCommandQueue = xQueueCreate(ROBOT_COMMAND_QUEUE_LENGTH, sizeof(RobotCommand));
    if (!robotCommandQueue) return false;
  }

  publishSnapshot();

  if (!controlTaskHandle) {
    const BaseType_t created = xTaskCreatePinnedToCore(
        controlTask,
        "ControlTask",
        CONTROL_TASK_STACK_BYTES,
        nullptr,
        CONTROL_TASK_PRIORITY,
        &controlTaskHandle,
        CONTROL_TASK_CORE);
    if (created != pdPASS) return false;
  }

#if ENABLE_MICRO_ROS
  if (!rosTaskHandle) {
    const BaseType_t created = xTaskCreatePinnedToCore(
        rosTask,
        "RosTask",
        ROS_TASK_STACK_BYTES,
        nullptr,
        ROS_TASK_PRIORITY,
        &rosTaskHandle,
        ROS_TASK_CORE);
    if (created != pdPASS) return false;
  }
#endif

  return true;
}

bool getRobotStateSnapshot(RobotStateSnapshot &snapshot) {
  portENTER_CRITICAL(&snapshotMux);
  snapshot = latestSnapshot;
  portEXIT_CRITICAL(&snapshotMux);
  return snapshot.stampMs > 0;
}

bool robotRequestMoveXYZAMmDeg(
    float xMm,
    float yMm,
    float zMm,
    bool useA,
    float aDeg,
    float speedMmS,
    float accelMmS2,
    float angularSpeedDegS,
    float angularAccelDegS2,
    uint32_t &commandId) {
  if (!robotCommandQueue) return false;

  RobotCommand command;
  command.type = RobotCommandType::MOVE_XYZ;
  command.xMm = xMm;
  command.yMm = yMm;
  command.zMm = zMm;
  command.speedMmS = speedMmS;
  command.accelMmS2 = accelMmS2;
  command.useA = useA;
  command.aDeg = aDeg;
  command.angularSpeedDegS = angularSpeedDegS;
  command.angularAccelDegS2 = angularAccelDegS2;

  command.id = allocateCommandId();

  if (xQueueSend(robotCommandQueue, &command, 0) != pdTRUE) return false;

  commandId = command.id;
  setMoveCommandPending(command.id);
  return true;
}

bool robotRequestMoveXYZMm(float xMm, float yMm, float zMm, float speedMmS, float accelMmS2, uint32_t &commandId) {
  return robotRequestMoveXYZAMmDeg(xMm, yMm, zMm, false, 0.0f, speedMmS, accelMmS2, 0.0f, 0.0f, commandId);
}

static bool enqueueControlCommand(RobotCommand &command, uint32_t &commandId) {
  if (!robotCommandQueue) return false;

  command.id = allocateCommandId();
  if (xQueueSend(robotCommandQueue, &command, 0) != pdTRUE) return false;

  commandId = command.id;
  setControlCommandPending(command.id);
  return true;
}

bool robotGetMoveCommandStatus(uint32_t commandId, bool &known, bool &accepted) {
  portENTER_CRITICAL(&commandStatusMux);
  const bool matches = lastMoveCommandId == commandId;
  known = matches && lastMoveCommandKnown;
  accepted = known && lastMoveCommandAccepted;
  portEXIT_CRITICAL(&commandStatusMux);
  return matches;
}

bool robotRequestHomeAxis(uint8_t axis, bool setLimits, float minMm, float maxMm, uint16_t fastRpm, uint16_t slowRpm, uint32_t &commandId) {
  RobotCommand command;
  command.type = RobotCommandType::HOME_AXIS;
  command.axis = axis;
  command.flag = setLimits;
  command.minMm = minMm;
  command.maxMm = maxMm;
  command.fastRpm = fastRpm;
  command.slowRpm = slowRpm;
  return enqueueControlCommand(command, commandId);
}

bool robotRequestGoOrigin(uint8_t axis, uint32_t &commandId) {
  RobotCommand command;
  command.type = RobotCommandType::GO_ORIGIN;
  command.axis = axis;
  return enqueueControlCommand(command, commandId);
}

bool robotRequestSetZero(uint8_t axis, float minMm, float maxMm, uint32_t &commandId) {
  RobotCommand command;
  command.type = RobotCommandType::SET_ZERO;
  command.axis = axis;
  command.minMm = minMm;
  command.maxMm = maxMm;
  return enqueueControlCommand(command, commandId);
}

bool robotRequestSetAxisLimits(uint8_t axis, float minMm, float maxMm, uint32_t &commandId) {
  RobotCommand command;
  command.type = RobotCommandType::SET_AXIS_LIMITS;
  command.axis = axis;
  command.minMm = minMm;
  command.maxMm = maxMm;
  return enqueueControlCommand(command, commandId);
}

bool robotRequestSetAxisEnable(uint8_t axis, bool enable, uint32_t &commandId) {
  RobotCommand command;
  command.type = RobotCommandType::SET_AXIS_ENABLE;
  command.axis = axis;
  command.flag = enable;
  return enqueueControlCommand(command, commandId);
}

bool robotRequestClearFault(uint32_t &commandId) {
  RobotCommand command;
  command.type = RobotCommandType::CLEAR_FAULT;
  return enqueueControlCommand(command, commandId);
}

bool robotRequestReleaseStall(uint8_t axis, uint32_t &commandId) {
  RobotCommand command;
  command.type = RobotCommandType::RELEASE_STALL;
  command.axis = axis;
  return enqueueControlCommand(command, commandId);
}

bool robotRequestAuxServoAngle(float angleDeg, uint32_t &commandId) {
  RobotCommand command;
  command.type = RobotCommandType::AUX_SERVO_ANGLE;
  command.xMm = angleDeg;
  return enqueueControlCommand(command, commandId);
}

bool robotRequestAuxServoPulseUs(uint16_t pulseUs, uint32_t &commandId) {
  RobotCommand command;
  command.type = RobotCommandType::AUX_SERVO_PULSE_US;
  command.pulseUs = pulseUs;
  return enqueueControlCommand(command, commandId);
}

bool robotRequestAuxServoDisable(uint32_t &commandId) {
  RobotCommand command;
  command.type = RobotCommandType::AUX_SERVO_DISABLE;
  return enqueueControlCommand(command, commandId);
}

bool robotGetControlCommandStatus(uint32_t commandId, bool &known, bool &accepted) {
  portENTER_CRITICAL(&commandStatusMux);
  const bool matches = lastControlCommandId == commandId;
  known = matches && lastControlCommandKnown;
  accepted = known && lastControlCommandAccepted;
  portEXIT_CRITICAL(&commandStatusMux);
  return matches;
}

bool robotRequestCommandText(const char *commandText) {
  if (!robotCommandQueue || !commandText || commandText[0] == '\0') return false;

  RobotCommand command;
  command.type = RobotCommandType::TEXT;
  snprintf(command.text, sizeof(command.text), "%s", commandText);
  return xQueueSend(robotCommandQueue, &command, 0) == pdTRUE;
}

void robotRequestStop() {
  if (!robotCommandQueue) return;

  RobotCommand command;
  command.type = RobotCommandType::STOP;
  xQueueSend(robotCommandQueue, &command, 0);
}

void robotRequestEmergencyStop() {
  portENTER_CRITICAL(&emergencyMux);
  emergencyStopRequested = true;
  portEXIT_CRITICAL(&emergencyMux);
}

float robotCommandablePositionMm(float mm) {
  return encoderCountsToMm(mmToEncoderCounts(mm));
}
