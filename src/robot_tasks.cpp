#include "robot_tasks.h"

#include <string.h>

#include "config.h"
#include "machine.h"
#include "ros_bridge.h"
#include "serial_commands.h"

enum class RobotCommandType : uint8_t {
  MOVE_XYZ,
  STOP,
};

struct RobotCommand {
  RobotCommandType type = RobotCommandType::STOP;
  uint32_t id = 0;
  float xMm = 0.0f;
  float yMm = 0.0f;
  float zMm = 0.0f;
  float speedMmS = 0.0f;
  float accelMmS2 = 0.0f;
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
  next.safetyFault = safetyFaultIsActive();
  snprintf(next.safetyReason, sizeof(next.safetyReason), "%s", safetyFaultText());
  next.stampMs = now;

  for (size_t i = 0; i < 4; i++) {
    const MotorNode &motor = motors[i];
    next.encoder[i] = motor.encoderOk ? motor.encoder : 0;
    next.motorPositionMm[i] = motor.encoderOk ? encoderCountsToMm(motor.encoder) : 0.0f;
    next.motorRpm[i] = motor.rpmOk ? motor.rpm : 0;
    next.lastAcc[i] = motor.lastAcc;
    next.moveStatus[i] = motor.moveStatus;
    next.homeStatus[i] = motor.homeStatus;
    next.motorOnline[i] = motorIsOnline(motor, now);
    next.encoderOk[i] = motor.encoderOk;
    next.rpmOk[i] = motor.rpmOk;
  }

  portENTER_CRITICAL(&snapshotMux);
  latestSnapshot = next;
  portEXIT_CRITICAL(&snapshotMux);
}

static void processRobotCommand(const RobotCommand &command) {
  switch (command.type) {
    case RobotCommandType::MOVE_XYZ: {
      const bool accepted = commandMoveXYZMm(
          command.xMm,
          command.yMm,
          command.zMm,
          command.speedMmS,
          command.accelMmS2);
      setMoveCommandStatus(command.id, accepted);
      break;
    }
    case RobotCommandType::STOP:
      stopAllMotors();
      break;
  }
}

static void controlTask(void *) {
  for (;;) {
    if (takeEmergencyStopRequest()) {
      stopAllMotors();
    }

    pollEncoders();
    drainCanReplies();

    RobotCommand command;
    while (robotCommandQueue && xQueueReceive(robotCommandQueue, &command, 0) == pdTRUE) {
      processRobotCommand(command);
    }

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

bool robotRequestMoveXYZMm(float xMm, float yMm, float zMm, float speedMmS, float accelMmS2, uint32_t &commandId) {
  if (!robotCommandQueue) return false;

  RobotCommand command;
  command.type = RobotCommandType::MOVE_XYZ;
  command.xMm = xMm;
  command.yMm = yMm;
  command.zMm = zMm;
  command.speedMmS = speedMmS;
  command.accelMmS2 = accelMmS2;

  portENTER_CRITICAL(&commandStatusMux);
  command.id = nextCommandId++;
  if (nextCommandId == 0) nextCommandId = 1;
  portEXIT_CRITICAL(&commandStatusMux);

  commandId = command.id;
  setMoveCommandPending(command.id);
  if (xQueueSend(robotCommandQueue, &command, 0) != pdTRUE) return false;
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
