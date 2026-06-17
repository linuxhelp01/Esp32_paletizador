#pragma once

#include <Arduino.h>

enum class Axis : uint8_t {
  X,
  X1,
  X2,
  Y,
  Z,
  ALL,
  UNKNOWN
};

struct MotorNode {
  const char *name;
  uint16_t canId;
  int8_t direction;
  int64_t rawEncoder = 0;
  int64_t encoder = 0;
  int64_t previousEncoder = 0;
  int64_t diagnosticRawEncoder = 0;
  uint32_t lastEncoderUpdateMs = 0;
  uint32_t lastRawEncoderUpdateMs = 0;
  int16_t rpm = 0;
  float velocityMmS = 0.0f;
  int32_t angleError = 0;
  uint8_t lastAcc = 0;
  uint8_t moveStatus = 0;
  uint8_t homeStatus = 0;
  uint8_t homeStatusSingleTurn = 0;
  uint8_t homeStatusOrigin = 0;
  uint8_t enableCommandStatus = 0;
  bool enabled = false;
  bool stalled = false;
  bool encoderOk = false;
  bool rawEncoderOk = false;
  bool rpmOk = false;
  bool angleErrorOk = false;
  bool enabledOk = false;
  bool stallOk = false;
  bool homeCommandOk = false;
  bool homeStatusOk = false;
  uint32_t lastSeenMs = 0;

  MotorNode(const char *nodeName, uint16_t nodeCanId, int8_t nodeDirection)
      : name(nodeName), canId(nodeCanId), direction(nodeDirection) {}
};

extern MotorNode motors[4];

bool beginMachine();
void pollEncoders();
void serviceMachine();
void drainCanReplies();
bool stopAllMotors();
bool safetyFaultIsActive();
const char *safetyFaultText();
bool motorIsOnline(const MotorNode &motor, uint32_t nowMs);
bool commandMoveXYZMm(float xMm, float yMm, float zMm, float speedMmS, float accelMmS2);
bool startHomeAxis(Axis axis, bool configureLimits, float minMm, float maxMm, uint16_t fastRpm, uint16_t slowRpm);
bool commandGoOriginAxis(Axis axis);
bool commandSetZeroAxis(Axis axis, bool configureLimits, float minMm, float maxMm);
bool commandSetAxisEnable(Axis axis, bool enable);
bool axisLimitsAreConfigured(Axis axis);
bool getAxisLimits(Axis axis, float &minMm, float &maxMm);
bool homingIsActive();
const char *homingStateText();
void getAxisPositionsMm(float &xMm, float &yMm, float &zMm);
bool axisPositionsAreValid();
bool isAtXYZMm(float xMm, float yMm, float zMm, float toleranceMm);

void printHelp();
void printStatus();
void handleMachineCommand(String line);

Axis parseAxis(String token);
uint16_t clampRpm(int32_t rpm);
uint8_t clampAcc(int32_t acc);
int32_t mmToEncoderCounts(float mm);
float encoderCountsToMm(int64_t encoderCounts);
uint16_t linearSpeedMmSToRpm(float speedMmS);
uint8_t angularAccelRpmSToMksAcc(float rpmPerS);
uint8_t linearAccelMmS2ToMksAcc(float mmPerS2);

template <typename Callback>
bool forEachMotorInAxis(Axis axis, Callback callback) {
  bool ok = true;
  switch (axis) {
    case Axis::X:
      ok &= callback(motors[0]);
      ok &= callback(motors[1]);
      break;
    case Axis::Y:
      ok &= callback(motors[2]);
      break;
    case Axis::Z:
      ok &= callback(motors[3]);
      break;
    case Axis::ALL:
      ok &= callback(motors[0]);
      ok &= callback(motors[1]);
      ok &= callback(motors[2]);
      ok &= callback(motors[3]);
      break;
    default:
      return false;
  }
  return ok;
}
