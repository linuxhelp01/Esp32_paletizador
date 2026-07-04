#pragma once

#include <Arduino.h>

#include "config.h"

enum class Axis : uint8_t {
  X,
  X1,
  X2,
  Y,
  Z,
  A,
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
  bool rotaryAxis = false;

  MotorNode(const char *nodeName, uint16_t nodeCanId, int8_t nodeDirection, bool nodeRotaryAxis = false)
      : name(nodeName), canId(nodeCanId), direction(nodeDirection), rotaryAxis(nodeRotaryAxis) {}
};

extern MotorNode motors[ROBOT_MOTOR_COUNT];

bool beginMachine();
void pollEncoders();
void serviceMachine();
void drainCanReplies();
bool stopAllMotors();
bool safetyFaultIsActive();
const char *safetyFaultText();
bool motorIsOnline(const MotorNode &motor, uint32_t nowMs);
bool commandMoveXYZMm(float xMm, float yMm, float zMm, float speedMmS, float accelMmS2);
bool commandMoveXYZAMmDeg(float xMm, float yMm, float zMm, bool useA, float aDeg, float speedMmS, float accelMmS2, float angularSpeedDegS, float angularAccelDegS2);
bool startHomeAxis(Axis axis, bool configureLimits, float minMm, float maxMm, uint16_t fastRpm, uint16_t slowRpm);
bool commandGoOriginAxis(Axis axis);
bool commandSetZeroAxis(Axis axis, bool configureLimits, float minMm, float maxMm);
bool commandSetAxisLimits(Axis axis, float minMm, float maxMm);
bool commandSetAxisEnable(Axis axis, bool enable);
bool commandSetAuxServoAngle(float angleDeg);
bool commandSetAuxServoPulseUs(uint16_t pulseUs);
bool commandDisableAuxServo();
bool getAuxServoState(bool &enabled, uint16_t &pulseUs, float &angleDeg);
bool commandClearSafetyFault();
bool commandReleaseStallAxis(Axis axis);
bool axisLimitsAreConfigured(Axis axis);
bool getAxisLimits(Axis axis, float &minMm, float &maxMm);
bool homingIsActive();
const char *homingStateText();
void getAxisPositionsMm(float &xMm, float &yMm, float &zMm);
bool axisPositionsAreValid();
bool isAtXYZMm(float xMm, float yMm, float zMm, float toleranceMm);
bool isAtXYZAMmDeg(float xMm, float yMm, float zMm, bool useA, float aDeg, float toleranceMm, float angularToleranceDeg);

void printHelp();
void printStatus();
void handleMachineCommand(String line);

Axis parseAxis(String token);
uint16_t clampRpm(int32_t rpm);
uint8_t clampAcc(int32_t acc);
int32_t mmToEncoderCounts(float mm);
int32_t degToEncoderCounts(float deg);
float encoderCountsToMm(int64_t encoderCounts);
float encoderCountsToDeg(int64_t encoderCounts);
float encoderCountsToRad(int64_t encoderCounts);
float motorPositionUnits(const MotorNode &motor);
float motorVelocityUnitsPerS(const MotorNode &motor);
float motorJointPosition(const MotorNode &motor);
float motorJointVelocity(const MotorNode &motor);
const char *motorPositionUnit(const MotorNode &motor);
uint16_t linearSpeedMmSToRpm(float speedMmS);
uint16_t angularSpeedDegSToRpm(float speedDegS);
uint8_t angularAccelRpmSToMksAcc(float rpmPerS);
uint8_t linearAccelMmS2ToMksAcc(float mmPerS2);
uint8_t angularAccelDegS2ToMksAcc(float degPerS2);

template <typename Callback>
bool forEachMotorInAxis(Axis axis, Callback callback) {
  bool ok = true;
  switch (axis) {
    case Axis::X:
      ok &= callback(motors[0]);
      ok &= callback(motors[1]);
      break;
    case Axis::X1:
      ok &= callback(motors[0]);
      break;
    case Axis::X2:
      ok &= callback(motors[1]);
      break;
    case Axis::Y:
      ok &= callback(motors[2]);
      break;
    case Axis::Z:
      ok &= callback(motors[3]);
      break;
    case Axis::A:
      ok &= callback(motors[4]);
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
