#pragma once

#include <Arduino.h>

#include "config.h"

struct RobotStateSnapshot {
  bool axisPositionsValid = false;
  float axisPositionMm[3] = {0.0f, 0.0f, 0.0f};
  bool axisLimitsConfigured[3] = {false, false, false};
  float axisMinMm[3] = {0.0f, 0.0f, 0.0f};
  float axisMaxMm[3] = {0.0f, 0.0f, 0.0f};
  int64_t encoder[ROBOT_MOTOR_COUNT] = {};
  int64_t rawDiagnosticEncoder[ROBOT_MOTOR_COUNT] = {};
  float motorPositionMm[ROBOT_MOTOR_COUNT] = {};
  float motorVelocityMmS[ROBOT_MOTOR_COUNT] = {};
  int16_t motorRpm[ROBOT_MOTOR_COUNT] = {};
  int32_t angleError[ROBOT_MOTOR_COUNT] = {};
  uint8_t lastAcc[ROBOT_MOTOR_COUNT] = {};
  uint8_t moveStatus[ROBOT_MOTOR_COUNT] = {};
  uint8_t homeStatus[ROBOT_MOTOR_COUNT] = {};
  uint8_t homeStatusSingleTurn[ROBOT_MOTOR_COUNT] = {};
  uint8_t homeStatusOrigin[ROBOT_MOTOR_COUNT] = {};
  bool enabled[ROBOT_MOTOR_COUNT] = {};
  bool stalled[ROBOT_MOTOR_COUNT] = {};
  bool motorOnline[ROBOT_MOTOR_COUNT] = {};
  bool encoderOk[ROBOT_MOTOR_COUNT] = {};
  bool rawEncoderOk[ROBOT_MOTOR_COUNT] = {};
  bool rpmOk[ROBOT_MOTOR_COUNT] = {};
  bool angleErrorOk[ROBOT_MOTOR_COUNT] = {};
  bool enabledOk[ROBOT_MOTOR_COUNT] = {};
  bool stallOk[ROBOT_MOTOR_COUNT] = {};
  bool homeStatusOk[ROBOT_MOTOR_COUNT] = {};
  bool auxServoEnabled = false;
  uint16_t auxServoPulseUs = 0;
  float auxServoAngleDeg = 0.0f;
  bool homingActive = false;
  char homingState[48] = "idle";
  bool safetyFault = false;
  char safetyReason[64] = "OK";
  uint32_t stampMs = 0;
};

bool beginRobotTasks();
bool getRobotStateSnapshot(RobotStateSnapshot &snapshot);
bool robotRequestMoveXYZMm(float xMm, float yMm, float zMm, float speedMmS, float accelMmS2, uint32_t &commandId);
bool robotRequestMoveXYZAMmDeg(float xMm, float yMm, float zMm, bool useA, float aDeg, float speedMmS, float accelMmS2, float angularSpeedDegS, float angularAccelDegS2, uint32_t &commandId);
bool robotGetMoveCommandStatus(uint32_t commandId, bool &known, bool &accepted);
bool robotRequestHomeAxis(uint8_t axis, bool setLimits, float minMm, float maxMm, uint16_t fastRpm, uint16_t slowRpm, uint32_t &commandId);
bool robotRequestGoOrigin(uint8_t axis, uint32_t &commandId);
bool robotRequestSetZero(uint8_t axis, float minMm, float maxMm, uint32_t &commandId);
bool robotRequestSetAxisLimits(uint8_t axis, float minMm, float maxMm, uint32_t &commandId);
bool robotRequestSetAxisEnable(uint8_t axis, bool enable, uint32_t &commandId);
bool robotRequestClearFault(uint32_t &commandId);
bool robotRequestReleaseStall(uint8_t axis, uint32_t &commandId);
bool robotRequestAuxServoAngle(float angleDeg, uint32_t &commandId);
bool robotRequestAuxServoPulseUs(uint16_t pulseUs, uint32_t &commandId);
bool robotRequestAuxServoDisable(uint32_t &commandId);
bool robotGetControlCommandStatus(uint32_t commandId, bool &known, bool &accepted);
bool robotRequestCommandText(const char *commandText);
void robotRequestStop();
void robotRequestEmergencyStop();
float robotCommandablePositionMm(float mm);
