#pragma once

#include <Arduino.h>

struct RobotStateSnapshot {
  bool axisPositionsValid = false;
  float axisPositionMm[3] = {0.0f, 0.0f, 0.0f};
  bool axisLimitsConfigured[3] = {false, false, false};
  float axisMinMm[3] = {0.0f, 0.0f, 0.0f};
  float axisMaxMm[3] = {0.0f, 0.0f, 0.0f};
  int64_t encoder[4] = {0, 0, 0, 0};
  int64_t rawDiagnosticEncoder[4] = {0, 0, 0, 0};
  float motorPositionMm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float motorVelocityMmS[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  int16_t motorRpm[4] = {0, 0, 0, 0};
  int32_t angleError[4] = {0, 0, 0, 0};
  uint8_t lastAcc[4] = {0, 0, 0, 0};
  uint8_t moveStatus[4] = {0, 0, 0, 0};
  uint8_t homeStatus[4] = {0, 0, 0, 0};
  uint8_t homeStatusSingleTurn[4] = {0, 0, 0, 0};
  uint8_t homeStatusOrigin[4] = {0, 0, 0, 0};
  bool enabled[4] = {false, false, false, false};
  bool stalled[4] = {false, false, false, false};
  bool motorOnline[4] = {false, false, false, false};
  bool encoderOk[4] = {false, false, false, false};
  bool rawEncoderOk[4] = {false, false, false, false};
  bool rpmOk[4] = {false, false, false, false};
  bool angleErrorOk[4] = {false, false, false, false};
  bool enabledOk[4] = {false, false, false, false};
  bool stallOk[4] = {false, false, false, false};
  bool homeStatusOk[4] = {false, false, false, false};
  bool homingActive = false;
  char homingState[48] = "idle";
  bool safetyFault = false;
  char safetyReason[64] = "OK";
  uint32_t stampMs = 0;
};

bool beginRobotTasks();
bool getRobotStateSnapshot(RobotStateSnapshot &snapshot);
bool robotRequestMoveXYZMm(float xMm, float yMm, float zMm, float speedMmS, float accelMmS2, uint32_t &commandId);
bool robotGetMoveCommandStatus(uint32_t commandId, bool &known, bool &accepted);
bool robotRequestHomeAxis(uint8_t axis, bool setLimits, float minMm, float maxMm, uint16_t fastRpm, uint16_t slowRpm, uint32_t &commandId);
bool robotRequestGoOrigin(uint8_t axis, uint32_t &commandId);
bool robotRequestSetZero(uint8_t axis, float minMm, float maxMm, uint32_t &commandId);
bool robotRequestSetAxisLimits(uint8_t axis, float minMm, float maxMm, uint32_t &commandId);
bool robotRequestSetAxisEnable(uint8_t axis, bool enable, uint32_t &commandId);
bool robotRequestClearFault(uint32_t &commandId);
bool robotRequestReleaseStall(uint8_t axis, uint32_t &commandId);
bool robotGetControlCommandStatus(uint32_t commandId, bool &known, bool &accepted);
bool robotRequestCommandText(const char *commandText);
void robotRequestStop();
void robotRequestEmergencyStop();
float robotCommandablePositionMm(float mm);
