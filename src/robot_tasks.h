#pragma once

#include <Arduino.h>

struct RobotStateSnapshot {
  bool axisPositionsValid = false;
  float axisPositionMm[3] = {0.0f, 0.0f, 0.0f};
  int64_t encoder[4] = {0, 0, 0, 0};
  float motorPositionMm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  int16_t motorRpm[4] = {0, 0, 0, 0};
  uint8_t lastAcc[4] = {0, 0, 0, 0};
  uint8_t moveStatus[4] = {0, 0, 0, 0};
  uint8_t homeStatus[4] = {0, 0, 0, 0};
  bool motorOnline[4] = {false, false, false, false};
  bool encoderOk[4] = {false, false, false, false};
  bool rpmOk[4] = {false, false, false, false};
  bool safetyFault = false;
  char safetyReason[64] = "OK";
  uint32_t stampMs = 0;
};

bool beginRobotTasks();
bool getRobotStateSnapshot(RobotStateSnapshot &snapshot);
bool robotRequestMoveXYZMm(float xMm, float yMm, float zMm, float speedMmS, float accelMmS2, uint32_t &commandId);
bool robotGetMoveCommandStatus(uint32_t commandId, bool &known, bool &accepted);
void robotRequestStop();
void robotRequestEmergencyStop();
float robotCommandablePositionMm(float mm);

