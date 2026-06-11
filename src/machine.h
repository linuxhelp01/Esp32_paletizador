#pragma once

#include <Arduino.h>

enum class Axis : uint8_t {
  X,
  Y,
  Y1,
  Y2,
  Z,
  UNKNOWN
};

struct MotorNode {
  const char *name;
  uint16_t canId;
  int8_t direction;
  int64_t rawEncoder = 0;
  int64_t encoder = 0;
  int16_t rpm = 0;
  uint8_t lastAcc = 0;
  uint8_t moveStatus = 0;
  uint8_t homeStatus = 0;
  bool encoderOk = false;
  bool rpmOk = false;
  uint32_t lastSeenMs = 0;

  MotorNode(const char *nodeName, uint16_t nodeCanId, int8_t nodeDirection)
      : name(nodeName), canId(nodeCanId), direction(nodeDirection) {}
};

extern MotorNode motors[4];

bool beginMachine();
void pollEncoders();
void drainCanReplies();

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
      break;
    case Axis::Y:
      ok &= callback(motors[1]);
      ok &= callback(motors[2]);
      break;
    case Axis::Z:
      ok &= callback(motors[3]);
      break;
    default:
      return false;
  }
  return ok;
}
