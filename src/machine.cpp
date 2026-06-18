#include "machine.h"

#include "config.h"
#include "mks_can.h"

#if ENABLE_MICRO_ROS
struct NullDebugSerial {
  template <typename... Args>
  void print(Args...) {}

  template <typename... Args>
  void println(Args...) {}
};

static NullDebugSerial DebugSerial;
#else
#define DebugSerial Serial
#endif

MotorNode motors[4] = {
  MotorNode("X1", CAN_ID_PHYSICAL_Y1, MOTOR_DIR_PHYSICAL_Y1),
  MotorNode("X2", CAN_ID_PHYSICAL_Y2, MOTOR_DIR_PHYSICAL_Y2),
  MotorNode("Y", CAN_ID_PHYSICAL_X, MOTOR_DIR_PHYSICAL_X),
  MotorNode("Z", CAN_ID_Z, MOTOR_DIR_Z),
};

static uint32_t lastEncoderPollMs = 0;
static uint32_t lastSpeedPollMs = 0;
static uint32_t lastRawEncoderPollMs = 0;
static uint32_t lastAngleErrorPollMs = 0;
static uint32_t lastHomeStatusPollMs = 0;
static uint32_t lastEnableStatusPollMs = 0;
static uint32_t lastStallStatusPollMs = 0;
static uint32_t encoderPollingPausedUntilMs = 0;
static uint8_t nextEncoderPollMotor = 0;
static uint8_t nextSpeedPollMotor = 0;
static uint8_t nextRawEncoderPollMotor = 0;
static uint8_t nextAngleErrorPollMotor = 0;
static uint8_t nextEnablePollMotor = 0;
static uint8_t nextStallPollMotor = 0;
static bool safetyFaultActive = false;
static const char *safetyFaultReason = "OK";
static bool xPairMotionMonitoringActive = false;
static int8_t xPairExpectedDirection = 0;

static bool commandStopAll();

struct AxisLimitState {
  bool configured = false;
  float minMm = 0.0f;
  float maxMm = 0.0f;
};

enum class HomingPhase : uint8_t {
  IDLE,
  CONFIG_FAST,
  START_FAST,
  WAIT_FAST,
  CONFIG_SLOW,
  START_SLOW,
  WAIT_SLOW,
  COMPLETE,
  FAILED,
};

struct HomingSequenceState {
  bool active = false;
  Axis requestedAxis = Axis::UNKNOWN;
  Axis axes[3] = {Axis::UNKNOWN, Axis::UNKNOWN, Axis::UNKNOWN};
  uint8_t axisCount = 0;
  uint8_t axisIndex = 0;
  HomingPhase phase = HomingPhase::IDLE;
  uint16_t fastRpm = HOME_FAST_RPM;
  uint16_t slowRpm = HOME_SLOW_RPM;
  uint32_t phaseStartedMs = 0;
  bool configureLimits = false;
  float limitMinMm = 0.0f;
  float limitMaxMm = 0.0f;
  char stateText[48] = "idle";
};

static AxisLimitState axisLimits[3];
static HomingSequenceState homing;

static String nextToken(String &line) {
  line.trim();
  const int idx = line.indexOf(' ');
  if (idx < 0) {
    String token = line;
    line = "";
    token.trim();
    return token;
  }

  String token = line.substring(0, idx);
  line = line.substring(idx + 1);
  token.trim();
  return token;
}

static int8_t axisLimitIndex(Axis axis) {
  switch (axis) {
    case Axis::X:
      return 0;
    case Axis::Y:
      return 1;
    case Axis::Z:
      return 2;
    default:
      return -1;
  }
}

static Axis currentHomingAxis() {
  if (!homing.active || homing.axisIndex >= homing.axisCount) return Axis::UNKNOWN;
  return homing.axes[homing.axisIndex];
}

static const char *axisName(Axis axis) {
  switch (axis) {
    case Axis::X:
      return "X";
    case Axis::X1:
      return "X1";
    case Axis::X2:
      return "X2";
    case Axis::Y:
      return "Y";
    case Axis::Z:
      return "Z";
    case Axis::ALL:
      return "ALL";
    default:
      return "?";
  }
}

static uint8_t motorIndex(const MotorNode &motor) {
  for (uint8_t i = 0; i < 4; i++) {
    if (&motors[i] == &motor) return i;
  }
  return 0;
}

static uint8_t homeDirectionForMotor(const MotorNode &motor) {
  switch (motorIndex(motor)) {
    case 0:
      return HOME_DIRECTION_X1;
    case 1:
      return HOME_DIRECTION_X2;
    case 2:
      return HOME_DIRECTION_Y;
    default:
      return HOME_DIRECTION_Z;
  }
}

static MotorNode *findMotorById(uint16_t canId) {
  for (MotorNode &motor : motors) {
    if (motor.canId == canId) return &motor;
  }
  return nullptr;
}

static int32_t toMotorTarget(const MotorNode &motor, int32_t robotTarget) {
  return robotTarget * motor.direction;
}

static int32_t toMotorSpeed(const MotorNode &motor, int32_t robotRpm) {
  return robotRpm * motor.direction;
}

static void pauseEncoderPolling() {
  encoderPollingPausedUntilMs = millis() + MOTION_COMMAND_POLL_PAUSE_MS;
}

static bool sendDriverCommand(uint16_t canId, const uint8_t *cmd, uint8_t len) {
  pauseEncoderPolling();
  mks::waitTxIdle(5);
  for (uint8_t attempt = 0; attempt < CAN_COMMAND_TX_RETRIES; attempt++) {
    if (mks::sendFrame(canId, cmd, len)) return true;
    drainCanReplies();
    mks::waitTxIdle(2);
    delay(1);
  }
  return false;
}

static bool sendTelemetryRequest(uint16_t canId, const uint8_t *cmd, uint8_t len) {
  return mks::sendFrame(canId, cmd, len, CAN_TELEMETRY_TX_TIMEOUT_MS);
}

static int8_t signOfDelta(int64_t value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

static void setSafetyFault(const char *reason) {
  if (!safetyFaultActive) {
    DebugSerial.print("SAFETY FAULT: ");
    DebugSerial.println(reason);
  }
  safetyFaultActive = true;
  safetyFaultReason = reason;
  xPairMotionMonitoringActive = false;
  xPairExpectedDirection = 0;
  commandStopAll();
}

static bool xPairSamplesCloseEnough(bool report) {
  const MotorNode &x1 = motors[0];
  const MotorNode &x2 = motors[1];
  const uint32_t skewMs = x1.lastEncoderUpdateMs > x2.lastEncoderUpdateMs
                              ? x1.lastEncoderUpdateMs - x2.lastEncoderUpdateMs
                              : x2.lastEncoderUpdateMs - x1.lastEncoderUpdateMs;
  if (skewMs <= X_PAIR_ALIGNMENT_SAMPLE_SKEW_MS) return true;
  if (report) {
    DebugSerial.print("X ERROR: muestras X1/X2 no sincronizadas, skewMs=");
    DebugSerial.println(skewMs);
  }
  return false;
}

static bool isXPairAligned(bool requireFreshSamples = false) {
  const MotorNode &x1 = motors[0];
  const MotorNode &x2 = motors[1];
  if (!x1.encoderOk || !x2.encoderOk) {
    DebugSerial.println("X ERROR: falta lectura de encoder en X1 o X2");
    commandStopAll();
    return false;
  }

  if (requireFreshSamples && !xPairSamplesCloseEnough(true)) {
    commandStopAll();
    return false;
  }

  const int64_t error = x1.encoder - x2.encoder;
  if (llabs(error) > X_PAIR_MAX_ERROR_COUNTS) {
    DebugSerial.print("X ERROR: X1/X2 desalineados, errorEnc=");
    DebugSerial.print(error);
    DebugSerial.print(" errorMm=");
    DebugSerial.println(encoderCountsToMm(error), 3);
    commandStopAll();
    return false;
  }

  return true;
}

static bool safetyAllowsMotion() {
  if (!safetyFaultActive) return true;
  DebugSerial.print("SAFETY ERROR: falla activa, ejecute FAULT RESET. motivo=");
  DebugSerial.println(safetyFaultReason);
  commandStopAll();
  return false;
}

static bool setAxisLimits(Axis axis, float minMm, float maxMm) {
  const int8_t index = axisLimitIndex(axis);
  if (index < 0 || !(minMm < maxMm)) return false;

  axisLimits[index].configured = true;
  axisLimits[index].minMm = minMm;
  axisLimits[index].maxMm = maxMm;
  return true;
}

bool getAxisLimits(Axis axis, float &minMm, float &maxMm) {
  const int8_t index = axisLimitIndex(axis);
  if (index < 0) return false;

  minMm = axisLimits[index].minMm;
  maxMm = axisLimits[index].maxMm;
  return axisLimits[index].configured;
}

bool axisLimitsAreConfigured(Axis axis) {
  const int8_t index = axisLimitIndex(axis);
  return index >= 0 && axisLimits[index].configured;
}

static bool targetWithinSoftwareLimits(Axis axis, float targetMm) {
  const int8_t index = axisLimitIndex(axis);
  if (index < 0) return false;
  if (!axisLimits[index].configured) return true;
  return targetMm >= axisLimits[index].minMm && targetMm <= axisLimits[index].maxMm;
}

static bool xyzWithinSoftwareLimits(float xMm, float yMm, float zMm) {
  return targetWithinSoftwareLimits(Axis::X, xMm) &&
         targetWithinSoftwareLimits(Axis::Y, yMm) &&
         targetWithinSoftwareLimits(Axis::Z, zMm);
}

static bool motorIsCommandable(const MotorNode &motor) {
  if (!motor.enabledOk || !motor.enabled) return false;
  if (motor.stallOk && motor.stalled) return false;
  return true;
}

static bool axisIsEnabledAndClear(Axis axis) {
  return forEachMotorInAxis(axis, [](MotorNode &motor) {
    return motorIsCommandable(motor);
  });
}

static bool axisIsSafeForMotion(Axis axis) {
  if (!safetyAllowsMotion()) return false;
  if (axis == Axis::X1 || axis == Axis::X2) {
    DebugSerial.println("X ERROR: X1 y X2 no se mueven por separado. Usa eje X.");
    return false;
  }
  if (axis == Axis::X) return isXPairAligned();
  return axis == Axis::Y || axis == Axis::Z || axis == Axis::ALL;
}

static void setXPairExpectedDirectionFromSpeed(int32_t rpm) {
  xPairMotionMonitoringActive = true;
  xPairExpectedDirection = signOfDelta(rpm);
}

static void setXPairExpectedDirectionFromTarget(int32_t target) {
  xPairMotionMonitoringActive = true;
  const MotorNode &x1 = motors[0];
  const MotorNode &x2 = motors[1];
  if (!x1.encoderOk || !x2.encoderOk) {
    xPairExpectedDirection = 0;
    return;
  }

  const int64_t average = (x1.encoder + x2.encoder) / 2;
  const int64_t delta = static_cast<int64_t>(target) - average;
  xPairExpectedDirection = signOfDelta(delta);
}

static void clearXPairExpectedDirectionIfAxis(Axis axis) {
  if (axis == Axis::X) {
    xPairMotionMonitoringActive = false;
    xPairExpectedDirection = 0;
  }
}

static void verifyXPairDirectionSafety() {
  if (safetyFaultActive || !xPairMotionMonitoringActive) return;
  const MotorNode &x1 = motors[0];
  const MotorNode &x2 = motors[1];
  if (!x1.encoderOk || !x2.encoderOk) return;
  if (!xPairSamplesCloseEnough(false)) return;
  
  if (!isXPairAligned()) {
    setSafetyFault("X1/X2 fuera de alineacion");
    return;
  }

  const int64_t delta1 = x1.encoder - x1.previousEncoder;
  const int64_t delta2 = x2.encoder - x2.previousEncoder;
  const bool x1Moved = llabs(delta1) >= X_PAIR_DIRECTION_MIN_DELTA_COUNTS;
  const bool x2Moved = llabs(delta2) >= X_PAIR_DIRECTION_MIN_DELTA_COUNTS;
  const int8_t dir1 = signOfDelta(delta1);
  const int8_t dir2 = signOfDelta(delta2);

  if (x1Moved && x2Moved && dir1 != dir2) {
    setSafetyFault("X1/X2 se mueven en sentidos opuestos");
    return;
  }

  if (xPairExpectedDirection != 0 &&
      ((x1Moved && dir1 != xPairExpectedDirection) || (x2Moved && dir2 != xPairExpectedDirection))) {
    setSafetyFault("eje X doble se mueve contra el sentido comandado");
  }
}

static bool resetSafetyFault() {
  commandStopAll();
  xPairMotionMonitoringActive = false;
  xPairExpectedDirection = 0;
  if (!isXPairAligned()) return false;

  safetyFaultActive = false;
  safetyFaultReason = "OK";
  DebugSerial.println("SAFETY RESET OK: falla liberada, funcionamiento normal habilitado.");
  return true;
}

Axis parseAxis(String token) {
  token.trim();
  token.toUpperCase();
  if (token == "X") return Axis::X;
  if (token == "X1") return Axis::X1;
  if (token == "X2") return Axis::X2;
  if (token == "Y") return Axis::Y;
  if (token == "Z") return Axis::Z;
  if (token == "ALL" || token == "TODOS") return Axis::ALL;
  return Axis::UNKNOWN;
}

uint16_t clampRpm(int32_t rpm) {
  rpm = abs(rpm);
  if (rpm > MAX_RPM) rpm = MAX_RPM;
  return static_cast<uint16_t>(rpm);
}

uint8_t clampAcc(int32_t acc) {
  if (acc < 0) return 0;
  if (acc > MAX_ACC) return MAX_ACC;
  return static_cast<uint8_t>(acc);
}

int32_t mmToEncoderCounts(float mm) {
  const float counts = mm * ENCODER_COUNTS_PER_MM;
  if (counts > MAX_INT24) return MAX_INT24;
  if (counts < MIN_INT24) return MIN_INT24;
  return static_cast<int32_t>(roundf(counts));
}

float encoderCountsToMm(int64_t encoderCounts) {
  return static_cast<float>(encoderCounts) / ENCODER_COUNTS_PER_MM;
}

uint16_t linearSpeedMmSToRpm(float speedMmS) {
  const float rpm = abs(speedMmS) * 60.0f / LEADSCREW_MM_PER_REV;
  return clampRpm(static_cast<int32_t>(roundf(rpm)));
}

uint8_t angularAccelRpmSToMksAcc(float rpmPerS) {
  rpmPerS = abs(rpmPerS);
  if (rpmPerS <= 0.0f) return 0;

  // Manual MKS: cada incremento de 1 RPM ocurre cada (256 - acc) * 50 us.
  const float accValue = 256.0f - (20000.0f / rpmPerS);
  if (accValue < 1.0f) return 1;
  if (accValue > 255.0f) return 255;
  return static_cast<uint8_t>(roundf(accValue));
}

uint8_t linearAccelMmS2ToMksAcc(float mmPerS2) {
  const float rpmPerS = abs(mmPerS2) * 60.0f / LEADSCREW_MM_PER_REV;
  return angularAccelRpmSToMksAcc(rpmPerS);
}

static void encodeSpeed(int32_t signedRpm, uint8_t &speedHigh, uint8_t &speedLow) {
  const uint16_t rpm12 = clampRpm(signedRpm) & 0x0FFF;
  const uint8_t dirBit = signedRpm < 0 ? mks::DIR_NEGATIVE : mks::DIR_POSITIVE;
  speedHigh = ((rpm12 >> 8) & 0x0F) | dirBit;
  speedLow = rpm12 & 0xFF;
}

static bool commandSpeed(MotorNode &motor, int32_t rpm, uint8_t acc) {
  if (!motorIsCommandable(motor)) return false;

  uint8_t speedHigh = 0;
  uint8_t speedLow = 0;
  encodeSpeed(toMotorSpeed(motor, rpm), speedHigh, speedLow);
  const uint8_t cmd[] = {mks::CMD_RUN_SPEED, speedHigh, speedLow, acc};
  const bool ok = sendDriverCommand(motor.canId, cmd, sizeof(cmd));
  if (ok) motor.lastAcc = acc;
  return ok;
}

static bool commandEmergencyStop(MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_EMERGENCY_STOP};
  return sendDriverCommand(motor.canId, cmd, sizeof(cmd));
}

static bool commandStopAll() {
  bool ok = true;
  for (MotorNode &motor : motors) {
    ok &= commandEmergencyStop(motor);
  }
  return ok;
}

bool stopAllMotors() {
  if (homing.active) {
    homing.active = false;
    homing.phase = HomingPhase::FAILED;
    snprintf(homing.stateText, sizeof(homing.stateText), "aborted:stop");
  }
  xPairMotionMonitoringActive = false;
  xPairExpectedDirection = 0;
  return commandStopAll();
}

bool safetyFaultIsActive() {
  return safetyFaultActive;
}

const char *safetyFaultText() {
  return safetyFaultReason;
}

bool motorIsOnline(const MotorNode &motor, uint32_t nowMs) {
  return motor.lastSeenMs > 0 && (nowMs - motor.lastSeenMs) <= MOTOR_ONLINE_TIMEOUT_MS;
}

static bool encoderSampleIsFresh(const MotorNode &motor, uint32_t nowMs) {
  return motor.encoderOk &&
         motor.lastEncoderUpdateMs > 0 &&
         (nowMs - motor.lastEncoderUpdateMs) <= POSITION_SAMPLE_MAX_AGE_MS;
}

void getAxisPositionsMm(float &xMm, float &yMm, float &zMm) {
  float xSumMm = 0.0f;
  uint8_t xSamples = 0;

  if (motors[0].encoderOk) {
    xSumMm += encoderCountsToMm(motors[0].encoder);
    xSamples++;
  }
  if (motors[1].encoderOk) {
    xSumMm += encoderCountsToMm(motors[1].encoder);
    xSamples++;
  }

  xMm = xSamples ? xSumMm / xSamples : 0.0f;
  yMm = motors[2].encoderOk ? encoderCountsToMm(motors[2].encoder) : 0.0f;
  zMm = motors[3].encoderOk ? encoderCountsToMm(motors[3].encoder) : 0.0f;
}

bool axisPositionsAreValid() {
  const uint32_t now = millis();
  return encoderSampleIsFresh(motors[0], now) &&
         encoderSampleIsFresh(motors[1], now) &&
         encoderSampleIsFresh(motors[2], now) &&
         encoderSampleIsFresh(motors[3], now);
}

bool isAtXYZMm(float xMm, float yMm, float zMm, float toleranceMm) {
  if (!axisPositionsAreValid()) return false;

  float currentX = 0.0f;
  float currentY = 0.0f;
  float currentZ = 0.0f;
  getAxisPositionsMm(currentX, currentY, currentZ);

  return fabsf(currentX - xMm) <= toleranceMm &&
         fabsf(currentY - yMm) <= toleranceMm &&
         fabsf(currentZ - zMm) <= toleranceMm;
}

static bool commandConfigureHome(MotorNode &motor, uint16_t rpm) {
  rpm = clampRpm(rpm);
  const uint8_t cmd[] = {
    mks::CMD_SET_HOME_PARAMS,
    HOME_TRIGGER_LEVEL,
    homeDirectionForMotor(motor),
    static_cast<uint8_t>((rpm >> 8) & 0xFF),
    static_cast<uint8_t>(rpm & 0xFF),
    HOME_LIMIT_ENABLE,
    HOME_MODE_ORIGIN_SWITCH,
  };
  return sendDriverCommand(motor.canId, cmd, sizeof(cmd));
}

static bool commandHome(MotorNode &motor) {
  motor.homeStatus = 0;
  motor.homeCommandOk = false;
  motor.homeStatusSingleTurn = 0;
  motor.homeStatusOrigin = 0;
  motor.homeStatusOk = false;
  const uint8_t cmd[] = {mks::CMD_HOME, 0x00};
  return sendDriverCommand(motor.canId, cmd, sizeof(cmd));
}

static bool commandGoOrigin(MotorNode &motor) {
  if (!motorIsCommandable(motor)) return false;
  motor.homeStatus = 0;
  motor.homeCommandOk = false;
  motor.homeStatusSingleTurn = 0;
  motor.homeStatusOrigin = 0;
  motor.homeStatusOk = false;
  const uint8_t cmd[] = {mks::CMD_HOME, 0x01};
  return sendDriverCommand(motor.canId, cmd, sizeof(cmd));
}

static bool commandSetZero(MotorNode &motor) {
  if (!motorIsCommandable(motor)) return false;
  const uint8_t cmd[] = {mks::CMD_SET_ZERO};
  return sendDriverCommand(motor.canId, cmd, sizeof(cmd));
}

static bool commandSetEnable(MotorNode &motor, bool enable) {
  const uint8_t cmd[] = {mks::CMD_SET_ENABLE, static_cast<uint8_t>(enable ? 0x01 : 0x00)};
  const bool ok = sendDriverCommand(motor.canId, cmd, sizeof(cmd));
  if (ok) {
    motor.enableCommandStatus = 0;
    motor.enabledOk = false;
  }
  return ok;
}

static bool commandReleaseStall(MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_RELEASE_STALL};
  const bool ok = sendDriverCommand(motor.canId, cmd, sizeof(cmd));
  if (ok) motor.stallOk = false;
  return ok;
}

static void setHomingText(const char *phase, Axis axis) {
  snprintf(homing.stateText, sizeof(homing.stateText), "%s:%s", phase, axisName(axis));
}

static bool configureHomingAxes(Axis axis) {
  homing.axisCount = 0;
  homing.axisIndex = 0;
  if (axis == Axis::ALL) {
    homing.axes[0] = Axis::X;
    homing.axes[1] = Axis::Y;
    homing.axes[2] = Axis::Z;
    homing.axisCount = 3;
    return true;
  }
  if (axis == Axis::X || axis == Axis::Y || axis == Axis::Z) {
    homing.axes[0] = axis;
    homing.axisCount = 1;
    return true;
  }
  return false;
}

static bool homingAxisHasFailure(Axis axis) {
  bool failed = false;
  forEachMotorInAxis(axis, [&failed](MotorNode &motor) {
    if (motor.homeStatus == 0 && motor.homeCommandOk) failed = true;
    if (motor.homeStatusOrigin == 2 && motor.homeStatusOk) failed = true;
    return true;
  });
  return failed;
}

static bool homingAxisIsComplete(Axis axis) {
  bool complete = true;
  forEachMotorInAxis(axis, [&complete](MotorNode &motor) {
    const bool commandComplete = motor.homeCommandOk && motor.homeStatus == 2;
    const bool originComplete = motor.homeStatusOk && motor.homeStatusOrigin == 1;
    complete &= commandComplete || originComplete;
    return true;
  });
  return complete;
}

static void failHoming(const char *reason) {
  homing.active = false;
  homing.phase = HomingPhase::FAILED;
  snprintf(homing.stateText, sizeof(homing.stateText), "failed:%s", reason);
  xPairMotionMonitoringActive = false;
  xPairExpectedDirection = 0;
  commandStopAll();
}

static void finishCurrentHomingAxis() {
  const Axis axis = currentHomingAxis();
  forEachMotorInAxis(axis, [](MotorNode &motor) {
    const uint8_t readEncoder[] = {mks::CMD_READ_ENCODER};
    const uint8_t readRaw[] = {mks::CMD_READ_RAW_ENCODER};
    mks::sendFrame(motor.canId, readEncoder, sizeof(readEncoder));
    mks::sendFrame(motor.canId, readRaw, sizeof(readRaw));
    return true;
  });

  if (homing.configureLimits && homing.axisCount == 1) {
    setAxisLimits(axis, homing.limitMinMm, homing.limitMaxMm);
  }

  homing.axisIndex++;
  if (homing.axisIndex >= homing.axisCount) {
    homing.active = false;
    homing.phase = HomingPhase::COMPLETE;
    snprintf(homing.stateText, sizeof(homing.stateText), "complete:%s", axisName(homing.requestedAxis));
    clearXPairExpectedDirectionIfAxis(axis);
    return;
  }

  homing.phase = HomingPhase::CONFIG_FAST;
  homing.phaseStartedMs = millis();
  setHomingText("config_fast", currentHomingAxis());
}

bool startHomeAxis(Axis axis, bool configureLimits, float minMm, float maxMm, uint16_t fastRpm, uint16_t slowRpm) {
  if (homing.active) return false;
  if (!safetyAllowsMotion()) return false;
  if (!configureHomingAxes(axis)) return false;
  if (configureLimits && homing.axisCount != 1) return false;
  if (configureLimits && !(minMm < maxMm)) return false;

  for (uint8_t i = 0; i < homing.axisCount; i++) {
    if (!axisIsEnabledAndClear(homing.axes[i])) return false;
    if (homing.axes[i] == Axis::X && !isXPairAligned()) return false;
  }

  homing.active = true;
  homing.requestedAxis = axis;
  homing.phase = HomingPhase::CONFIG_FAST;
  homing.fastRpm = fastRpm > 0 ? clampRpm(fastRpm) : HOME_FAST_RPM;
  homing.slowRpm = slowRpm > 0 ? clampRpm(slowRpm) : HOME_SLOW_RPM;
  homing.configureLimits = configureLimits;
  homing.limitMinMm = minMm;
  homing.limitMaxMm = maxMm;
  homing.phaseStartedMs = millis();
  setHomingText("config_fast", currentHomingAxis());
  if (axis == Axis::X) {
    xPairMotionMonitoringActive = true;
    xPairExpectedDirection = 0;
  }
  return true;
}

bool commandGoOriginAxis(Axis axis) {
  if (!safetyAllowsMotion() || homing.active) return false;
  if (axis == Axis::X1 || axis == Axis::X2 || axis == Axis::UNKNOWN) return false;
  if (!axisIsEnabledAndClear(axis)) return false;
  if (axis == Axis::X && !isXPairAligned()) return false;
  if (axis == Axis::ALL && !isXPairAligned()) return false;
  return forEachMotorInAxis(axis, [](MotorNode &motor) {
    return commandGoOrigin(motor);
  });
}

bool commandSetZeroAxis(Axis axis, bool configureLimits, float minMm, float maxMm) {
  if (!safetyAllowsMotion() || homing.active) return false;
  if (axis == Axis::X1 || axis == Axis::X2 || axis == Axis::ALL || axis == Axis::UNKNOWN) return false;
  if (configureLimits && !setAxisLimits(axis, minMm, maxMm)) return false;
  if (!axisIsEnabledAndClear(axis)) return false;
  if (axis == Axis::X && !isXPairAligned()) return false;
  return forEachMotorInAxis(axis, [](MotorNode &motor) {
    return commandSetZero(motor);
  });
}

bool commandSetAxisLimits(Axis axis, float minMm, float maxMm) {
  if (axis == Axis::ALL || axis == Axis::X1 || axis == Axis::X2 || axis == Axis::UNKNOWN) return false;
  return setAxisLimits(axis, minMm, maxMm);
}

bool commandSetAxisEnable(Axis axis, bool enable) {
  if (axis == Axis::UNKNOWN) return false;
  if (enable && safetyFaultActive) return false;
  if (!enable) {
    if (homing.active) {
      homing.active = false;
      homing.phase = HomingPhase::FAILED;
      snprintf(homing.stateText, sizeof(homing.stateText), "aborted:disable");
    }
    clearXPairExpectedDirectionIfAxis(axis);
  }
  return forEachMotorInAxis(axis, [enable](MotorNode &motor) {
    return commandSetEnable(motor, enable);
  });
}

bool commandClearSafetyFault() {
  return resetSafetyFault();
}

bool commandReleaseStallAxis(Axis axis) {
  if (axis == Axis::UNKNOWN) return false;
  return forEachMotorInAxis(axis, [](MotorNode &motor) {
    return commandReleaseStall(motor);
  });
}

static bool commandSetBusFocMode(MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_SET_MODE, 0x05};
  return sendDriverCommand(motor.canId, cmd, sizeof(cmd));
}

static bool commandSetBusFocModeBroadcast() {
  const uint8_t cmd[] = {mks::CMD_SET_MODE, 0x05};
  return sendDriverCommand(mks::CAN_ID_BROADCAST, cmd, sizeof(cmd));
}

static bool commandEnableSyncBroadcast(bool enable) {
  const uint8_t cmd[] = {mks::CMD_SYNC_ENABLE, static_cast<uint8_t>(enable ? 0x01 : 0x00)};
  return sendDriverCommand(mks::CAN_ID_BROADCAST, cmd, sizeof(cmd));
}

static bool commandSyncTrigger() {
  if (!mks::waitTxIdle(10)) return false;
  const uint8_t cmd[] = {mks::CMD_SYNC_TRIGGER};
  bool ok = true;
  for (uint8_t i = 0; i < 3; i++) {
    ok &= sendDriverCommand(mks::CAN_ID_BROADCAST, cmd, sizeof(cmd));
    delay(1);
  }
  return ok;
}

static bool commandPrepareSynchronizedMove() {
  pauseEncoderPolling();
  drainCanReplies();
  const bool idle = mks::waitTxIdle(10);
  const bool ok = idle && commandEnableSyncBroadcast(true);
  delay(2);
  return ok;
}

static bool commandAbsolutePosition(MotorNode &motor, int32_t encoderTarget, uint16_t rpm, uint8_t acc) {
  if (!motorIsCommandable(motor)) return false;

  encoderTarget = toMotorTarget(motor, encoderTarget);
  if (encoderTarget < MIN_INT24) encoderTarget = MIN_INT24;
  if (encoderTarget > MAX_INT24) encoderTarget = MAX_INT24;

  const uint32_t coord24 = static_cast<uint32_t>(encoderTarget) & 0x00FFFFFF;
  const uint8_t cmd[] = {
    mks::CMD_RUN_ABS_COORD,
    static_cast<uint8_t>((rpm >> 8) & 0xFF),
    static_cast<uint8_t>(rpm & 0xFF),
    acc,
    static_cast<uint8_t>((coord24 >> 16) & 0xFF),
    static_cast<uint8_t>((coord24 >> 8) & 0xFF),
    static_cast<uint8_t>(coord24 & 0xFF),
  };

  const bool ok = sendDriverCommand(motor.canId, cmd, sizeof(cmd));
  if (ok) motor.lastAcc = acc;
  return ok;
}

bool commandMoveXYZMm(float xMm, float yMm, float zMm, float speedMmS, float accelMmS2) {
  const int32_t xTarget = mmToEncoderCounts(xMm);
  const int32_t yTarget = mmToEncoderCounts(yMm);
  const int32_t zTarget = mmToEncoderCounts(zMm);
  const uint16_t rpm = speedMmS > 0.0f ? linearSpeedMmSToRpm(speedMmS) : DEFAULT_RPM;
  const uint8_t acc = accelMmS2 > 0.0f ? linearAccelMmS2ToMksAcc(accelMmS2) : DEFAULT_ACC;

  if (!safetyAllowsMotion() || !isXPairAligned()) return false;
  if (homing.active) return false;
  if (!axisIsEnabledAndClear(Axis::ALL)) return false;
  if (!xyzWithinSoftwareLimits(xMm, yMm, zMm)) return false;

  setXPairExpectedDirectionFromTarget(xTarget);
  bool ok = commandPrepareSynchronizedMove();
  ok &= commandAbsolutePosition(motors[0], xTarget, rpm, acc);
  ok &= commandAbsolutePosition(motors[1], xTarget, rpm, acc);
  ok &= commandAbsolutePosition(motors[2], yTarget, rpm, acc);
  ok &= commandAbsolutePosition(motors[3], zTarget, rpm, acc);
  if (ok) ok &= commandSyncTrigger();
  if (!ok) {
    xPairMotionMonitoringActive = false;
    xPairExpectedDirection = 0;
  }
  return ok;
}

static bool requestEncoder(const MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_READ_ENCODER};
  return sendTelemetryRequest(motor.canId, cmd, sizeof(cmd));
}

static bool requestSpeed(const MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_READ_SPEED_RPM};
  return sendTelemetryRequest(motor.canId, cmd, sizeof(cmd));
}

static bool requestRawEncoder(const MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_READ_RAW_ENCODER};
  return sendTelemetryRequest(motor.canId, cmd, sizeof(cmd));
}

static bool requestAngleError(const MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_READ_ANGLE_ERROR};
  return sendTelemetryRequest(motor.canId, cmd, sizeof(cmd));
}

static bool requestEnableStatus(const MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_READ_ENABLE};
  return sendTelemetryRequest(motor.canId, cmd, sizeof(cmd));
}

static bool requestHomeStatus(const MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_READ_HOME_STATUS};
  return sendTelemetryRequest(motor.canId, cmd, sizeof(cmd));
}

static bool requestStallStatus(const MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_READ_STALL};
  return sendTelemetryRequest(motor.canId, cmd, sizeof(cmd));
}

static void handleEncoderReply(MotorNode &motor, const twai_message_t &msg) {
  if (msg.data_length_code < 8) return;

  int64_t value = 0;
  for (uint8_t i = 1; i <= 6; i++) {
    value = (value << 8) | msg.data[i];
  }
  if (value & 0x0000800000000000LL) {
    value |= 0xFFFF000000000000LL;
  }

  const uint32_t now = millis();
  if (motor.encoderOk && motor.lastEncoderUpdateMs != now) {
    const uint32_t dtMs = now - motor.lastEncoderUpdateMs;
    if (dtMs > 0) {
      const int64_t delta = (value * motor.direction) - motor.encoder;
      const float instantVelocityMmS = encoderCountsToMm(delta) * 1000.0f / static_cast<float>(dtMs);
      motor.velocityMmS = 0.7f * motor.velocityMmS + 0.3f * instantVelocityMmS;
    }
  }

  motor.rawEncoder = value;
  motor.previousEncoder = motor.encoder;
  motor.encoder = value * motor.direction;
  motor.encoderOk = true;
  motor.lastEncoderUpdateMs = now;
  motor.lastSeenMs = motor.lastEncoderUpdateMs;
  verifyXPairDirectionSafety();
}

static void handleSpeedReply(MotorNode &motor, const twai_message_t &msg) {
  if (msg.data_length_code < 4) return;

  const int16_t rawRpm = static_cast<int16_t>((static_cast<uint16_t>(msg.data[1]) << 8) | msg.data[2]);
  motor.rpm = rawRpm * motor.direction;
  motor.rpmOk = true;
  motor.lastSeenMs = millis();
}

static void handleRawEncoderReply(MotorNode &motor, const twai_message_t &msg) {
  if (msg.data_length_code < 8) return;

  int64_t value = 0;
  for (uint8_t i = 1; i <= 6; i++) {
    value = (value << 8) | msg.data[i];
  }
  if (value & 0x0000800000000000LL) {
    value |= 0xFFFF000000000000LL;
  }

  motor.diagnosticRawEncoder = value * motor.direction;
  motor.rawEncoderOk = true;
  motor.lastRawEncoderUpdateMs = millis();
  motor.lastSeenMs = motor.lastRawEncoderUpdateMs;
}

static void handleAngleErrorReply(MotorNode &motor, const twai_message_t &msg) {
  if (msg.data_length_code >= 6) {
    motor.angleError =
        (static_cast<int32_t>(msg.data[1]) << 24) |
        (static_cast<int32_t>(msg.data[2]) << 16) |
        (static_cast<int32_t>(msg.data[3]) << 8) |
        static_cast<int32_t>(msg.data[4]);
  } else if (msg.data_length_code >= 4) {
    motor.angleError = static_cast<int16_t>((static_cast<uint16_t>(msg.data[1]) << 8) | msg.data[2]);
  } else {
    return;
  }
  motor.angleErrorOk = true;
  motor.lastSeenMs = millis();
}

static void handleEnableReply(MotorNode &motor, const twai_message_t &msg) {
  if (msg.data_length_code < 3) return;
  motor.enabled = msg.data[1] != 0;
  motor.enabledOk = true;
  motor.lastSeenMs = millis();
}

static void handleHomeStatusReply(MotorNode &motor, const twai_message_t &msg) {
  if (msg.data_length_code < 4) return;
  motor.homeStatusSingleTurn = msg.data[1];
  motor.homeStatusOrigin = msg.data[2];
  motor.homeStatusOk = true;
  motor.lastSeenMs = millis();
}

static void handleStallReply(MotorNode &motor, const twai_message_t &msg) {
  if (msg.data_length_code < 3) return;
  motor.stalled = msg.data[1] != 0;
  motor.stallOk = true;
  motor.lastSeenMs = millis();
}

static void handleStatusReply(MotorNode &motor, const twai_message_t &msg) {
  if (msg.data_length_code < 3) return;
  const uint8_t code = msg.data[0];
  const uint8_t status = msg.data[1];

  if (code == mks::CMD_HOME) {
    motor.homeStatus = status;
    motor.homeCommandOk = true;
  } else if (code == mks::CMD_SET_ENABLE) {
    motor.enableCommandStatus = status;
    if (status == 1) {
      motor.enabledOk = false;
    }
  } else if (code == mks::CMD_RUN_ABS_COORD || code == mks::CMD_RUN_SPEED || code == mks::CMD_EMERGENCY_STOP) {
    motor.moveStatus = status;
  }
  motor.lastSeenMs = millis();
}

void drainCanReplies() {
  twai_message_t rx = {};
  while (mks::readAnyFrame(rx, 0)) {
    MotorNode *motor = findMotorById(rx.identifier);
    if (!motor) continue;
    if (!mks::verifyChecksum(rx)) continue;

    switch (rx.data[0]) {
      case mks::CMD_READ_ENCODER:
        handleEncoderReply(*motor, rx);
        break;
      case mks::CMD_READ_SPEED_RPM:
        handleSpeedReply(*motor, rx);
        break;
      case mks::CMD_READ_RAW_ENCODER:
        handleRawEncoderReply(*motor, rx);
        break;
      case mks::CMD_READ_ANGLE_ERROR:
        handleAngleErrorReply(*motor, rx);
        break;
      case mks::CMD_READ_ENABLE:
        handleEnableReply(*motor, rx);
        break;
      case mks::CMD_READ_HOME_STATUS:
        handleHomeStatusReply(*motor, rx);
        break;
      case mks::CMD_READ_STALL:
        handleStallReply(*motor, rx);
        break;
      case mks::CMD_RUN_ABS_COORD:
      case mks::CMD_RUN_SPEED:
      case mks::CMD_EMERGENCY_STOP:
      case mks::CMD_HOME:
      case mks::CMD_SET_ZERO:
      case mks::CMD_SET_MODE:
      case mks::CMD_SET_ENABLE:
      case mks::CMD_SET_HOME_PARAMS:
      case mks::CMD_RELEASE_STALL:
        handleStatusReply(*motor, rx);
        break;
      default:
        motor->lastSeenMs = millis();
        break;
    }
  }
}

void pollEncoders() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - encoderPollingPausedUntilMs) < 0) return;

  if (now - lastEncoderPollMs >= ENCODER_POLL_MS) {
    lastEncoderPollMs = now;
    for (uint8_t i = 0; i < ENCODER_REQUESTS_PER_POLL; i++) {
      requestEncoder(motors[nextEncoderPollMotor]);
      nextEncoderPollMotor = (nextEncoderPollMotor + 1) % 4;
    }
  }

  if (now - lastSpeedPollMs >= SPEED_POLL_MS) {
    lastSpeedPollMs = now;
    requestSpeed(motors[nextSpeedPollMotor]);
    nextSpeedPollMotor = (nextSpeedPollMotor + 1) % 4;
  }

  if (homing.active && now - lastHomeStatusPollMs >= HOME_STATUS_POLL_MS) {
    lastHomeStatusPollMs = now;
    const Axis axis = currentHomingAxis();
    forEachMotorInAxis(axis, [](MotorNode &motor) {
      requestHomeStatus(motor);
      return true;
    });
  }

  if (!homing.active && now - lastRawEncoderPollMs >= RAW_ENCODER_DIAGNOSTIC_POLL_MS) {
    lastRawEncoderPollMs = now;
    requestRawEncoder(motors[nextRawEncoderPollMotor]);
    nextRawEncoderPollMotor = (nextRawEncoderPollMotor + 1) % 4;
  }

  if (now - lastAngleErrorPollMs >= ANGLE_ERROR_POLL_MS) {
    lastAngleErrorPollMs = now;
    requestAngleError(motors[nextAngleErrorPollMotor]);
    nextAngleErrorPollMotor = (nextAngleErrorPollMotor + 1) % 4;
  }

  if (now - lastEnableStatusPollMs >= ENABLE_STATUS_POLL_MS) {
    lastEnableStatusPollMs = now;
    requestEnableStatus(motors[nextEnablePollMotor]);
    nextEnablePollMotor = (nextEnablePollMotor + 1) % 4;
  }

  if (now - lastStallStatusPollMs >= STALL_STATUS_POLL_MS) {
    lastStallStatusPollMs = now;
    requestStallStatus(motors[nextStallPollMotor]);
    nextStallPollMotor = (nextStallPollMotor + 1) % 4;
  }
}

void serviceMachine() {
  if (!homing.active) return;

  const uint32_t now = millis();
  const Axis axis = currentHomingAxis();
  if (axis == Axis::UNKNOWN) {
    failHoming("axis");
    return;
  }

  if (homing.phase != HomingPhase::CONFIG_FAST &&
      homing.phase != HomingPhase::START_FAST &&
      homing.phase != HomingPhase::CONFIG_SLOW &&
      homing.phase != HomingPhase::START_SLOW &&
      now - homing.phaseStartedMs > HOME_PHASE_TIMEOUT_MS) {
    failHoming("timeout");
    return;
  }

  if (homingAxisHasFailure(axis)) {
    failHoming("driver");
    return;
  }

  switch (homing.phase) {
    case HomingPhase::CONFIG_FAST:
      {
        const uint16_t rpm = homing.fastRpm;
        if (!forEachMotorInAxis(axis, [rpm](MotorNode &motor) {
              return commandConfigureHome(motor, rpm);
            })) {
          failHoming("cfg_fast");
          return;
        }
      }
      homing.phase = HomingPhase::START_FAST;
      homing.phaseStartedMs = now;
      setHomingText("start_fast", axis);
      break;

    case HomingPhase::START_FAST:
      if (!forEachMotorInAxis(axis, [](MotorNode &motor) {
            return commandHome(motor);
          })) {
        failHoming("home_fast");
        return;
      }
      homing.phase = HomingPhase::WAIT_FAST;
      homing.phaseStartedMs = now;
      setHomingText("wait_fast", axis);
      break;

    case HomingPhase::WAIT_FAST:
      if (homingAxisIsComplete(axis)) {
        homing.phase = HomingPhase::CONFIG_SLOW;
        homing.phaseStartedMs = now;
        setHomingText("config_slow", axis);
      }
      break;

    case HomingPhase::CONFIG_SLOW:
      {
        const uint16_t rpm = homing.slowRpm;
        if (!forEachMotorInAxis(axis, [rpm](MotorNode &motor) {
              return commandConfigureHome(motor, rpm);
            })) {
          failHoming("cfg_slow");
          return;
        }
      }
      homing.phase = HomingPhase::START_SLOW;
      homing.phaseStartedMs = now;
      setHomingText("start_slow", axis);
      break;

    case HomingPhase::START_SLOW:
      if (!forEachMotorInAxis(axis, [](MotorNode &motor) {
            return commandHome(motor);
          })) {
        failHoming("home_slow");
        return;
      }
      homing.phase = HomingPhase::WAIT_SLOW;
      homing.phaseStartedMs = now;
      setHomingText("wait_slow", axis);
      break;

    case HomingPhase::WAIT_SLOW:
      if (homingAxisIsComplete(axis)) {
        finishCurrentHomingAxis();
      }
      break;

    default:
      break;
  }
}

bool homingIsActive() {
  return homing.active;
}

const char *homingStateText() {
  return homing.stateText;
}

void printHelp() {
  DebugSerial.println();
  DebugSerial.println("Comandos:");
  DebugSerial.println("  HELLO");
  DebugSerial.println("  POS <X|Y|Z> <encoder_abs> [rpm] [acc]");
  DebugSerial.println("      Posicion angular absoluta del motor en cuentas de encoder.");
  DebugSerial.println("  POSANG <X|Y|Z> <encoder_abs> [rpm] [rpm_s]");
  DebugSerial.println("      Posicion angular. En X mueve X1/X2 juntos.");
  DebugSerial.println("  POSMM <X|Y|Z> <mm_abs> [rpm] [acc]");
  DebugSerial.println("      Posicion lineal absoluta del extremo del robot en milimetros.");
  DebugSerial.println("  POSLINE <X|Y|Z> <mm_abs> [mm_s] [mm_s2]");
  DebugSerial.println("      Eje del robot: posicion, velocidad y aceleracion lineal.");
  DebugSerial.println("  POSXYZ <x_mm> <y_mm> <z_mm> [mm_s] [mm_s2]");
  DebugSerial.println("      Movimiento lineal coordinado XYZ con disparo broadcast 4Bh.");
  DebugSerial.println("  SYNC <0|1>         (configura marca sincronizada MKS 4Ah por broadcast)");
  DebugSerial.println("  TRIGGER            (disparo sincronizado MKS 4Bh por broadcast)");
  DebugSerial.println("  VEL <X|Y|Z> <rpm> [acc]");
  DebugSerial.println("  VELANG <X|Y|Z> <rpm> [rpm_s]");
  DebugSerial.println("  HOME <X|Y|Z|ALL> [min_mm max_mm] [fast_rpm slow_rpm]");
  DebugSerial.println("      0x91 0x00 doble pasada: rapida y luego lenta.");
  DebugSerial.println("  ORIGIN <X|Y|Z|ALL>");
  DebugSerial.println("      0x91 0x01: vuelve al origen de coordenadas ya definido.");
  DebugSerial.println("  ZERO <X|Y|Z> <min_mm> <max_mm>");
  DebugSerial.println("      0x92: define cero actual y limites de software del eje.");
  DebugSerial.println("  ENABLE <X|Y|Z|ALL>");
  DebugSerial.println("  DISABLE <X|Y|Z|ALL>");
  DebugSerial.println("  PING              (fuerza consulta CAN a todos los motores)");
  DebugSerial.println("  STOP");
  DebugSerial.println("  FAULT STATUS      (muestra el estado de seguridad)");
  DebugSerial.println("  FAULT RESET       (libera falla si X1/X2 tienen encoder valido y estan alineados)");
  DebugSerial.println("  FAULT TEST        (simula falla del eje X doble para probar enclavamiento)");
  DebugSerial.println("  STATUS");
  DebugSerial.println();
}

void printStatus() {
  DebugSerial.println("STATUS");
  DebugSerial.print("  safetyFault=");
  DebugSerial.print(safetyFaultActive ? 1 : 0);
  DebugSerial.print(" reason=");
  DebugSerial.println(safetyFaultReason);
  DebugSerial.print("  homing=");
  DebugSerial.println(homing.stateText);
  for (Axis axis : {Axis::X, Axis::Y, Axis::Z}) {
    const int8_t index = axisLimitIndex(axis);
    DebugSerial.print("  limit ");
    DebugSerial.print(axisName(axis));
    DebugSerial.print(" configured=");
    DebugSerial.print(axisLimits[index].configured ? 1 : 0);
    DebugSerial.print(" min=");
    DebugSerial.print(axisLimits[index].minMm, 3);
    DebugSerial.print(" max=");
    DebugSerial.println(axisLimits[index].maxMm, 3);
  }
  const uint32_t now = millis();
  for (const MotorNode &motor : motors) {
    const bool online = motorIsOnline(motor, now);
    DebugSerial.print("  ");
    DebugSerial.print(motor.name);
    DebugSerial.print(" id=0x");
    DebugSerial.print(motor.canId, HEX);
    DebugSerial.print(" online=");
    DebugSerial.print(online ? 1 : 0);
    DebugSerial.print(" enabled=");
    if (motor.enabledOk) DebugSerial.print(motor.enabled ? 1 : 0);
    else DebugSerial.print("?");
    DebugSerial.print(" stalled=");
    if (motor.stallOk) DebugSerial.print(motor.stalled ? 1 : 0);
    else DebugSerial.print("?");
    DebugSerial.print(" angularEnc=");
    if (motor.encoderOk) DebugSerial.print(motor.encoder);
    else DebugSerial.print("?");
    DebugSerial.print(" linearMm=");
    if (motor.encoderOk) DebugSerial.print(encoderCountsToMm(motor.encoder), 3);
    else DebugSerial.print("?");
    DebugSerial.print(" rpm=");
    if (motor.rpmOk) DebugSerial.print(motor.rpm);
    else DebugSerial.print("?");
    DebugSerial.print(" velMmS=");
    if (motor.encoderOk) DebugSerial.print(motor.velocityMmS, 3);
    else DebugSerial.print("?");
    DebugSerial.print(" angleErr=");
    if (motor.angleErrorOk) DebugSerial.print(motor.angleError);
    else DebugSerial.print("?");
    DebugSerial.print(" acc=");
    DebugSerial.print(motor.lastAcc);
    DebugSerial.print(" moveStatus=");
    DebugSerial.print(motor.moveStatus);
    DebugSerial.print(" home91=");
    DebugSerial.print(motor.homeStatus);
    DebugSerial.print(" home3B=[");
    if (motor.homeStatusOk) {
      DebugSerial.print(motor.homeStatusSingleTurn);
      DebugSerial.print(",");
      DebugSerial.print(motor.homeStatusOrigin);
    } else {
      DebugSerial.print("?,?");
    }
    DebugSerial.print("]");
    DebugSerial.print(" last=");
    DebugSerial.print(motor.lastSeenMs);
    DebugSerial.print(" raw31=");
    if (motor.encoderOk) DebugSerial.print(motor.rawEncoder);
    else DebugSerial.print("?");
    DebugSerial.print(" raw35=");
    if (motor.rawEncoderOk) DebugSerial.println(motor.diagnosticRawEncoder);
    else DebugSerial.println("?");
  }
}

static void reportCommandResult(const char *label, bool ok) {
  DebugSerial.print(label);
  DebugSerial.println(ok ? " OK" : " ERROR");
}

void handleMachineCommand(String line) {
  String command = nextToken(line);
  command.toUpperCase();

  if (command == "HELP" || command == "?") {
    printHelp();
    return;
  }

  if (command == "HELLO") {
    DebugSerial.println("PALLETIZER_LINK_OK");
    return;
  }

  if (command == "STATUS") {
    printStatus();
    return;
  }

  if (command == "FAULT") {
    String subCommand = nextToken(line);
    subCommand.toUpperCase();
    if (subCommand == "STATUS" || subCommand.length() == 0) {
      DebugSerial.print("SAFETY ");
      DebugSerial.print(safetyFaultActive ? "FAULT " : "OK ");
      DebugSerial.println(safetyFaultReason);
      return;
    }
    if (subCommand == "RESET") {
      reportCommandResult("FAULT RESET", resetSafetyFault());
      return;
    }
    if (subCommand == "TEST") {
      setSafetyFault("test manual de seguridad del eje X doble");
      reportCommandResult("FAULT TEST", safetyFaultActive);
      return;
    }
    DebugSerial.println("FAULT ERROR: usa FAULT STATUS, FAULT RESET o FAULT TEST.");
    return;
  }

  if (command == "PING") {
    bool ok = true;
    for (const MotorNode &motor : motors) {
      ok &= requestEncoder(motor);
      ok &= requestSpeed(motor);
      ok &= requestRawEncoder(motor);
      ok &= requestAngleError(motor);
      ok &= requestEnableStatus(motor);
      ok &= requestHomeStatus(motor);
      ok &= requestStallStatus(motor);
      delay(5);
    }
    reportCommandResult("PING", ok);
    return;
  }

  if (command == "SYNC") {
    const bool enable = nextToken(line).toInt() != 0;
    reportCommandResult("SYNC", commandEnableSyncBroadcast(enable));
    return;
  }

  if (command == "TRIGGER") {
    reportCommandResult("TRIGGER", commandSyncTrigger());
    return;
  }

  if (command == "STOP") {
    xPairMotionMonitoringActive = false;
    xPairExpectedDirection = 0;
    reportCommandResult("STOP", commandStopAll());
    return;
  }

  if (command == "VEL") {
    Axis axis = parseAxis(nextToken(line));
    int32_t rpm = nextToken(line).toInt();
    uint8_t acc = line.length() ? clampAcc(nextToken(line).toInt()) : DEFAULT_ACC;
    if (!axisIsSafeForMotion(axis) || axisLimitsAreConfigured(axis) || homing.active) {
      reportCommandResult("VEL", false);
      return;
    }

    if (axis == Axis::X) setXPairExpectedDirectionFromSpeed(rpm);
    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [rpm, acc](MotorNode &motor) {
      return commandSpeed(motor, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
    if (!ok) clearXPairExpectedDirectionIfAxis(axis);
    reportCommandResult("VEL", ok);
    return;
  }

  if (command == "VELANG") {
    Axis axis = parseAxis(nextToken(line));
    int32_t rpm = nextToken(line).toInt();
    uint8_t acc = line.length() ? angularAccelRpmSToMksAcc(nextToken(line).toFloat()) : DEFAULT_ACC;
    if (!axisIsSafeForMotion(axis) || axisLimitsAreConfigured(axis) || homing.active) {
      reportCommandResult("VELANG", false);
      return;
    }

    if (axis == Axis::X) setXPairExpectedDirectionFromSpeed(rpm);
    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [rpm, acc](MotorNode &motor) {
      return commandSpeed(motor, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
    if (!ok) clearXPairExpectedDirectionIfAxis(axis);
    reportCommandResult("VELANG", ok);
    return;
  }

  if (command == "POS") {
    Axis axis = parseAxis(nextToken(line));
    int32_t target = nextToken(line).toInt();
    uint16_t rpm = line.length() ? clampRpm(nextToken(line).toInt()) : DEFAULT_RPM;
    uint8_t acc = line.length() ? clampAcc(nextToken(line).toInt()) : DEFAULT_ACC;
    if (!axisIsSafeForMotion(axis)) {
      reportCommandResult("POS", false);
      return;
    }

    if (axis == Axis::X) setXPairExpectedDirectionFromTarget(target);
    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [target, rpm, acc](MotorNode &motor) {
      return commandAbsolutePosition(motor, target, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
    if (!ok) clearXPairExpectedDirectionIfAxis(axis);
    reportCommandResult("POS", ok);
    return;
  }

  if (command == "POSANG") {
    Axis axis = parseAxis(nextToken(line));
    int32_t target = nextToken(line).toInt();
    uint16_t rpm = line.length() ? clampRpm(nextToken(line).toInt()) : DEFAULT_RPM;
    uint8_t acc = line.length() ? angularAccelRpmSToMksAcc(nextToken(line).toFloat()) : DEFAULT_ACC;
    if (!axisIsSafeForMotion(axis)) {
      reportCommandResult("POSANG", false);
      return;
    }

    if (axis == Axis::X) setXPairExpectedDirectionFromTarget(target);
    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [target, rpm, acc](MotorNode &motor) {
      return commandAbsolutePosition(motor, target, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
    if (!ok) clearXPairExpectedDirectionIfAxis(axis);
    reportCommandResult("POSANG", ok);
    return;
  }

  if (command == "POSMM") {
    Axis axis = parseAxis(nextToken(line));
    const float targetMm = nextToken(line).toFloat();
    int32_t target = mmToEncoderCounts(targetMm);
    uint16_t rpm = line.length() ? clampRpm(nextToken(line).toInt()) : DEFAULT_RPM;
    uint8_t acc = line.length() ? clampAcc(nextToken(line).toInt()) : DEFAULT_ACC;
    if (!axisIsSafeForMotion(axis) || !targetWithinSoftwareLimits(axis, targetMm) || homing.active) {
      reportCommandResult("POSMM", false);
      return;
    }

    if (axis == Axis::X) setXPairExpectedDirectionFromTarget(target);
    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [target, rpm, acc](MotorNode &motor) {
      return commandAbsolutePosition(motor, target, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
    if (!ok) clearXPairExpectedDirectionIfAxis(axis);
    reportCommandResult("POSMM", ok);
    return;
  }

  if (command == "POSLINE") {
    Axis axis = parseAxis(nextToken(line));
    const float targetMm = nextToken(line).toFloat();
    int32_t target = mmToEncoderCounts(targetMm);
    uint16_t rpm = line.length() ? linearSpeedMmSToRpm(nextToken(line).toFloat()) : DEFAULT_RPM;
    uint8_t acc = line.length() ? linearAccelMmS2ToMksAcc(nextToken(line).toFloat()) : DEFAULT_ACC;
    if (!axisIsSafeForMotion(axis) || !targetWithinSoftwareLimits(axis, targetMm) || homing.active) {
      reportCommandResult("POSLINE", false);
      return;
    }

    if (axis == Axis::X) setXPairExpectedDirectionFromTarget(target);
    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [target, rpm, acc](MotorNode &motor) {
      return commandAbsolutePosition(motor, target, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
    if (!ok) clearXPairExpectedDirectionIfAxis(axis);
    reportCommandResult("POSLINE", ok);
    return;
  }

  if (command == "POSXYZ") {
    const float xMm = nextToken(line).toFloat();
    const float yMm = nextToken(line).toFloat();
    const float zMm = nextToken(line).toFloat();
    const float speedMmS = line.length() ? nextToken(line).toFloat() : 0.0f;
    const float accelMmS2 = line.length() ? nextToken(line).toFloat() : 0.0f;
    const bool ok = commandMoveXYZMm(xMm, yMm, zMm, speedMmS, accelMmS2);
    reportCommandResult("POSXYZ", ok);
    return;
  }

  if (command == "HOME") {
    Axis axis = parseAxis(nextToken(line));
    bool configureLimits = false;
    float minMm = 0.0f;
    float maxMm = 0.0f;
    uint16_t fastRpm = HOME_FAST_RPM;
    uint16_t slowRpm = HOME_SLOW_RPM;
    if (line.length()) {
      minMm = nextToken(line).toFloat();
      if (line.length()) {
        maxMm = nextToken(line).toFloat();
        configureLimits = true;
      }
    }
    if (line.length()) fastRpm = clampRpm(nextToken(line).toInt());
    if (line.length()) slowRpm = clampRpm(nextToken(line).toInt());

    const bool ok = startHomeAxis(axis, configureLimits, minMm, maxMm, fastRpm, slowRpm);
    reportCommandResult("HOME", ok);
    return;
  }

  if (command == "ORIGIN") {
    Axis axis = parseAxis(nextToken(line));
    const bool ok = commandGoOriginAxis(axis);
    reportCommandResult("ORIGIN", ok);
    return;
  }

  if (command == "ZERO") {
    Axis axis = parseAxis(nextToken(line));
    const float minMm = nextToken(line).toFloat();
    const float maxMm = nextToken(line).toFloat();
    const bool ok = commandSetZeroAxis(axis, true, minMm, maxMm);
    reportCommandResult("ZERO", ok);
    return;
  }

  if (command == "ENABLE") {
    Axis axis = parseAxis(nextToken(line));
    const bool ok = commandSetAxisEnable(axis, true);
    reportCommandResult("ENABLE", ok);
    return;
  }

  if (command == "DISABLE") {
    Axis axis = parseAxis(nextToken(line));
    const bool ok = commandSetAxisEnable(axis, false);
    reportCommandResult("DISABLE", ok);
    return;
  }

  DebugSerial.print("Comando desconocido: ");
  DebugSerial.println(command);
  printHelp();
}

bool beginMachine() {
  if (!mks::begin()) return false;

  commandSetBusFocModeBroadcast();
  delay(20);
  commandEnableSyncBroadcast(true);
  delay(20);

  for (MotorNode &motor : motors) {
    commandSetBusFocMode(motor);
    delay(20);
    commandEmergencyStop(motor);
    delay(20);
    commandSetEnable(motor, true);
    delay(20);
    requestEnableStatus(motor);
    requestStallStatus(motor);
    requestEncoder(motor);
    delay(5);
  }

  return true;
}
