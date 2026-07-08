#include "machine.h"

#include "config.h"
#include "mks_can.h"

MotorNode motors[5] = {
  MotorNode("X1", CAN_ID_PHYSICAL_Y1, MOTOR_DIR_PHYSICAL_Y1),
  MotorNode("X2", CAN_ID_PHYSICAL_Y2, MOTOR_DIR_PHYSICAL_Y2),
  MotorNode("Y", CAN_ID_PHYSICAL_X, MOTOR_DIR_PHYSICAL_X),
  MotorNode("Z", CAN_ID_Z, MOTOR_DIR_Z),
  MotorNode("A", CAN_ID_A, MOTOR_DIR_A, true),
};

static uint32_t lastEncoderPollMs = 0;
static uint32_t lastSpeedPollMs = 0;
static uint32_t encoderPollingPausedUntilMs = 0;
static bool safetyFaultActive = false;
static const char *safetyFaultReason = "OK";
static bool xPairMotionMonitoringActive = false;
static int8_t xPairExpectedDirection = 0;

static bool commandStopAll();

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

static int8_t signOfDelta(int64_t value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

static void setSafetyFault(const char *reason) {
  if (!safetyFaultActive) {
    Serial.print("SAFETY FAULT: ");
    Serial.println(reason);
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
    Serial.print("X ERROR: muestras X1/X2 no sincronizadas, skewMs=");
    Serial.println(skewMs);
  }
  return false;
}

static bool isXPairAligned(bool requireFreshSamples = false) {
  const MotorNode &x1 = motors[0];
  const MotorNode &x2 = motors[1];
  if (!x1.encoderOk || !x2.encoderOk) {
    Serial.println("X ERROR: falta lectura de encoder en X1 o X2");
    commandStopAll();
    return false;
  }

  if (requireFreshSamples && !xPairSamplesCloseEnough(true)) {
    commandStopAll();
    return false;
  }

  const int64_t error = x1.encoder - x2.encoder;
  if (llabs(error) > X_PAIR_MAX_ERROR_COUNTS) {
    Serial.print("X ERROR: X1/X2 desalineados, errorEnc=");
    Serial.print(error);
    Serial.print(" errorMm=");
    Serial.println(encoderCountsToMm(error), 3);
    commandStopAll();
    return false;
  }

  return true;
}

static bool safetyAllowsMotion() {
  if (!safetyFaultActive) return true;
  Serial.print("SAFETY ERROR: falla activa, ejecute FAULT RESET. motivo=");
  Serial.println(safetyFaultReason);
  commandStopAll();
  return false;
}

static bool axisIsSafeForMotion(Axis axis) {
  if (!safetyAllowsMotion()) return false;
  if (axis == Axis::X1 || axis == Axis::X2) {
    Serial.println("X ERROR: X1 y X2 no se mueven por separado. Usa eje X.");
    return false;
  }
  if (axis == Axis::X) return isXPairAligned();
  return axis != Axis::UNKNOWN;
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
  Serial.println("SAFETY RESET OK: falla liberada, funcionamiento normal habilitado.");
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
  if (token == "A") return Axis::A;
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

int32_t degreesToEncoderCounts(float degrees) {
  const float counts = degrees * static_cast<float>(ENCODER_COUNTS_PER_REV) / 360.0f;
  if (counts > MAX_INT24) return MAX_INT24;
  if (counts < MIN_INT24) return MIN_INT24;
  return static_cast<int32_t>(roundf(counts));
}

float encoderCountsToDegrees(int64_t encoderCounts) {
  return static_cast<float>(encoderCounts) * 360.0f / static_cast<float>(ENCODER_COUNTS_PER_REV);
}

uint16_t linearSpeedMmSToRpm(float speedMmS) {
  const float rpm = abs(speedMmS) * 60.0f / LEADSCREW_MM_PER_REV;
  return clampRpm(static_cast<int32_t>(roundf(rpm)));
}

uint16_t angularSpeedDegSToRpm(float speedDegS) {
  return clampRpm(static_cast<int32_t>(roundf(abs(speedDegS) / 6.0f)));
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

static bool commandHome(MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_HOME, 0x00};
  return sendDriverCommand(motor.canId, cmd, sizeof(cmd));
}

static bool commandSetZero(MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_SET_ZERO};
  return sendDriverCommand(motor.canId, cmd, sizeof(cmd));
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

static bool requestEncoder(const MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_READ_ENCODER};
  return mks::sendFrame(motor.canId, cmd, sizeof(cmd));
}

static bool requestSpeed(const MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_READ_SPEED_RPM};
  return mks::sendFrame(motor.canId, cmd, sizeof(cmd));
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

  motor.rawEncoder = value;
  motor.previousEncoder = motor.encoder;
  motor.encoder = value * motor.direction;
  motor.encoderOk = true;
  motor.lastEncoderUpdateMs = millis();
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

static void handleStatusReply(MotorNode &motor, const twai_message_t &msg) {
  if (msg.data_length_code < 3) return;
  const uint8_t code = msg.data[0];
  const uint8_t status = msg.data[1];

  if (code == mks::CMD_HOME) {
    motor.homeStatus = status;
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
      case mks::CMD_RUN_ABS_COORD:
      case mks::CMD_RUN_SPEED:
      case mks::CMD_EMERGENCY_STOP:
      case mks::CMD_HOME:
      case mks::CMD_SET_ZERO:
      case mks::CMD_SET_MODE:
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
    for (const MotorNode &motor : motors) {
      requestEncoder(motor);
    }
  }

  if (now - lastSpeedPollMs >= SPEED_POLL_MS) {
    lastSpeedPollMs = now;
    for (const MotorNode &motor : motors) {
      requestSpeed(motor);
    }
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Comandos:");
  Serial.println("  HELLO");
  Serial.println("  POS <X|Y|Z|A> <encoder_abs> [rpm] [acc]");
  Serial.println("      Posicion angular absoluta del motor en cuentas de encoder.");
  Serial.println("  POSANG <X|Y|Z|A> <encoder_abs> [rpm] [rpm_s]");
  Serial.println("      Posicion angular. En X mueve X1/X2 juntos.");
  Serial.println("  POSA <deg_abs> [deg_s] [deg_s2]");
  Serial.println("      Posicion absoluta del eje rotatorio A, sin limite de zona de trabajo.");
  Serial.println("  POSMM <X|Y|Z> <mm_abs> [rpm] [acc]");
  Serial.println("      Posicion lineal absoluta del extremo del robot en milimetros.");
  Serial.println("  POSLINE <X|Y|Z> <mm_abs> [mm_s] [mm_s2]");
  Serial.println("      Eje del robot: posicion, velocidad y aceleracion lineal.");
  Serial.println("  POSXYZ <x_mm> <y_mm> <z_mm> [mm_s] [mm_s2]");
  Serial.println("      Movimiento lineal coordinado XYZ con disparo broadcast 4Bh.");
  Serial.println("  SYNC <0|1>         (configura marca sincronizada MKS 4Ah por broadcast)");
  Serial.println("  TRIGGER            (disparo sincronizado MKS 4Bh por broadcast)");
  Serial.println("  VEL <X|Y|Z|A> <rpm> [acc]");
  Serial.println("  VELANG <X|Y|Z|A> <rpm> [rpm_s]");
  Serial.println("  VELA <deg_s> [deg_s2]");
  Serial.println("  HOME X            (homing simultaneo X1/X2)");
  Serial.println("  HOME <X|Y|Z|A>");
  Serial.println("  ZERO <X|Y|Z|A>");
  Serial.println("  PING              (fuerza consulta CAN a todos los motores)");
  Serial.println("  STOP");
  Serial.println("  FAULT STATUS      (muestra el estado de seguridad)");
  Serial.println("  FAULT RESET       (libera falla si X1/X2 tienen encoder valido y estan alineados)");
  Serial.println("  FAULT TEST        (simula falla del eje X doble para probar enclavamiento)");
  Serial.println("  STATUS");
  Serial.println();
}

void printStatus() {
  Serial.println("STATUS");
  Serial.print("  safetyFault=");
  Serial.print(safetyFaultActive ? 1 : 0);
  Serial.print(" reason=");
  Serial.println(safetyFaultReason);
  const uint32_t now = millis();
  for (const MotorNode &motor : motors) {
    const bool online = motor.lastSeenMs > 0 && (now - motor.lastSeenMs) <= MOTOR_ONLINE_TIMEOUT_MS;
    Serial.print("  ");
    Serial.print(motor.name);
    Serial.print(" id=0x");
    Serial.print(motor.canId, HEX);
    Serial.print(" online=");
    Serial.print(online ? 1 : 0);
    Serial.print(" angularEnc=");
    if (motor.encoderOk) Serial.print(motor.encoder);
    else Serial.print("?");
    Serial.print(" angleDeg=");
    if (motor.encoderOk) Serial.print(encoderCountsToDegrees(motor.encoder), 3);
    else Serial.print("?");
    Serial.print(" linearMm=");
    if (motor.encoderOk && !motor.rotaryAxis) Serial.print(encoderCountsToMm(motor.encoder), 3);
    else Serial.print("?");
    Serial.print(" rpm=");
    if (motor.rpmOk) Serial.print(motor.rpm);
    else Serial.print("?");
    Serial.print(" acc=");
    Serial.print(motor.lastAcc);
    Serial.print(" moveStatus=");
    Serial.print(motor.moveStatus);
    Serial.print(" homeStatus=");
    Serial.print(motor.homeStatus);
    Serial.print(" last=");
    Serial.print(motor.lastSeenMs);
    Serial.print(" rawEnc=");
    if (motor.encoderOk) Serial.println(motor.rawEncoder);
    else Serial.println("?");
  }
}

static void reportCommandResult(const char *label, bool ok) {
  Serial.print(label);
  Serial.println(ok ? " OK" : " ERROR");
}

void handleMachineCommand(String line) {
  String command = nextToken(line);
  command.toUpperCase();

  if (command == "HELP" || command == "?") {
    printHelp();
    return;
  }

  if (command == "HELLO") {
    Serial.println("PALLETIZER_LINK_OK");
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
      Serial.print("SAFETY ");
      Serial.print(safetyFaultActive ? "FAULT " : "OK ");
      Serial.println(safetyFaultReason);
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
    Serial.println("FAULT ERROR: usa FAULT STATUS, FAULT RESET o FAULT TEST.");
    return;
  }

  if (command == "PING") {
    bool ok = true;
    for (const MotorNode &motor : motors) {
      ok &= requestEncoder(motor);
      ok &= requestSpeed(motor);
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
    if (!axisIsSafeForMotion(axis)) {
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
    if (!axisIsSafeForMotion(axis)) {
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

  if (command == "VELA") {
    const float speedDegS = nextToken(line).toFloat();
    const float accelDegS2 = line.length() ? nextToken(line).toFloat() : 6000.0f;
    if (!axisIsSafeForMotion(Axis::A)) {
      reportCommandResult("VELA", false);
      return;
    }

    int32_t rpm = angularSpeedDegSToRpm(speedDegS);
    if (speedDegS < 0.0f) rpm = -rpm;
    const uint8_t acc = angularAccelRpmSToMksAcc(accelDegS2 / 6.0f);
    bool ok = commandPrepareSynchronizedMove();
    ok &= commandSpeed(motors[4], rpm, acc);
    if (ok) ok &= commandSyncTrigger();
    reportCommandResult("VELA", ok);
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

  if (command == "POSA") {
    const int32_t target = degreesToEncoderCounts(nextToken(line).toFloat());
    const uint16_t rpm = line.length() ? angularSpeedDegSToRpm(nextToken(line).toFloat()) : DEFAULT_RPM;
    const float accelDegS2 = line.length() ? nextToken(line).toFloat() : 6000.0f;
    const uint8_t acc = angularAccelRpmSToMksAcc(accelDegS2 / 6.0f);
    if (!axisIsSafeForMotion(Axis::A)) {
      reportCommandResult("POSA", false);
      return;
    }

    bool ok = commandPrepareSynchronizedMove();
    ok &= commandAbsolutePosition(motors[4], target, rpm, acc);
    if (ok) ok &= commandSyncTrigger();
    reportCommandResult("POSA", ok);
    return;
  }

  if (command == "POSMM") {
    Axis axis = parseAxis(nextToken(line));
    int32_t target = mmToEncoderCounts(nextToken(line).toFloat());
    uint16_t rpm = line.length() ? clampRpm(nextToken(line).toInt()) : DEFAULT_RPM;
    uint8_t acc = line.length() ? clampAcc(nextToken(line).toInt()) : DEFAULT_ACC;
    if (!axisIsSafeForMotion(axis)) {
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
    int32_t target = mmToEncoderCounts(nextToken(line).toFloat());
    uint16_t rpm = line.length() ? linearSpeedMmSToRpm(nextToken(line).toFloat()) : DEFAULT_RPM;
    uint8_t acc = line.length() ? linearAccelMmS2ToMksAcc(nextToken(line).toFloat()) : DEFAULT_ACC;
    if (!axisIsSafeForMotion(axis)) {
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
    const int32_t xTarget = mmToEncoderCounts(nextToken(line).toFloat());
    const int32_t yTarget = mmToEncoderCounts(nextToken(line).toFloat());
    const int32_t zTarget = mmToEncoderCounts(nextToken(line).toFloat());
    uint16_t rpm = line.length() ? linearSpeedMmSToRpm(nextToken(line).toFloat()) : DEFAULT_RPM;
    uint8_t acc = line.length() ? linearAccelMmS2ToMksAcc(nextToken(line).toFloat()) : DEFAULT_ACC;
    if (!safetyAllowsMotion() || !isXPairAligned()) {
      reportCommandResult("POSXYZ", false);
      return;
    }

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
    reportCommandResult("POSXYZ", ok);
    return;
  }

  if (command == "HOME") {
    Axis axis = parseAxis(nextToken(line));
    if (!safetyAllowsMotion()) {
      reportCommandResult("HOME", false);
      return;
    }
    if (axis == Axis::X1 || axis == Axis::X2 || axis == Axis::UNKNOWN) {
      Serial.println("X ERROR: usa HOME X para referenciar X1/X2 juntos.");
      reportCommandResult("HOME", false);
      return;
    }
    if (axis == Axis::X) {
      xPairMotionMonitoringActive = true;
      xPairExpectedDirection = 0;
    }
    bool ok = forEachMotorInAxis(axis, [](MotorNode &motor) {
      return commandHome(motor);
    });
    if (!ok) clearXPairExpectedDirectionIfAxis(axis);
    reportCommandResult("HOME", ok);
    return;
  }

  if (command == "ZERO") {
    Axis axis = parseAxis(nextToken(line));
    if (!safetyAllowsMotion()) {
      reportCommandResult("ZERO", false);
      return;
    }
    if (axis == Axis::X1 || axis == Axis::X2 || axis == Axis::UNKNOWN) {
      Serial.println("X ERROR: usa ZERO X para ajustar cero en X1/X2 juntos.");
      reportCommandResult("ZERO", false);
      return;
    }
    bool ok = forEachMotorInAxis(axis, [](MotorNode &motor) {
      return commandSetZero(motor);
    });
    clearXPairExpectedDirectionIfAxis(axis);
    reportCommandResult("ZERO", ok);
    return;
  }

  Serial.print("Comando desconocido: ");
  Serial.println(command);
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
  }

  return true;
}
