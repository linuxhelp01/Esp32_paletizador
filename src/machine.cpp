#include "machine.h"

#include "config.h"
#include "mks_can.h"

MotorNode motors[4] = {
  MotorNode("X", CAN_ID_X, MOTOR_DIR_X),
  MotorNode("Y1", CAN_ID_Y1, MOTOR_DIR_Y1),
  MotorNode("Y2", CAN_ID_Y2, MOTOR_DIR_Y2),
  MotorNode("Z", CAN_ID_Z, MOTOR_DIR_Z),
};

static uint32_t lastEncoderPollMs = 0;
static uint8_t pollIndex = 0;

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

static bool isYPairAligned() {
  const MotorNode &y1 = motors[1];
  const MotorNode &y2 = motors[2];
  if (!y1.encoderOk || !y2.encoderOk) {
    Serial.println("Y ERROR: falta lectura de encoder en Y1 o Y2");
    commandStopAll();
    return false;
  }

  const int64_t error = y1.encoder - y2.encoder;
  if (llabs(error) > Y_PAIR_MAX_ERROR_COUNTS) {
    Serial.print("Y ERROR: Y1/Y2 desalineados, errorEnc=");
    Serial.print(error);
    Serial.print(" errorMm=");
    Serial.println(encoderCountsToMm(error), 3);
    commandStopAll();
    return false;
  }

  return true;
}

static bool axisIsSafeForMotion(Axis axis) {
  if (axis == Axis::Y1 || axis == Axis::Y2) {
    Serial.println("Y ERROR: Y1 y Y2 no se mueven por separado. Usa eje Y.");
    return false;
  }
  if (axis == Axis::Y) return isYPairAligned();
  return axis != Axis::UNKNOWN;
}

Axis parseAxis(String token) {
  token.trim();
  token.toUpperCase();
  if (token == "X") return Axis::X;
  if (token == "Y") return Axis::Y;
  if (token == "Y1") return Axis::Y1;
  if (token == "Y2") return Axis::Y2;
  if (token == "Z") return Axis::Z;
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
  uint8_t speedHigh = 0;
  uint8_t speedLow = 0;
  encodeSpeed(toMotorSpeed(motor, rpm), speedHigh, speedLow);
  const uint8_t cmd[] = {mks::CMD_RUN_SPEED, speedHigh, speedLow, acc};
  const bool ok = mks::sendFrame(motor.canId, cmd, sizeof(cmd));
  if (ok) motor.lastAcc = acc;
  return ok;
}

static bool commandEmergencyStop(MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_EMERGENCY_STOP};
  return mks::sendFrame(motor.canId, cmd, sizeof(cmd));
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
  return mks::sendFrame(motor.canId, cmd, sizeof(cmd));
}

static bool commandSetZero(MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_SET_ZERO};
  return mks::sendFrame(motor.canId, cmd, sizeof(cmd));
}

static bool commandSetBusFocMode(MotorNode &motor) {
  const uint8_t cmd[] = {mks::CMD_SET_MODE, 0x05};
  return mks::sendFrame(motor.canId, cmd, sizeof(cmd));
}

static bool commandSetBusFocModeBroadcast() {
  const uint8_t cmd[] = {mks::CMD_SET_MODE, 0x05};
  return mks::sendFrame(mks::CAN_ID_BROADCAST, cmd, sizeof(cmd));
}

static bool commandEnableSyncBroadcast(bool enable) {
  const uint8_t cmd[] = {mks::CMD_SYNC_ENABLE, static_cast<uint8_t>(enable ? 0x01 : 0x00)};
  return mks::sendFrame(mks::CAN_ID_BROADCAST, cmd, sizeof(cmd));
}

static bool commandSyncTrigger() {
  const uint8_t cmd[] = {mks::CMD_SYNC_TRIGGER};
  bool ok = true;
  for (uint8_t i = 0; i < 3; i++) {
    ok &= mks::sendFrame(mks::CAN_ID_BROADCAST, cmd, sizeof(cmd));
    delay(1);
  }
  return ok;
}

static bool commandPrepareSynchronizedMove() {
  const bool ok = commandEnableSyncBroadcast(true);
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

  const bool ok = mks::sendFrame(motor.canId, cmd, sizeof(cmd));
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
  motor.encoder = value * motor.direction;
  motor.encoderOk = true;
  motor.lastSeenMs = millis();
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
  if (millis() - lastEncoderPollMs < ENCODER_POLL_MS) return;
  lastEncoderPollMs = millis();

  MotorNode &motor = motors[pollIndex];
  requestEncoder(motor);
  requestSpeed(motor);

  pollIndex++;
  if (pollIndex >= (sizeof(motors) / sizeof(motors[0]))) pollIndex = 0;
}

void printHelp() {
  Serial.println();
  Serial.println("Comandos:");
  Serial.println("  HELLO");
  Serial.println("  POS <X|Y|Z> <encoder_abs> [rpm] [acc]");
  Serial.println("      Posicion angular absoluta del motor en cuentas de encoder.");
  Serial.println("  POSANG <X|Y|Z> <encoder_abs> [rpm] [rpm_s]");
  Serial.println("      Posicion angular. En Y mueve Y1/Y2 juntos.");
  Serial.println("  POSMM <X|Y|Z> <mm_abs> [rpm] [acc]");
  Serial.println("      Posicion lineal absoluta del extremo del robot en milimetros.");
  Serial.println("  POSLINE <X|Y|Z> <mm_abs> [mm_s] [mm_s2]");
  Serial.println("      Eje del robot: posicion, velocidad y aceleracion lineal.");
  Serial.println("  POSXYZ <x_mm> <y_mm> <z_mm> [mm_s] [mm_s2]");
  Serial.println("      Movimiento lineal coordinado XYZ con disparo broadcast 4Bh.");
  Serial.println("  SYNC <0|1>         (configura marca sincronizada MKS 4Ah por broadcast)");
  Serial.println("  TRIGGER            (disparo sincronizado MKS 4Bh por broadcast)");
  Serial.println("  VEL <X|Y|Z> <rpm> [acc]");
  Serial.println("  VELANG <X|Y|Z> <rpm> [rpm_s]");
  Serial.println("  HOME Y            (homing simultaneo Y1/Y2)");
  Serial.println("  HOME <X|Y|Z>");
  Serial.println("  ZERO <X|Y|Z>");
  Serial.println("  PING              (fuerza consulta CAN a todos los motores)");
  Serial.println("  STOP");
  Serial.println("  STATUS");
  Serial.println();
}

void printStatus() {
  Serial.println("STATUS");
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
    Serial.print(" linearMm=");
    if (motor.encoderOk) Serial.print(encoderCountsToMm(motor.encoder), 3);
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

    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [rpm, acc](MotorNode &motor) {
      return commandSpeed(motor, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
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

    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [rpm, acc](MotorNode &motor) {
      return commandSpeed(motor, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
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

    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [target, rpm, acc](MotorNode &motor) {
      return commandAbsolutePosition(motor, target, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
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

    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [target, rpm, acc](MotorNode &motor) {
      return commandAbsolutePosition(motor, target, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
    reportCommandResult("POSANG", ok);
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

    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [target, rpm, acc](MotorNode &motor) {
      return commandAbsolutePosition(motor, target, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
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

    bool ok = commandPrepareSynchronizedMove();
    ok &= forEachMotorInAxis(axis, [target, rpm, acc](MotorNode &motor) {
      return commandAbsolutePosition(motor, target, rpm, acc);
    });
    if (ok) ok &= commandSyncTrigger();
    reportCommandResult("POSLINE", ok);
    return;
  }

  if (command == "POSXYZ") {
    const int32_t xTarget = mmToEncoderCounts(nextToken(line).toFloat());
    const int32_t yTarget = mmToEncoderCounts(nextToken(line).toFloat());
    const int32_t zTarget = mmToEncoderCounts(nextToken(line).toFloat());
    uint16_t rpm = line.length() ? linearSpeedMmSToRpm(nextToken(line).toFloat()) : DEFAULT_RPM;
    uint8_t acc = line.length() ? linearAccelMmS2ToMksAcc(nextToken(line).toFloat()) : DEFAULT_ACC;
    if (!isYPairAligned()) {
      reportCommandResult("POSXYZ", false);
      return;
    }

    bool ok = commandPrepareSynchronizedMove();
    ok &= commandAbsolutePosition(motors[0], xTarget, rpm, acc);
    ok &= commandAbsolutePosition(motors[1], yTarget, rpm, acc);
    ok &= commandAbsolutePosition(motors[2], yTarget, rpm, acc);
    ok &= commandAbsolutePosition(motors[3], zTarget, rpm, acc);
    if (ok) ok &= commandSyncTrigger();
    reportCommandResult("POSXYZ", ok);
    return;
  }

  if (command == "HOME") {
    Axis axis = parseAxis(nextToken(line));
    if (axis == Axis::Y1 || axis == Axis::Y2 || axis == Axis::UNKNOWN) {
      Serial.println("Y ERROR: usa HOME Y para referenciar Y1/Y2 juntos.");
      reportCommandResult("HOME", false);
      return;
    }
    bool ok = forEachMotorInAxis(axis, [](MotorNode &motor) {
      return commandHome(motor);
    });
    reportCommandResult("HOME", ok);
    return;
  }

  if (command == "ZERO") {
    Axis axis = parseAxis(nextToken(line));
    if (axis == Axis::Y1 || axis == Axis::Y2 || axis == Axis::UNKNOWN) {
      Serial.println("Y ERROR: usa ZERO Y para ajustar cero en Y1/Y2 juntos.");
      reportCommandResult("ZERO", false);
      return;
    }
    bool ok = forEachMotorInAxis(axis, [](MotorNode &motor) {
      return commandSetZero(motor);
    });
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
