#include <Arduino.h>
#include "config.h"
#include "machine.h"
#include "ros_bridge.h"
#include "serial_commands.h"

// =====================================================
// ESP32-S3 + TWAI/CAN para paletizador cartesiano
//
// La funcionalidad esta separada en:
//   config.h             pines, IDs y limites
//   mks_can.*            driver TWAI y checksum MKS
//   machine.*            ejes, motores, comandos y telemetria
//   serial_commands.*    lectura de comandos desde USB/Serial
// =====================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

#if !ENABLE_MICRO_ROS
  Serial.println();
  Serial.println("ESP32-S3 paletizador cartesiano CAN/TWAI");
#endif

  if (!beginMachine()) {
#if !ENABLE_MICRO_ROS
    Serial.println("ERROR iniciando CAN/TWAI");
#endif
    return;
  }

#if !ENABLE_MICRO_ROS
  Serial.println("CAN/TWAI iniciado a 1 Mbit/s");
#endif

#if ENABLE_MICRO_ROS
  beginRosBridge();
#else
  printHelp();
#endif
}

void loop() {
#if !ENABLE_MICRO_ROS
  handleSerialInput();
#else
  static uint32_t lastRosConnectAttemptMs = 0;
  const uint32_t nowMs = millis();
  if (!rosBridgeReady() && nowMs - lastRosConnectAttemptMs >= 1000) {
    lastRosConnectAttemptMs = nowMs;
    beginRosBridge();
  }
#endif
  pollEncoders();
  drainCanReplies();
  spinRosBridge();
}
