#include <Arduino.h>

#include "config.h"
#include "machine.h"
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

  Serial.println();
  Serial.println("ESP32-S3 paletizador cartesiano CAN/TWAI");

  if (!beginMachine()) {
    Serial.println("ERROR iniciando CAN/TWAI");
    return;
  }

  Serial.println("CAN/TWAI iniciado a 500 kbit/s");
  printHelp();
}

void loop() {
  handleSerialInput();
  pollEncoders();
  drainCanReplies();
}
