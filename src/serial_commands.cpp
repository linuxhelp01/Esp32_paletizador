#include "serial_commands.h"

#include <Arduino.h>

#include "machine.h"

static String serialLine;

void handleSerialInput() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      serialLine.trim();
      if (serialLine.length()) handleMachineCommand(serialLine);
      serialLine = "";
      continue;
    }
    if (serialLine.length() < 120) serialLine += c;
  }
}
