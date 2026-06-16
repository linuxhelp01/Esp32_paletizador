#pragma once

#include <Arduino.h>
#include "driver/twai.h"

namespace mks {

static constexpr uint8_t CMD_READ_ENCODER = 0x31;
static constexpr uint8_t CMD_READ_SPEED_RPM = 0x32;
static constexpr uint8_t CMD_SET_MODE = 0x82;
static constexpr uint8_t CMD_SET_ZERO = 0x92;
static constexpr uint8_t CMD_RUN_SPEED = 0xF6;
static constexpr uint8_t CMD_RUN_ABS_COORD = 0xF5;
static constexpr uint8_t CMD_EMERGENCY_STOP = 0xF7;
static constexpr uint8_t CMD_HOME = 0x91;
static constexpr uint8_t CMD_SYNC_ENABLE = 0x4A;
static constexpr uint8_t CMD_SYNC_TRIGGER = 0x4B;

static constexpr uint16_t CAN_ID_BROADCAST = 0x00;

static constexpr uint8_t DIR_POSITIVE = 0x00;
static constexpr uint8_t DIR_NEGATIVE = 0x80;

uint8_t checksum(uint16_t canId, const uint8_t *data, uint8_t len);
bool begin();
bool sendFrame(uint16_t canId, const uint8_t *payloadWithoutCrc, uint8_t lenWithoutCrc);
bool readAnyFrame(twai_message_t &rx, uint32_t timeoutMs = 0);
bool verifyChecksum(const twai_message_t &msg);
bool waitTxIdle(uint32_t timeoutMs);

}  // namespace mks
