#include "mks_can.h"

#include "config.h"

namespace mks {

uint8_t checksum(uint16_t canId, const uint8_t *data, uint8_t len) {
  uint16_t sum = canId;
  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF;
}

bool begin() {
  twai_general_config_t gConfig = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  gConfig.tx_queue_len = 16;
  gConfig.rx_queue_len = 32;
  twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&gConfig, &tConfig, &fConfig) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  return true;
}

bool sendFrame(uint16_t canId, const uint8_t *payloadWithoutCrc, uint8_t lenWithoutCrc, uint32_t timeoutMs) {
  if (lenWithoutCrc > 7) return false;

  twai_message_t msg = {};
  msg.identifier = canId;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = lenWithoutCrc + 1;

  for (uint8_t i = 0; i < lenWithoutCrc; i++) {
    msg.data[i] = payloadWithoutCrc[i];
  }
  msg.data[lenWithoutCrc] = checksum(canId, payloadWithoutCrc, lenWithoutCrc);

  return twai_transmit(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

bool readAnyFrame(twai_message_t &rx, uint32_t timeoutMs) {
  return twai_receive(&rx, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

bool verifyChecksum(const twai_message_t &msg) {
  if (msg.data_length_code == 0) return false;
  const uint8_t payloadLen = msg.data_length_code - 1;
  return msg.data[payloadLen] == checksum(msg.identifier, msg.data, payloadLen);
}

bool waitTxIdle(uint32_t timeoutMs) {
  const uint32_t startMs = millis();
  twai_status_info_t status = {};
  do {
    if (twai_get_status_info(&status) != ESP_OK) return false;
    if (status.msgs_to_tx == 0) return true;
    delay(1);
  } while (millis() - startMs <= timeoutMs);
  return false;
}

}  // namespace mks
