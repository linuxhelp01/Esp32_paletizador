#pragma once

#include <Arduino.h>
#include "driver/twai.h"

// Pines ESP32-S3 hacia transceiver CAN, por ejemplo SN65HVD230.
static constexpr gpio_num_t CAN_TX = GPIO_NUM_5;
static constexpr gpio_num_t CAN_RX = GPIO_NUM_4;

// Ajusta estos ID segun la direccion CAN configurada en cada driver.
static constexpr uint16_t CAN_ID_X = 0x01;
static constexpr uint16_t CAN_ID_Y1 = 0x02;
static constexpr uint16_t CAN_ID_Y2 = 0x03;
static constexpr uint16_t CAN_ID_Z = 0x04;

static constexpr uint16_t MAX_RPM = 3000;
static constexpr uint8_t MAX_ACC = 255;
static constexpr uint8_t DEFAULT_ACC = 10;
static constexpr uint16_t DEFAULT_RPM = 500;
static constexpr uint32_t ENCODER_POLL_MS = 100;
static constexpr uint32_t MOTOR_ONLINE_TIMEOUT_MS = 1500;
static constexpr uint32_t SERIAL_BAUD = 115200;

// Husillo: una vuelta completa del motor desplaza linealmente 8 mm.
// El encoder MKS reporta 16384 cuentas por vuelta.
static constexpr int32_t ENCODER_COUNTS_PER_REV = 16384;
static constexpr float LEADSCREW_MM_PER_REV = 8.0f;
static constexpr float ENCODER_COUNTS_PER_MM = ENCODER_COUNTS_PER_REV / LEADSCREW_MM_PER_REV;

// Sentido logico de cada motor respecto al eje del robot.
// Si al ordenar un movimiento positivo el encoder corregido baja en vez de subir,
// cambia el signo de ese motor de 1 a -1, o de -1 a 1.
static constexpr int8_t MOTOR_DIR_X = 1;
static constexpr int8_t MOTOR_DIR_Y1 = 1;
static constexpr int8_t MOTOR_DIR_Y2 = 1;
static constexpr int8_t MOTOR_DIR_Z = 1;

// Diferencia maxima permitida entre Y1 e Y2, en cuentas de encoder corregidas.
// Con 2048 cuentas/mm, 1024 cuentas equivalen a 0.5 mm.
static constexpr int32_t Y_PAIR_MAX_ERROR_COUNTS = 1024;

static constexpr int32_t MIN_INT24 = -8388607;
static constexpr int32_t MAX_INT24 = 8388607;
