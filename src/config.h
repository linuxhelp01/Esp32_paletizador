#pragma once

#include <Arduino.h>
#include "driver/twai.h"

// Pines ESP32-S3 hacia transceiver CAN, por ejemplo SN65HVD230.
static constexpr gpio_num_t CAN_TX = GPIO_NUM_5;
static constexpr gpio_num_t CAN_RX = GPIO_NUM_4;

// Ajusta estos ID segun la direccion CAN configurada en cada driver.
// Los nombres PHYSICAL_* documentan el cableado original antes del cambio
// de nomenclatura: el eje doble antiguo Y ahora se opera como X1/X2, y el
// motor antiguo X ahora se opera como Y.
static constexpr uint16_t CAN_ID_PHYSICAL_X = 0x01;
static constexpr uint16_t CAN_ID_PHYSICAL_Y1 = 0x02;
static constexpr uint16_t CAN_ID_PHYSICAL_Y2 = 0x03;
static constexpr uint16_t CAN_ID_Z = 0x04;

static constexpr uint16_t MAX_RPM = 3000;
static constexpr uint8_t MAX_ACC = 255;
static constexpr uint8_t DEFAULT_ACC = 10;
static constexpr uint16_t DEFAULT_RPM = 500;
// Muestreo: se consulta cada encoder cada 10 ms, equivalente a 100 Hz por motor.
// La velocidad RPM se consulta menos seguido para dejar prioridad a movimiento.
static constexpr uint32_t ENCODER_POLL_MS = 10;
static constexpr uint32_t SPEED_POLL_MS = 100;
static constexpr uint32_t MOTION_COMMAND_POLL_PAUSE_MS = 25;
static constexpr uint8_t CAN_COMMAND_TX_RETRIES = 3;
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
// Se invirtio el sentido positivo de los ejes fisicos X e Y: la correccion se
// aplica tanto al comando de movimiento como a la lectura de encoder/rpm.
static constexpr int8_t MOTOR_DIR_PHYSICAL_X = -1;
static constexpr int8_t MOTOR_DIR_PHYSICAL_Y1 = -1;
static constexpr int8_t MOTOR_DIR_PHYSICAL_Y2 = -1;
static constexpr int8_t MOTOR_DIR_Z = 1;

// Diferencia maxima permitida entre X1 e X2, en cuentas de encoder corregidas.
// Con 2048 cuentas/mm, 4096 cuentas equivalen a 2.0 mm.
static constexpr int32_t X_PAIR_MAX_ERROR_COUNTS = 4096;
static constexpr uint32_t X_PAIR_ALIGNMENT_SAMPLE_SKEW_MS = 20;

// Verificacion dinamica del eje doble: si al moverse los motores avanzan en
// sentidos opuestos, o avanzan contra el sentido esperado, se detiene todo y
// queda enclavado un estado de falla hasta ejecutar FAULT RESET.
static constexpr int32_t X_PAIR_DIRECTION_MIN_DELTA_COUNTS = 256;

static constexpr int32_t MIN_INT24 = -8388607;
static constexpr int32_t MAX_INT24 = 8388607;
