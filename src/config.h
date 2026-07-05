#pragma once

#include <Arduino.h>
#include "driver/twai.h"

#ifndef ENABLE_MICRO_ROS
#error "ENABLE_MICRO_ROS must be defined by platformio.ini"
#endif

#if !ENABLE_MICRO_ROS
#error "This firmware must be compiled with micro-ROS enabled"
#endif

// Dimensiones logicas del robot. X usa dos drivers fisicos: X1/X2.
static constexpr size_t ROBOT_MOTOR_COUNT = 5;
static constexpr size_t ROBOT_CARTESIAN_MOTOR_COUNT = 4;
static constexpr size_t ROBOT_LINEAR_AXIS_COUNT = 3;

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
static constexpr uint16_t CAN_ID_A = 0x05;  // Eje rotatorio MKS Servo42D.

static constexpr uint16_t MAX_RPM = 3000;
static constexpr uint8_t MAX_ACC = 255;
static constexpr uint8_t DEFAULT_ACC = 10;
static constexpr uint16_t DEFAULT_RPM = 500;
// Scheduler de posicion 0x31 por criticidad del eje. X1/X2 se leen como par
// critico para detectar desalineacion; Y/Z tienen frecuencia buena para
// correcciones <50 ms; A baja frecuencia en reposo y sube cuando se mueve.
static constexpr uint32_t ENCODER_X_PAIR_POLL_MS = 4;       // 250 Hz por motor.
static constexpr uint32_t ENCODER_LINEAR_POLL_MS = 5;       // 200 Hz por motor.
static constexpr uint32_t ENCODER_A_IDLE_POLL_MS = 20;      // 50 Hz.
static constexpr uint32_t ENCODER_A_ACTIVE_POLL_MS = 5;     // 200 Hz.
static constexpr uint8_t ENCODER_MAX_REQUESTS_PER_CONTROL_CYCLE = 2;
static constexpr uint8_t CAN_POLL_SLOT_COUNT = 20;          // Ventana determinista de 20 ms.
static constexpr uint32_t POSITION_SAMPLE_MAX_AGE_MS = 10;
static constexpr uint32_t SPEED_POLL_MS = 100;
static constexpr uint32_t RAW_ENCODER_DIAGNOSTIC_POLL_MS = 2000;
static constexpr uint32_t ANGLE_ERROR_POLL_MS = 50;
static constexpr uint32_t HOME_STATUS_POLL_MS = 20;
static constexpr uint32_t ENABLE_STATUS_POLL_MS = 500;
static constexpr uint32_t STALL_STATUS_POLL_MS = 500;
static constexpr uint32_t MOTION_COMMAND_POLL_PAUSE_MS = 25;
static constexpr uint8_t CAN_COMMAND_TX_RETRIES = 3;
static constexpr uint32_t CAN_TELEMETRY_TX_TIMEOUT_MS = 0;
static constexpr uint32_t MOTOR_ONLINE_TIMEOUT_MS = 1500;
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr size_t MICRO_ROS_DOMAIN_ID = 10;
static constexpr BaseType_t CONTROL_TASK_CORE = 1;
static constexpr BaseType_t ROS_TASK_CORE = 0;
static constexpr UBaseType_t CONTROL_TASK_PRIORITY = 4;
static constexpr UBaseType_t ROS_TASK_PRIORITY = 3;
static constexpr uint32_t CONTROL_TASK_STACK_BYTES = 8192;
static constexpr uint32_t ROS_TASK_STACK_BYTES = 16384;
static constexpr uint32_t CONTROL_TASK_PERIOD_MS = 1;
static constexpr uint32_t ROS_TASK_PERIOD_MS = 1;
static constexpr uint8_t ROBOT_COMMAND_QUEUE_LENGTH = 8;
static constexpr uint8_t ROBOT_MAX_COMMANDS_PER_CONTROL_CYCLE = 2;
static constexpr uint8_t CAN_MAX_REPLIES_PER_CONTROL_CYCLE = 8;
static constexpr uint32_t MICRO_ROS_RECONNECT_PERIOD_MS = 1000;
static constexpr uint32_t MICRO_ROS_HEALTH_CHECK_PERIOD_MS = 5000;
static constexpr int MICRO_ROS_PING_TIMEOUT_MS = 100;
static constexpr uint8_t MICRO_ROS_PING_ATTEMPTS = 3;
static constexpr int MICRO_ROS_ENTITY_CREATION_TIMEOUT_MS = 15000;
static constexpr int MICRO_ROS_ENTITY_DESTROY_TIMEOUT_MS = 15000;
static constexpr uint32_t ACTION_PROCESS_PERIOD_MS = 5;
static constexpr uint32_t ACTION_FEEDBACK_PERIOD_MS = 20;
static constexpr float ACTION_RESULT_MIN_TOLERANCE_MM = 1.0f;
static constexpr uint32_t ACTION_RESULT_POSITION_STABLE_MS = 50;
static constexpr uint32_t ROS_FAST_TELEMETRY_PERIOD_MS = 50;
static constexpr uint32_t ROS_STATUS_TELEMETRY_PERIOD_MS = 250;
static constexpr uint32_t ROS_SERVICE_ACCEPT_WAIT_MS = 5;

// Homing MKS 0x91/0x90. Los valores de trigger/direccion/limite dependen del
// cableado fisico de los finales de carrera. Los defaults replican el ejemplo
// del manual Makerbase para origin-switch homing.
static constexpr uint16_t HOME_FAST_RPM = 300;
static constexpr uint16_t HOME_SLOW_RPM = 80;
static constexpr uint8_t HOME_TRIGGER_LEVEL = 0;
static constexpr uint8_t HOME_LIMIT_ENABLE = 0;
static constexpr uint8_t HOME_MODE_ORIGIN_SWITCH = 0;
static constexpr uint32_t HOME_PHASE_TIMEOUT_MS = 60000;
static constexpr uint8_t HOME_DIRECTION_X1 = 0;
static constexpr uint8_t HOME_DIRECTION_X2 = 0;
static constexpr uint8_t HOME_DIRECTION_Y = 0;
static constexpr uint8_t HOME_DIRECTION_Z = 0;
static constexpr uint8_t HOME_DIRECTION_A = 0;

// Husillo: una vuelta completa del motor desplaza linealmente 8 mm.
// El encoder MKS reporta 16384 cuentas por vuelta.
static constexpr int32_t ENCODER_COUNTS_PER_REV = 16384;
static constexpr float LEADSCREW_MM_PER_REV = 8.0f;
static constexpr float ENCODER_COUNTS_PER_MM = ENCODER_COUNTS_PER_REV / LEADSCREW_MM_PER_REV;
static constexpr float ENCODER_COUNTS_PER_DEG = ENCODER_COUNTS_PER_REV / 360.0f;

// Sentido logico de cada motor respecto al eje del robot.
// Si al ordenar un movimiento positivo el encoder corregido baja en vez de subir,
// cambia el signo de ese motor de 1 a -1, o de -1 a 1.
// Se invirtio el sentido positivo de los ejes fisicos X e Y: la correccion se
// aplica tanto al comando de movimiento como a la lectura de encoder/rpm.
static constexpr int8_t MOTOR_DIR_PHYSICAL_X = -1;
static constexpr int8_t MOTOR_DIR_PHYSICAL_Y1 = -1;
static constexpr int8_t MOTOR_DIR_PHYSICAL_Y2 = -1;
static constexpr int8_t MOTOR_DIR_Z = 1;
static constexpr int8_t MOTOR_DIR_A = 1;

// Servo PWM auxiliar tradicional. GPIO17 soporta LEDC/PWM en ESP32-S3 y no se
// usa en este firmware para CAN, USB CDC nativo ni buses de los drivers.
static constexpr gpio_num_t AUX_SERVO_PWM_PIN = GPIO_NUM_17;
static constexpr uint32_t AUX_SERVO_PWM_FREQ_HZ = 50;
static constexpr uint16_t AUX_SERVO_MIN_US = 500;
static constexpr uint16_t AUX_SERVO_MAX_US = 2500;
static constexpr uint16_t AUX_SERVO_CENTER_US = 1500;
static constexpr float AUX_SERVO_MIN_DEG = 0.0f;
static constexpr float AUX_SERVO_MAX_DEG = 180.0f;
// Operacion normal: el servo se controla por ROS2/UI mediante SetGripper o
// comandos SERVO/SERVO_US. Activar solo para pruebas electricas de banco.
static constexpr bool AUX_SERVO_TEST_SWEEP_ENABLED = false;
static constexpr uint32_t AUX_SERVO_TEST_SWEEP_PERIOD_MS = 1000;

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
