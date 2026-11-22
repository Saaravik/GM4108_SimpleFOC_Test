#pragma once

#include <Arduino.h>

namespace cfg {
constexpr int encoderSck = 18;
constexpr int encoderMiso = 19;
constexpr int encoderMosi = 23;
constexpr int encoderCs = 27;
constexpr long encoderClockHz = 1000000;
constexpr int encoderBits = 14;
constexpr uint16_t encoderAngleRegister = 0x3FFF;
constexpr uint16_t encoderDiagnosticsRegister = 0x3FFD;
constexpr uint16_t encoderClearErrorRegister = 0x0001;

constexpr int pwmA = 25;
constexpr int pwmB = 26;
constexpr int pwmC = 32;
constexpr int driverEnable = 33;
constexpr long pwmFrequencyHz = 25000;
constexpr unsigned long serialBaud = 115200;

// Confirmar la variante y los límites de banco antes de habilitar ambos permisos.
constexpr int polePairs = 11;  // GM4108H 24N/22P: 22 polos, 11 pares.
constexpr bool polePairsConfirmed = true;
constexpr bool benchLimitsConfirmed = true;
constexpr float supplyVolts = 8.00f;       // Sustituir por la tensión validada en el Mini.
constexpr float driverLimitVolts = 3.20f;  // Tope de la orden PWM de cada salida.
constexpr float motorLimitVolts = 1.50f;   // Tope de Uq durante control.
constexpr float alignVolts = 1.50f;        // Uq durante alineación.
constexpr float speedLimit = 1.0f;         // rad/s
constexpr float acceleration = 0.50f;      // rad/s²
constexpr float velocityP = 0.20f;
constexpr float velocityI = 0.10f;
constexpr float velocityFilterSeconds = 0.05f;

constexpr uint32_t sensorPeriodMs = 5;
constexpr unsigned minimumGoodSamples = 20;
constexpr float manualTravelRad = 0.0873f;  // Aproximadamente 5 grados.
constexpr float sensorJumpMarginRad = 0.015f;
constexpr float sensorPlausibilityRadPerSec = 20.0f;
constexpr uint32_t velocityWindowUs = 20000;
constexpr uint32_t maxControlGapUs = 100000;
constexpr uint32_t commandTimeoutMs = 5000;
constexpr uint32_t confirmationTimeoutMs = 30000;
constexpr uint32_t monitorIntervalMs = 1000;
constexpr uint32_t diagnosticIntervalMs = 500;
}
