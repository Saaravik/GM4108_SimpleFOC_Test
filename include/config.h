#pragma once

namespace cfg {
constexpr int encoderSck = 18;
constexpr int encoderMiso = 19;
constexpr int encoderMosi = 23;
constexpr int encoderCs = 27;
constexpr long encoderClockHz = 1000000;

constexpr int pwmA = 25;
constexpr int pwmB = 26;
constexpr int pwmC = 32;
constexpr int driverEnable = 33;
constexpr long pwmFrequencyHz = 25000;
constexpr unsigned long serialBaud = 115200;

constexpr int polePairs = 11;
constexpr float MOTOR_LINE_RESISTANCE = 9.5f;
constexpr float MOTOR_PHASE_RESISTANCE = MOTOR_LINE_RESISTANCE / 2.0f;  // Estrella.

constexpr float supplyVolts = 8.0f;
constexpr float driverLimitVolts = 3.2f;
constexpr float motorLimitVolts = 1.5f;
constexpr float alignVolts = 1.5f;
constexpr float speedLimit = 1.0f;  // rad/s

constexpr float velocityP = 0.20f;
constexpr float velocityI = 0.10f;
constexpr float velocityD = 0.0f;
constexpr float velocityFilterSeconds = 0.05f;
constexpr float velocityOutputRamp = 2.0f;  // V/s

constexpr unsigned long motionPermitMs = 15000;
constexpr unsigned long monitorIntervalMs = 20;
}
