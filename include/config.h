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

constexpr float supplyVolts = 12.0f;
constexpr float driverLimitVolts = 3.0f;
constexpr float motorLimitVolts = 3.0f;
constexpr float alignVolts = 1.5f;
constexpr float velocityLimit = 1.0f;       // rad/s al arrancar
constexpr float maxVelocityLimit = 20.0f;  // techo de consigna en Velocity
constexpr float overspeedLimit = 10.0f;     // rad/s al arrancar
constexpr float minOverspeedLimit = 1.0f;
constexpr float maxOverspeedLimit = 20.0f;  // techo experimental de protección
constexpr unsigned long overspeedSampleUs = 10000;
constexpr unsigned int overspeedSamples = 3;
constexpr float recoverySpeedLimit = 0.2f;  // rad/s
constexpr unsigned long recoveryStillUs = 500000;
constexpr unsigned long recoveryMaxSampleUs = 2 * overspeedSampleUs;

constexpr float velocityP = 0.20f;
constexpr float velocityI = 0.10f;
constexpr float velocityD = 0.0f;
constexpr float velocityOutputRamp = 2.0f;  // V/s
constexpr float velocityFilterSeconds = 0.05f;

constexpr unsigned long motionPermitMs = 30000;
constexpr unsigned long monitorIntervalMs = 20;
}
