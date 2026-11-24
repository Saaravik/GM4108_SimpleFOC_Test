#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <driver/gpio.h>
#include <float.h>
#include <stdlib.h>

#include "config.h"

namespace {
class AlignmentSensor final : public MagneticSensorSPI {
public:
    using MagneticSensorSPI::MagneticSensorSPI;

    void beginSweepCapture() { reading_ = 0; }

    float getAngle() override {
        const float angle = MagneticSensorSPI::getAngle();
        if (reading_ < 2) {
            sweepEnds_[reading_++] = angle;
        }
        return angle;
    }

    float endSweepCapture() {
        const float displacement = reading_ == 2 ? sweepEnds_[1] - sweepEnds_[0] : NAN;
        reading_ = 2;
        return displacement;
    }

private:
    // En FOCMotor::alignSensor() de 2.4.0, las dos primeras consultas son mid_angle y end_angle.
    unsigned reading_ = 2;
    float sweepEnds_[2] = {};
};

AlignmentSensor encoder(AS5048_SPI, cfg::encoderCs, cfg::encoderClockHz);
BLDCDriver3PWM driver(cfg::pwmA, cfg::pwmB, cfg::pwmC, cfg::driverEnable);

constexpr uint8_t monitorChannels = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
static_assert(cfg::alignVolts <= cfg::motorLimitVolts && cfg::motorLimitVolts <= cfg::driverLimitVolts &&
              cfg::driverLimitVolts <= cfg::supplyVolts);

bool inRange(float value, float upper) {
    return isfinite(value) && value >= 0.0f && value <= upper;
}

bool supportedMode(double mode) {
    return mode == static_cast<double>(MotionControlType::torque) ||
           mode == static_cast<double>(MotionControlType::velocity) ||
           mode == static_cast<double>(MotionControlType::angle);
}

class SeededAngleFilter final : public LowPassFilter {
public:
    SeededAngleFilter(const LowPassFilter& filter, float angle) : LowPassFilter(filter.Tf, filter.Ts) {
        y_prev = angle;
    }
};

enum class Fault { none, overspeed, target, configuration, driver, calibration, encoder, internal, enablePin };

bool critical(Fault fault) { return fault >= Fault::driver; }

const char* faultName(Fault fault) {
    switch (fault) {
        case Fault::none: return "none";
        case Fault::overspeed: return "OVERSPEED";
        case Fault::target: return "TARGET";
        case Fault::configuration: return "CONFIGURATION";
        case Fault::driver: return "DRIVER";
        case Fault::calibration: return "CALIBRATION";
        case Fault::encoder: return "ENCODER";
        case Fault::internal: return "INTERNAL";
        case Fault::enablePin: return "ENABLE_PIN";
    }
    return "INTERNAL";
}

constexpr auto enableGpio = static_cast<gpio_num_t>(cfg::driverEnable);
constexpr double radiansPerCount = _2PI / static_cast<double>(1 << 14);

// Commander llama a estos métodos virtuales. BLDCMotor conserva la implementación FOC.
class SupervisedMotor final : public BLDCMotor {
public:
    using BLDCMotor::BLDCMotor;

    void enable() override {
        if (calibrating_) {
            BLDCMotor::enable();
            return;
        }

        checkPermit();
        checkCommandState();
        double angle;
        if (!enabled && permitted_ && calibrated() && fault_ == Fault::none && checkInterlocks(&angle)) {
            target = controller == MotionControlType::angle ? shaftAngle() : 0.0f;
            if (!isfinite(target)) {
                latchFault(Fault::encoder);
                return;
            }
            if (controller == MotionControlType::angle) {
                LPF_angle = SeededAngleFilter(LPF_angle, target);
            }
            current_sp = 0.0f;
            haveSpeedSample_ = true;
            lastSpeedAngle_ = angle;
            lastSpeedSampleUs_ = micros();
            overspeedCount_ = 0;
            BLDCMotor::enable();
        }
    }

    void disable() override {
        digitalWrite(cfg::driverEnable, LOW);
        if (driver && driver->initialized) {
            BLDCMotor::disable();
        }

        permitted_ = false;
        target = 0.0f;
        current_sp = 0.0f;
        voltage = {};
    }

    int initFOC() override {
        // MFR borra la dirección y el cero antes de invocar este método.
        disable();
        sensor_direction = calibrated() ? Direction::CCW : Direction::CW;
        zero_electric_angle = calibratedZero_;
        return 0;
    }

    void calibrate() {
        checkCommandState();
        if (fault_ != Fault::none || calibrated()) {
            Serial.println(F("FOC: calibracion rechazada. Se requiere reinicio."));
            return;
        }
        if (voltage_limit < cfg::alignVolts) {
            Serial.println(F("FOC: ajustar MLU a por lo menos 1.5 V antes de calibrar."));
            return;
        }
        if (!checkInterlocks()) {
            return;
        }
        Serial.println(F("FOC: inicio de calibracion energizada."));
        calibrating_ = true;
        sensor_direction = Direction::UNKNOWN;
        zero_electric_angle = NOT_SET;
        pp_check_result = false;

        const bool initialized = BLDCMotor::init() == 1;
        encoder.beginSweepCapture();
        const bool aligned = initialized && BLDCMotor::initFOC() == 1;
        const float displacement = encoder.endSweepCapture();
        disable();
        calibrating_ = false;

        if (!initialized) {
            latchFault(Fault::driver);
        } else if (!isfinite(displacement)) {
            latchFault(Fault::encoder);
        } else if (!aligned && sensor_direction == Direction::UNKNOWN && zero_electric_angle == NOT_SET) {
            retryCalibration(F("movimiento insuficiente"));
        } else if (!aligned || sensor_direction != Direction::CCW || displacement <= 0.0f ||
                   !inRange(zero_electric_angle, _2PI)) {
            latchFault(Fault::calibration);
        } else if (checkInterlocks()) {
            const float error = fabsf(displacement * cfg::polePairs - _2PI);
            if (error > cfg::polePairTolerance) {
                Serial.printf("FOC: error de pares %.3f rad electricos.\r\n", error);
                retryCalibration(F("desplazamiento fuera de tolerancia"));
                return;
            }
            if (!pp_check_result) {
                Serial.printf("FOC: aviso, PP nativo fuera de margen, aceptado con %.3f rad electricos (max %.2f).\r\n",
                              error, cfg::polePairTolerance);
            }
            calibratedZero_ = zero_electric_angle;
            Serial.println(F("FOC: CCW, 11 pares OK, motor deshabilitado."));
        }
    }

    void authorize() {
        checkPermit();
        checkCommandState();
        if (fault_ != Fault::none || !calibrated() || permitted_ || enabled || !checkInterlocks()) {
            Serial.println(F("FOC: autorizacion rechazada."));
            return;
        }

        target = 0.0f;
        permitStart_ = millis();
        permitted_ = true;
        Serial.print(F("FOC: autorizado por "));
        Serial.print(cfg::motionPermitMs / 1000);
        Serial.println(F(" s. ME1 habilita, ME0 deshabilita."));
    }

    bool checkPermit() {
        if (permitted_ && millis() - permitStart_ >= cfg::motionPermitMs) {
            disable();
            return true;
        }
        return false;
    }

    bool prepareModeChange(MotionControlType nextMode) {
        if (nextMode == controller) {
            return true;
        }
        if (fault_ != Fault::none || target != 0.0f || !prepareConfiguration()) {
            return false;
        }

        disable();  // Borra la salida anterior antes de cambiar de unidades.
        return true;
    }

    bool prepareConfiguration() {
        if (enabled || calibrating_ || critical(fault_)) {
            return false;
        }
        permitted_ = false;
        resetMotionControllers();
        return true;
    }

    void setMechanicalZero() {
        if (!calibrated() || fault_ != Fault::none || !prepareConfiguration() || !checkInterlocks()) {
            Serial.println(F("FOC: Z1 requiere calibracion valida y motor deshabilitado."));
            return;
        }
        mechanicalZero_ = static_cast<float>(sensor_direction) * sensor->getAngle();
        sensor_offset = mechanicalZero_;
        shaft_angle = 0.0f;
        target = 0.0f;
        Serial.println(F("FOC: cero mecanico relativo establecido."));
    }

    float overspeedLimit = cfg::overspeedLimit;

    void checkCommandState() {
        if (critical(fault_)) {
            return;
        }
        if (!structureValid()) {
            latchFault(Fault::internal);
        } else if (!parametersValid()) {
            latchFault(Fault::configuration);
        } else if (!isfinite(target)) {
            latchFault(Fault::target);
        } else if (!enabled) {
            target = 0.0f;
        } else if (controller != MotionControlType::angle) {
            const float targetLimit = controller == MotionControlType::velocity ? velocity_limit : voltage_limit;
            target = constrain(target, -targetLimit, targetLimit);
        }
    }

    void checkFeedback() {
        if (critical(fault_)) {
            return;
        }
        if (!enabled && driver && driver->initialized && gpio_get_level(enableGpio) != LOW) {
            latchFault(Fault::enablePin);
            return;
        }
        if (!calibrated()) {
            return;
        }
        if (enabled) {
            if (!isfinite(shaft_angle) || !isfinite(shaft_velocity)) {
                latchFault(Fault::encoder);
                return;
            }
            if (!isfinite(current_sp) || !inRange(fabsf(voltage.q), voltage_limit)) {
                latchFault(Fault::internal);
                return;
            }
        }

        const unsigned long now = micros();
        const unsigned long elapsed = now - lastSpeedSampleUs_;
        if (haveSpeedSample_ && elapsed < cfg::overspeedSampleUs) {
            return;
        }

        // loopFOC() ya actualizó el sensor. Usar el ángulo acumulado evita saltos en 2π.
        const double angle = sensor->getPreciseAngle();
        if (!isfinite(angle)) {
            latchFault(Fault::encoder);
            return;
        }
        const bool hadSample = haveSpeedSample_;
        const double distance = fabs(angle - lastSpeedAngle_);
        haveSpeedSample_ = true;
        lastSpeedSampleUs_ = now;
        lastSpeedAngle_ = angle;
        if (!hadSample) {
            return;
        }

        const double speed = distance * 1000000.0 / elapsed;
        if (enabled) {
            if (speed <= overspeedLimit) {
                overspeedCount_ = 0;
            } else if (++overspeedCount_ >= cfg::overspeedSamples) {
                latchFault(Fault::overspeed);
            }
        } else if (speed <= cfg::recoverySpeedLimit && elapsed <= cfg::recoveryMaxSampleUs) {
            quietUs_ = min(quietUs_ + elapsed, cfg::recoveryStillUs);
        } else {
            quietUs_ = 0;
        }
    }

    void reportStatus() const {
        const char* state = fault_ != Fault::none ? "FAULT" :
                            enabled ? "ENABLED" :
                            calibrating_ ? "CALIBRATING" :
                            calibrated() ? "READY" : "UNCALIBRATED";
        Serial.printf("FOC: estado=%s calibrated=%s enabled=%s fault=%s\r\n",
                      state, calibrated() ? "true" : "false", enabled ? "true" : "false",
                      faultName(fault_));
    }

    void recover() {
        if (fault_ == Fault::none) {
            Serial.println(F("FOC: sin fallo que recuperar."));
            return;
        }
        if (critical(fault_)) {
            Serial.printf("FOC: recuperacion rechazada, fallo critico %s.\r\n", faultName(fault_));
            return;
        }
        if (enabled || target != 0.0f || calibrating_) {
            Serial.println(F("FOC: recuperacion rechazada, se requiere ME0 y consigna cero."));
            return;
        }
        if (!calibrated() || !structureValid()) {
            latchFault(Fault::internal);
            return;
        }
        if (!parametersValid()) {
            Serial.println(F("FOC: recuperacion rechazada, parametros de control invalidos."));
            return;
        }

        double angle;
        if (!checkInterlocks(&angle)) {
            return;
        }
        const unsigned long elapsed = micros() - lastSpeedSampleUs_;
        const double allowedDistance = cfg::recoverySpeedLimit * (elapsed * 1e-6) + radiansPerCount;
        if (elapsed > cfg::recoveryMaxSampleUs || fabs(angle - lastSpeedAngle_) > allowedDistance) {
            quietUs_ = 0;
        }
        if (quietUs_ < cfg::recoveryStillUs) {
            Serial.println(F("FOC: recuperacion rechazada, rotor en movimiento o reposo sin confirmar."));
            return;
        }

        disable();
        resetMotionControllers();
        LPF_velocity = LowPassFilter(LPF_velocity.Tf);
        fault_ = Fault::none;
        Serial.println(F("FOC: recuperado. Motor deshabilitado. Nueva autorizacion con A1."));
    }

    void latchFault(Fault fault) {
        if (fault_ != Fault::none && (critical(fault_) || !critical(fault))) {
            return;
        }

        disable();
        fault_ = fault;
        haveSpeedSample_ = false;
        quietUs_ = 0;
        if (critical(fault)) {
            calibratedZero_ = NOT_SET;
            zero_electric_angle = NOT_SET;
            sensor_direction = Direction::CW;
        }
        Serial.printf("FOC: fallo enclavado. %s\r\n", faultName(fault));
    }

private:
    bool calibrated() const { return calibratedZero_ != NOT_SET; }

    void resetMotionControllers() {
        PID_velocity.reset();
        P_angle.reset();
    }

    void retryCalibration(const __FlashStringHelper* reason) {
        sensor_direction = Direction::CW;
        zero_electric_angle = NOT_SET;
        Serial.print(F("FOC: calibracion rechazada, "));
        Serial.print(reason);
        Serial.println(F(". Motor deshabilitado. Repetir C1 tras revisar el rotor."));
    }

    bool structureValid() const {
        const Direction expectedDirection = calibrated() ? Direction::CCW : Direction::CW;
        return supportedMode(static_cast<double>(controller)) &&
               torque_controller == TorqueControlType::voltage &&
               foc_modulation == FOCModulationType::SinePWM && modulation_centered == 1 &&
               motion_downsample == 0 && sensor_offset == mechanicalZero_ &&
               sensor_direction == expectedDirection && zero_electric_angle == calibratedZero_;
    }

    bool parametersValid() const {
        return inRange(voltage_limit, cfg::motorLimitVolts) &&
               inRange(velocity_limit, cfg::maxVelocityLimit) &&
               overspeedLimit >= cfg::minOverspeedLimit && overspeedLimit <= cfg::maxOverspeedLimit &&
               inRange(PID_velocity.limit, voltage_limit) &&
               isfinite(PID_velocity.P) && isfinite(PID_velocity.I) && isfinite(PID_velocity.D) &&
               isfinite(PID_velocity.output_ramp) && PID_velocity.output_ramp >= 0.0f &&
               isfinite(LPF_velocity.Tf) && LPF_velocity.Tf >= 0.0f &&
               inRange(P_angle.limit, velocity_limit) &&
               isfinite(P_angle.P) && isfinite(P_angle.I) && isfinite(P_angle.D) &&
               isfinite(P_angle.output_ramp) && P_angle.output_ramp >= 0.0f &&
               isfinite(LPF_angle.Tf) && LPF_angle.Tf >= 0.0f;
    }

    bool checkInterlocks(double* checkedAngle = nullptr) {
        if (!driver || !driver->initialized) {
            latchFault(Fault::driver);
            return false;
        }
        if (gpio_get_level(enableGpio) != LOW) {
            latchFault(Fault::enablePin);
            return false;
        }

        if (!sensor) {
            latchFault(Fault::encoder);
            return false;
        }
        sensor->update();
        const double angle = sensor->getPreciseAngle();
        if (!isfinite(angle)) {
            latchFault(Fault::encoder);
            return false;
        }
        if (checkedAngle) {
            *checkedAngle = angle;
        }
        return true;
    }

    float calibratedZero_ = NOT_SET;
    float mechanicalZero_ = 0.0f;
    unsigned long permitStart_ = 0;
    unsigned long lastSpeedSampleUs_ = 0;
    unsigned long quietUs_ = 0;
    double lastSpeedAngle_ = 0.0;
    unsigned int overspeedCount_ = 0;
    bool permitted_ = false;
    bool calibrating_ = false;
    bool haveSpeedSample_ = false;
    Fault fault_ = Fault::none;
};

SupervisedMotor motor(cfg::polePairs, cfg::MOTOR_PHASE_RESISTANCE);
Commander command = Commander(Serial);

bool isLineEnd(char value) {
    return value == '\n' || value == '\r';
}

double readNumber(const char* value) {
    char* end = nullptr;
    const double number = strtod(value, &end);
    return end != value && isLineEnd(*end) ? number : NAN;
}

bool allowSetting(const char* text, double minimum, double maximum) {
    if (isLineEnd(text[0])) {
        return true;
    }
    const double value = readNumber(text);
    return value >= minimum && value <= maximum && motor.prepareConfiguration();
}

bool allowMotionCommand(const char* cmd) {
    const char subcommand = cmd[1];
    if (isLineEnd(subcommand) || subcommand == SCMD_TIME) {
        return true;
    }

    const double mode = readNumber(cmd + 1);
    return supportedMode(mode) && motor.prepareModeChange(static_cast<MotionControlType>(mode));
}

// Longitud de la consulta nativa que sustituye a una escritura rechazada.
// Un resultado negativo delega la orden original a Commander.
int rejectedWriteOffset(const char* cmd) {
    switch (cmd[0]) {
        case CMD_MOTION_TYPE:
            if (cmd[1] == SCMD_DOWNSAMPLE) {
                return isLineEnd(cmd[2]) ? -1 : 2;
            }
            return allowMotionCommand(cmd) ? -1 : 1;
        case CMD_LIMITS:
            if (cmd[1] == SCMD_LIM_VOLT) {
                return allowSetting(cmd + 2, 0, cfg::motorLimitVolts) ? -1 : 2;
            }
            if (cmd[1] == SCMD_LIM_VEL) {
                return allowSetting(cmd + 2, 0, cfg::maxVelocityLimit) ? -1 : 2;
            }
            break;
        case CMD_V_PID:
        case CMD_A_PID:
            switch (cmd[1]) {
                case SCMD_PID_P:
                case SCMD_PID_I:
                case SCMD_PID_D:
                    return allowSetting(cmd + 2, -FLT_MAX, FLT_MAX) ? -1 : 2;
                case SCMD_PID_RAMP:
                case SCMD_LPF_TF:
                    return allowSetting(cmd + 2, 0, FLT_MAX) ? -1 : 2;
                case SCMD_PID_LIM:
                    return allowSetting(cmd + 2, 0, cmd[0] == CMD_A_PID ? motor.velocity_limit : motor.voltage_limit)
                               ? -1 : 2;
            }
            break;
        case CMD_TORQUE_TYPE:
            return isLineEnd(cmd[1]) || cmd[1] == SCMD_TIME ? -1 : 1;
        case CMD_SENSOR:
        case CMD_PWMMOD:
            if (!isLineEnd(cmd[1]) && !isLineEnd(cmd[2])) {
                return 2;
            }
            break;
        case CMD_STATUS: {
            const double value = readNumber(cmd + 1);
            return isLineEnd(cmd[1]) || value == 0 || value == 1 ? -1 : 1;
        }
        default:
            if (isDigit(cmd[0]) || cmd[0] == '+' || cmd[0] == '-') {
                // Las consignas compuestas pueden escribir límites con EN alto.
                const double value = readNumber(cmd);
                return value >= -FLT_MAX && value <= FLT_MAX ? -1 : 0;
            }
            break;
    }
    return -1;
}

void doMotor(char* cmd) {
    motor.checkPermit();
    const int offset = rejectedWriteOffset(cmd);
    if (offset >= 0) {
        char query[] = {cmd[0], cmd[1], '\0', '\0'};
        query[offset] = '\n';
        query[offset + 1] = '\0';
        command.motor(&motor, query);
    } else {
        command.motor(&motor, cmd);
    }
    motor.checkCommandState();
}

bool confirmed(const char* cmd) {
    return cmd[0] == '1' && isLineEnd(cmd[1]);
}

void doCalibration(char* cmd) {
    if (confirmed(cmd)) {
        motor.calibrate();
    } else {
        Serial.println(F("FOC: usar C1 para calibrar."));
    }
}

void doAuthorize(char* cmd) {
    if (confirmed(cmd)) {
        motor.authorize();
    } else {
        Serial.println(F("FOC: usar A1 para autorizar."));
    }
}

void doMechanicalZero(char* cmd) {
    if (confirmed(cmd)) {
        motor.setMechanicalZero();
    } else {
        Serial.println(F("FOC: usar Z1 para fijar el cero mecanico."));
    }
}

void doOverspeed(char* cmd) {
    motor.checkPermit();
    if (command.verbose != VerboseMode::machine_readable && command.verbose != VerboseMode::nothing) {
        Serial.print(F("S: "));
    }
    char query[] = "\n";
    const bool allowed = allowSetting(cmd, cfg::minOverspeedLimit, cfg::maxOverspeedLimit);
    command.scalar(&motor.overspeedLimit, allowed ? cmd : query);
}

void doRecovery(char* cmd) {
    motor.checkPermit();
    if (command.verbose == VerboseMode::machine_readable) {
        Serial.print(' ');
    }
    if (isLineEnd(cmd[0])) {
        motor.reportStatus();
    } else if (confirmed(cmd)) {
        motor.recover();
    } else {
        Serial.println(F("FOC: usar R para consultar o R1 para recuperar."));
    }
}
}

void setup() {
    pinMode(cfg::driverEnable, OUTPUT);
    digitalWrite(cfg::driverEnable, LOW);
    Serial.begin(cfg::serialBaud);

    SPI.begin(cfg::encoderSck, cfg::encoderMiso, cfg::encoderMosi, cfg::encoderCs);
    encoder.init(&SPI);
    encoder.min_elapsed_time = 0.01f;  // Reduce el ruido de cuantización a baja velocidad.
    motor.linkSensor(&encoder);
    motor.sensor_direction = Direction::CW;  // Sentido bruto hasta que se calibre.

    driver.voltage_power_supply = cfg::supplyVolts;
    driver.voltage_limit = cfg::driverLimitVolts;
    driver.pwm_frequency = cfg::pwmFrequencyHz;
    motor.linkDriver(&driver);

    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;
    motor.feed_forward_voltage = {};
    motor.updateVoltageLimit(cfg::motorLimitVolts);
    motor.updateVelocityLimit(cfg::velocityLimit);
    motor.voltage_sensor_align = cfg::alignVolts;
    motor.PID_velocity.P = cfg::velocityP;
    motor.PID_velocity.I = cfg::velocityI;
    motor.PID_velocity.D = cfg::velocityD;
    motor.PID_velocity.output_ramp = cfg::velocityOutputRamp;
    motor.P_angle.P = cfg::angleP;
    motor.LPF_velocity.Tf = cfg::velocityFilterSeconds;

    if (driver.init() != 1) {
        motor.latchFault(Fault::driver);
    } else if (gpio_input_enable(enableGpio) != ESP_OK) {
        motor.latchFault(Fault::enablePin);
    } else {
        motor.disable();
    }

    command.add('M', doMotor, "motor");
    command.add('C', doCalibration, "calibracion supervisada (C1)");
    command.add('A', doAuthorize, "autorizar movimiento (A1)");
    command.add('S', doOverspeed, "umbral de sobrevelocidad (rad/s)");
    command.add('Z', doMechanicalZero, "cero mecanico relativo (Z1)");
    command.add('R', doRecovery, "estado y recuperacion (R1)");
    command.verbose = VerboseMode::machine_readable;

    motor.useMonitoring(Serial);
    motor.monitor_start_char = '\0';
    motor.monitor_end_char = '\0';
    motor.monitor_downsample = 0;
    motor.monitor_variables = monitorChannels;
}

void loop() {
    if (motor.checkPermit()) {
        Serial.println(F("FOC: autorizacion caducada. Motor deshabilitado."));
    }
    motor.loopFOC();
    motor.move();
    motor.checkFeedback();

    static unsigned long lastMonitor = 0;
    const unsigned long now = millis();
    // Cuatro columnas y hasta seis decimales caben en 64 bytes.
    if (now - lastMonitor >= cfg::monitorIntervalMs && Serial.availableForWrite() >= 64) {
        lastMonitor = now;
        motor.monitor_variables &= monitorChannels;
        motor.monitor_decimals = min(motor.monitor_decimals, 6U);
        motor.monitor();
    }
    command.run();
}
