#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <driver/gpio.h>
#include <float.h>
#include <stdlib.h>

#include "config.h"

namespace {
MagneticSensorSPI sensor(AS5048_SPI, cfg::encoderCs, cfg::encoderClockHz);
BLDCDriver3PWM driver(cfg::pwmA, cfg::pwmB, cfg::pwmC, cfg::driverEnable);

constexpr uint8_t monitorChannels = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
static_assert(cfg::alignVolts <= cfg::motorLimitVolts && cfg::motorLimitVolts <= cfg::driverLimitVolts &&
              cfg::driverLimitVolts <= cfg::supplyVolts);

bool inRange(float value, float upper) {
    return isfinite(value) && value >= 0.0f && value <= upper;
}

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
constexpr uint16_t angleCounts = 1 << 14;
constexpr double radiansPerCount = _2PI / static_cast<double>(angleCounts);

// Sondeo de salud independiente. MagneticSensorSPI sigue alimentando el lazo FOC.
bool readCheckedAngle(uint16_t& angle) {
    constexpr uint16_t readDiagnostics = 0x7FFD;  // DIAAGC, con paridad par
    constexpr uint16_t readAngle = 0xFFFF;
    constexpr uint16_t spiError = 1 << 14;
    constexpr uint16_t diagnosticsMask = 0x0F00;
    constexpr uint16_t offsetCompensated = 0x0100;
    SPI.beginTransaction(SPISettings(cfg::encoderClockHz, MSBFIRST, SPI_MODE1));
    auto transfer = [](uint16_t word) {
        digitalWrite(cfg::encoderCs, LOW);
        const uint16_t response = SPI.transfer16(word);
        digitalWrite(cfg::encoderCs, HIGH);
        delayMicroseconds(1);
        return response;
    };
    transfer(readDiagnostics);
    const uint16_t diagnostics = transfer(readAngle);
    const uint16_t response = transfer(0);
    SPI.endTransaction();

    angle = response & (angleCounts - 1);
    return !(diagnostics & spiError) && !(response & spiError) &&
           !__builtin_parity(diagnostics) && !__builtin_parity(response) &&
           (diagnostics & diagnosticsMask) == offsetCompensated;
}

double angularDistance(uint16_t angle, uint16_t previous) {
    const unsigned delta = (static_cast<unsigned>(angle) + angleCounts - previous) % angleCounts;
    return min(delta, angleCounts - delta) * radiansPerCount;
}

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
        uint16_t angle;
        if (!enabled && permitted_ && calibrated() && fault_ == Fault::none && checkInterlocks(&angle)) {
            target = 0.0f;
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
        const bool aligned = initialized && BLDCMotor::initFOC() == 1;
        disable();
        calibrating_ = false;

        if (!initialized) {
            latchFault(Fault::driver);
        } else if (!aligned || !pp_check_result || sensor_direction != Direction::CCW ||
                   !inRange(zero_electric_angle, _2PI)) {
            Serial.print(F("FOC: resultado de pares de polos "));
            Serial.println(pp_check_result ? F("OK") : F("fallo"));
            latchFault(Fault::calibration);
        } else if (checkInterlocks()) {
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
        PID_velocity.reset();
        return true;
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
        } else {
            const float targetLimit = controller == MotionControlType::velocity ? velocity_limit : voltage_limit;
            target = enabled ? constrain(target, -targetLimit, targetLimit) : 0.0f;
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

        uint16_t angle;
        if (!readCheckedAngle(angle)) {
            latchFault(Fault::encoder);
            return;
        }
        const bool hadSample = haveSpeedSample_;
        const double distance = angularDistance(angle, lastSpeedAngle_);
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

        uint16_t angle;
        if (!checkInterlocks(&angle)) {
            return;
        }
        const unsigned long elapsed = micros() - lastSpeedSampleUs_;
        const double allowedDistance = cfg::recoverySpeedLimit * (elapsed * 1e-6) + radiansPerCount;
        if (elapsed > cfg::recoveryMaxSampleUs || angularDistance(angle, lastSpeedAngle_) > allowedDistance) {
            quietUs_ = 0;
        }
        if (quietUs_ < cfg::recoveryStillUs) {
            Serial.println(F("FOC: recuperacion rechazada, rotor en movimiento o reposo sin confirmar."));
            return;
        }

        disable();
        PID_velocity.reset();
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

    bool structureValid() const {
        const Direction expectedDirection = calibrated() ? Direction::CCW : Direction::CW;
        return (controller == MotionControlType::velocity || controller == MotionControlType::torque) &&
               torque_controller == TorqueControlType::voltage &&
               foc_modulation == FOCModulationType::SinePWM && modulation_centered == 1 &&
               motion_downsample == 0 && sensor_offset == 0.0f &&
               sensor_direction == expectedDirection && zero_electric_angle == calibratedZero_;
    }

    bool parametersValid() const {
        return inRange(voltage_limit, cfg::motorLimitVolts) &&
               inRange(velocity_limit, cfg::maxVelocityLimit) &&
               overspeedLimit >= cfg::minOverspeedLimit && overspeedLimit <= cfg::maxOverspeedLimit &&
               inRange(PID_velocity.limit, voltage_limit) &&
               isfinite(PID_velocity.P) && isfinite(PID_velocity.I) && isfinite(PID_velocity.D) &&
               isfinite(PID_velocity.output_ramp) && PID_velocity.output_ramp >= 0.0f &&
               isfinite(LPF_velocity.Tf) && LPF_velocity.Tf >= 0.0f;
    }

    bool checkInterlocks(uint16_t* checkedAngle = nullptr) {
        if (!driver || !driver->initialized) {
            latchFault(Fault::driver);
            return false;
        }
        if (gpio_get_level(enableGpio) != LOW) {
            latchFault(Fault::enablePin);
            return false;
        }

        uint16_t angle;
        if (!readCheckedAngle(angle)) {
            latchFault(Fault::encoder);
            return false;
        }
        if (checkedAngle) {
            *checkedAngle = angle;
        }
        return true;
    }

    float calibratedZero_ = NOT_SET;
    unsigned long permitStart_ = 0;
    unsigned long lastSpeedSampleUs_ = 0;
    unsigned long quietUs_ = 0;
    uint16_t lastSpeedAngle_ = 0;
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
    const bool supported = mode == static_cast<double>(MotionControlType::torque) ||
                           mode == static_cast<double>(MotionControlType::velocity);
    return supported && motor.prepareModeChange(static_cast<MotionControlType>(mode));
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
            switch (cmd[1]) {
                case SCMD_PID_P:
                case SCMD_PID_I:
                case SCMD_PID_D:
                    return allowSetting(cmd + 2, -FLT_MAX, FLT_MAX) ? -1 : 2;
                case SCMD_PID_RAMP:
                case SCMD_LPF_TF:
                    return allowSetting(cmd + 2, 0, FLT_MAX) ? -1 : 2;
                case SCMD_PID_LIM:
                    return allowSetting(cmd + 2, 0, motor.voltage_limit) ? -1 : 2;
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
                // El formato compuesto de Velocity podría escribir voltage_limit con EN alto.
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
    sensor.init(&SPI);
    sensor.min_elapsed_time = 0.01f;  // Reduce el ruido de cuantización a baja velocidad.
    motor.linkSensor(&sensor);
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
