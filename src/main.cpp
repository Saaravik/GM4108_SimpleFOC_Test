#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <stdlib.h>

#include "config.h"

namespace {
MagneticSensorSPI sensor(AS5048_SPI, cfg::encoderCs, cfg::encoderClockHz);
BLDCDriver3PWM driver(cfg::pwmA, cfg::pwmB, cfg::pwmC, cfg::driverEnable);
LowPassFilter overspeedFilter(cfg::velocityFilterSeconds);

constexpr uint8_t monitorChannels = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;

bool inRange(float value, float upper) {
    return isfinite(value) && value >= 0.0f && value <= upper;
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
        if (!enabled && permitted_ && calibrated() && !faulted_) {
            target = 0.0f;
            current_sp = 0.0f;
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
        if (faulted_ || calibrated()) {
            Serial.println(F("FOC: calibracion rechazada. Se requiere reinicio."));
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
            latchFault(F("fallo de inicializacion del motor"));
        } else if (!aligned || !pp_check_result || sensor_direction != Direction::CCW ||
                   !inRange(zero_electric_angle, _2PI)) {
            Serial.print(F("FOC: resultado de pares de polos "));
            Serial.println(pp_check_result ? F("OK") : F("fallo"));
            latchFault(F("calibracion invalida, revisar direccion CCW y 11 pares de polos"));
        } else {
            calibratedZero_ = zero_electric_angle;
            Serial.println(F("FOC: CCW, 11 pares OK, motor deshabilitado."));
        }
    }

    void authorize() {
        checkPermit();
        if (faulted_ || !calibrated() || permitted_ || enabled) {
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
        if (enabled || target != 0.0f || calibrating_ || faulted_) {
            return false;
        }

        disable();  // Cancela la autorización y borra la salida anterior.
        PID_velocity.reset();
        return true;
    }

    void checkCommandState() {
        const bool validFilter = isfinite(LPF_velocity.Tf) && LPF_velocity.Tf >= 0.0f;
        const Direction expectedDirection = calibrated() ? Direction::CCW : Direction::CW;
        const bool validMode = controller == MotionControlType::velocity || controller == MotionControlType::torque;
        if (!validMode ||
            torque_controller != TorqueControlType::voltage ||
            foc_modulation != FOCModulationType::SinePWM || modulation_centered != 1 ||
            motion_downsample != 0 || sensor_offset != 0.0f ||
            sensor_direction != expectedDirection ||
            zero_electric_angle != calibratedZero_ ||
            voltage_limit != cfg::motorLimitVolts ||
            !inRange(velocity_limit, cfg::speedLimit) ||
            !inRange(PID_velocity.limit, voltage_limit) ||
            !isfinite(PID_velocity.P) || !isfinite(PID_velocity.I) || !isfinite(PID_velocity.D) ||
            !isfinite(PID_velocity.output_ramp) || PID_velocity.output_ramp < 0.0f ||
            !validFilter || !isfinite(target)) {
            latchFault(F("parametros de control invalidos"));
            // Mantener la lectura del encoder disponible con la potencia deshabilitada.
            if (!validMode) {
                controller = MotionControlType::velocity;
            }
            torque_controller = TorqueControlType::voltage;
            updateVoltageLimit(cfg::motorLimitVolts);
            motion_downsample = 0;
            sensor_offset = 0.0f;
            sensor_direction = expectedDirection;
            zero_electric_angle = calibratedZero_;
            if (!validFilter) {
                LPF_velocity.Tf = cfg::velocityFilterSeconds;
            }
            return;
        }

        const float targetLimit = controller == MotionControlType::velocity ? velocity_limit : voltage_limit;
        target = enabled ? constrain(target, -targetLimit, targetLimit) : 0.0f;
    }

    void checkFeedback(float speed) {
        if (!enabled) {
            return;
        }

        if (!isfinite(current_sp) || !isfinite(shaft_angle) || !isfinite(shaft_velocity) || !isfinite(speed)) {
            latchFault(F("realimentacion o consigna no finita"));
        } else if (!inRange(fabsf(voltage.q), cfg::motorLimitVolts)) {
            latchFault(F("tension Q fuera de limite"));
        } else if (fabsf(speed) > cfg::speedLimit) {
            latchFault(F("sobrevelocidad"));
        }
    }

    void latchFault(const __FlashStringHelper* reason) {
        disable();
        if (!faulted_) {
            faulted_ = true;
            Serial.print(F("FOC: fallo enclavado. "));
            Serial.println(reason);
        }
    }

private:
    bool calibrated() const { return calibratedZero_ != NOT_SET; }

    float calibratedZero_ = NOT_SET;
    unsigned long permitStart_ = 0;
    bool permitted_ = false;
    bool calibrating_ = false;
    bool faulted_ = false;
};

SupervisedMotor motor(cfg::polePairs, cfg::MOTOR_PHASE_RESISTANCE);
Commander command = Commander(Serial);

bool isLineEnd(char value) {
    return value == '\n' || value == '\r';
}

bool allowMotionCommand(const char* cmd) {
    const char subcommand = cmd[1];
    if (isLineEnd(subcommand) || subcommand == SCMD_TIME || subcommand == SCMD_DOWNSAMPLE) {
        return true;
    }

    char* end = nullptr;
    const double mode = strtod(cmd + 1, &end);
    const bool supported = mode == static_cast<double>(MotionControlType::torque) ||
                           mode == static_cast<double>(MotionControlType::velocity);
    return end != cmd + 1 && isLineEnd(*end) && supported &&
           motor.prepareModeChange(static_cast<MotionControlType>(mode));
}

void doMotor(char* cmd) {
    motor.checkPermit();
    if (cmd[0] == CMD_MOTION_TYPE && !allowMotionCommand(cmd)) {
        // La respuesta nativa de MC devuelve a Studio el modo que sigue activo.
        char query[] = {CMD_MOTION_TYPE, '\n', '\0'};
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
    motor.updateVelocityLimit(cfg::speedLimit);
    motor.voltage_sensor_align = cfg::alignVolts;
    motor.PID_velocity.P = cfg::velocityP;
    motor.PID_velocity.I = cfg::velocityI;
    motor.PID_velocity.D = cfg::velocityD;
    motor.PID_velocity.output_ramp = cfg::velocityOutputRamp;
    motor.LPF_velocity.Tf = cfg::velocityFilterSeconds;

    if (driver.init() != 1) {
        motor.latchFault(F("fallo de inicializacion del driver PWM"));
    } else {
        motor.disable();
    }

    command.add('M', doMotor, "motor");
    command.add('C', doCalibration, "calibracion supervisada (C1)");
    command.add('A', doAuthorize, "autorizar movimiento (A1)");
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
    motor.checkFeedback(overspeedFilter(sensor.getVelocity()));

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
