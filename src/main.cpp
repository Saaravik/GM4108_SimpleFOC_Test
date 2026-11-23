#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

namespace {
MagneticSensorSPI sensor(AS5048_SPI, cfg::encoderCs, cfg::encoderClockHz);
BLDCDriver3PWM driver(cfg::pwmA, cfg::pwmB, cfg::pwmC, cfg::driverEnable);
BLDCMotor motor(cfg::polePairs, cfg::MOTOR_PHASE_RESISTANCE);
Commander command(Serial);

bool focReady = false;
unsigned long lastLoopUs = 0;
unsigned long lastWithinSpeedLimitMs = 0;

void stopMotor() {
    motor.disable();
    motor.target = 0;
    motor.current_sp = 0;
    motor.voltage = {};
}

void fault(const char* reason) {
    focReady = false;
    digitalWrite(cfg::driverEnable, LOW);
    if (driver.initialized) stopMotor();
    Serial.println(reason);
}

bool endOfCommand(char ch) {
    return ch == '\n';
}

bool numberInRange(const char* input, float minValue, float maxValue, bool integer = false) {
    char* end;
    const float value = strtof(input, &end);
    return end != input && endOfCommand(*end) && isfinite(value) &&
           value >= minValue && value <= maxValue && (!integer || value == truncf(value));
}

bool allowMonitor(const char* cmd) {
    const char* value = cmd + 2;
    switch (cmd[1]) {
        case SCMD_DOWNSAMPLE:
            return endOfCommand(*value) || numberInRange(value, 0, 10000, true);
        case SCMD_CLEAR:
        case SCMD_TIME:
            return endOfCommand(*value);
        case SCMD_GET:
            return numberInRange(value, 0, 7, true);
        case CMD_DECIMAL:
            return numberInRange(value, 0, 6, true);
        case SCMD_SET:
            if (endOfCommand(*value)) return true;
            return strspn(value, "01") == 7 && endOfCommand(value[7]);
        default:
            return false;
    }
}

bool allowMotorCommand(const char* cmd) {
    const char group = cmd[0];
    const char sub = cmd[1];
    const char* value = cmd + 2;

    if (endOfCommand(group)) return true;  // Consulta de consigna.
    if (isDigit(group) || group == '-' || group == '+') {
        // Commander admite un segundo valor que cambia el límite de tensión.
        return numberInRange(cmd, -motor.velocity_limit, motor.velocity_limit) &&
               (motor.enabled || numberInRange(cmd, 0, 0));
    }

    switch (group) {
        case CMD_STATUS:
            return endOfCommand(sub) ||
                   (sub == '0' && endOfCommand(*value)) ||
                   (sub == '1' && endOfCommand(*value) && focReady && motor.target == 0);
        case CMD_V_PID:
            switch (sub) {
                case SCMD_PID_P:
                    return endOfCommand(*value) || numberInRange(value, 0, cfg::maxVelocityP);
                case SCMD_PID_I:
                    return endOfCommand(*value) || numberInRange(value, 0, cfg::maxVelocityI);
                case SCMD_PID_D:
                    return endOfCommand(*value) || numberInRange(value, 0, cfg::maxVelocityD);
                case SCMD_LPF_TF:
                    return endOfCommand(*value) ||
                           numberInRange(value, cfg::minFilterSeconds, cfg::maxFilterSeconds);
                case SCMD_PID_LIM:
                case SCMD_PID_RAMP:
                    return endOfCommand(*value);
                default:
                    return false;
            }
        case CMD_MONITOR:
            return allowMonitor(cmd);
        case CMD_MOTION_TYPE:
        case CMD_TORQUE_TYPE:
            return endOfCommand(sub) ||
                   ((sub == SCMD_TIME || sub == SCMD_DOWNSAMPLE) && endOfCommand(*value));
        case CMD_RESIST:
        case CMD_KV_RATING:
            return endOfCommand(sub);
        case CMD_LIMITS:
            return (sub == SCMD_LIM_CURR || sub == SCMD_LIM_VOLT || sub == SCMD_LIM_VEL) &&
                   endOfCommand(*value);
        case CMD_INDUCTANCE:
        case CMD_SENSOR:
        case CMD_PWMMOD:
        case CMD_A_PID:
        case CMD_C_D_PID:
        case CMD_C_Q_PID:
            return sub >= 'A' && sub <= 'Z' && endOfCommand(*value);
        default:
            return false;
    }
}

void doMotor(char* cmd) {
    if (!allowMotorCommand(cmd)) {
        Serial.println(F("err"));
        return;
    }

    if (cmd[0] == CMD_STATUS) {
        if (cmd[1] == '0' && driver.initialized) stopMotor();
        if (cmd[1] == '0' || (cmd[1] == '1' && motor.enabled)) {
            cmd[1] = '\n';
            cmd[2] = '\0';
        }
    }
    command.motor(&motor, cmd);
}

bool allowCommanderOption(const char* line, char maxValue) {
    return endOfCommand(line[1]) ||
           (line[1] >= '0' && line[1] <= maxValue && endOfCommand(line[2]));
}

void runCommander() {
    // Conservar una trama por vuelta y descartar la línea entera si resulta inválida.
    // El lector de Commander 2.4.0 reinicia el búfer al llenarlo y sigue leyendo.
    static char line[MAX_COMMAND_LENGTH + 1];
    static size_t length = 0;
    static bool discard = false;

    for (int budget = MAX_COMMAND_LENGTH; budget > 0 && Serial.available(); --budget) {
        const int ch = Serial.read();
        if (ch != '\r' && ch != '\n') {
            if (ch < ' ' || ch > '~' || length >= MAX_COMMAND_LENGTH - 1) discard = true;
            if (!discard) line[length++] = static_cast<char>(ch);
            continue;
        }

        const bool complete = !discard && length != 0;
        line[length] = '\n';
        line[length + 1] = '\0';
        length = 0;
        discard = false;
        if (!complete) continue;

        if ((line[0] == CMD_DECIMAL && !allowCommanderOption(line, '6')) ||
            (line[0] == CMD_VERBOSE && !allowCommanderOption(line, '3'))) {
            Serial.println(F("err"));
        } else {
            command.run(line);
        }
        return;
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
    motor.linkDriver(&driver);

    driver.voltage_power_supply = cfg::supplyVolts;
    driver.voltage_limit = cfg::driverLimitVolts;
    driver.pwm_frequency = cfg::pwmFrequencyHz;
    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;
    motor.updateVoltageLimit(cfg::motorLimitVolts);
    motor.updateVelocityLimit(cfg::speedLimit);
    motor.voltage_sensor_align = cfg::alignVolts;
    motor.PID_velocity.P = cfg::velocityP;
    motor.PID_velocity.I = cfg::velocityI;
    motor.PID_velocity.D = cfg::velocityD;
    motor.PID_velocity.output_ramp = cfg::velocityOutputRamp;
    motor.LPF_velocity.Tf = cfg::velocityFilterSeconds;
    motor.target = 0;

    motor.useMonitoring(Serial);
    motor.monitor_start_char = 'M';
    motor.monitor_end_char = 'M';
    motor.monitor_downsample = 0;
    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL;
    command.verbose = VerboseMode::machine_readable;
    command.add('M', doMotor, "motor GM4108");

    if (!driver.init()) {
        fault("Fallo driver.init()");
        return;
    }
    if (!cfg::alignOnBoot) {
        motor.sensor_direction = Direction::CW;  // Sentido bruto para el diagnóstico por USB.
        Serial.println(F("Prueba sin potencia: EN bajo, alineacion desactivada"));
        return;
    }
    if (!motor.init()) {
        fault("Fallo motor.init()");
        return;
    }
    if (!motor.initFOC() || !motor.pp_check_result ||
        !isfinite(motor.zero_electric_angle) || motor.zero_electric_angle == NOT_SET) {
        fault("Fallo de alineacion FOC");
        return;
    }
    stopMotor();
    focReady = true;
    Serial.println(F("FOC listo, EN bajo; ME1 habilita con consigna cero"));
}

void loop() {
    const unsigned long nowUs = micros();
    if (motor.enabled && nowUs - lastLoopUs > cfg::maxLoopGapUs) {
        fault("Fallo: intervalo de control excedido");
    }
    lastLoopUs = nowUs;

    motor.loopFOC();
    motor.move();

    // Usar la velocidad nativa anterior al LPF ajustable para el corte.
    const unsigned long nowMs = millis();
    const float speed = sensor.getVelocity();
    if (motor.enabled && (!isfinite(speed) || !isfinite(motor.current_sp))) {
        fault("Fallo: lectura u orden invalida");
    } else if (!motor.enabled || fabsf(speed) <= cfg::speedLimit) {
        lastWithinSpeedLimitMs = nowMs;
    } else if (nowMs - lastWithinSpeedLimitMs >= cfg::overspeedHoldMs) {
        fault("Fallo: sobrevelocidad");
    }

    if (Serial.availableForWrite() >= 96) motor.monitor();
    runCommander();
}
