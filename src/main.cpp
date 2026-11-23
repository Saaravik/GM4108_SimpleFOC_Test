#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

#include "config.h"

namespace {
static_assert(!cfg::alignOnBoot, "El firmware de diagnostico requiere alignOnBoot = false");

// BLDCMotor usa el driver incluso al cambiar su estado lógico desde Commander.
class DiagnosticDriver final : public BLDCDriver {
public:
    int init() override { return 0; }
    void enable() override {}
    void disable() override {}
    void setPwm(float, float, float) override {}
    void setPhaseState(PhaseState, PhaseState, PhaseState) override {}
};

class DiagnosticMotor final : public BLDCMotor {
public:
    using BLDCMotor::BLDCMotor;

    int initFOC() override {
        // Commander puede pedir una realineación y restablecer el sentido del sensor.
        sensor_direction = Direction::CW;
        return 0;
    }
};

MagneticSensorSPI sensor(AS5048_SPI, cfg::encoderCs, cfg::encoderClockHz);
DiagnosticDriver driver;
DiagnosticMotor motor(cfg::polePairs, cfg::MOTOR_PHASE_RESISTANCE);
Commander command = Commander(Serial);

void doMotor(char* cmd) {
    command.motor(&motor, cmd);
}
}

void setup() {
    pinMode(cfg::driverEnable, OUTPUT_OPEN_DRAIN);
    digitalWrite(cfg::driverEnable, LOW);
    Serial.begin(cfg::serialBaud);

    SPI.begin(cfg::encoderSck, cfg::encoderMiso, cfg::encoderMosi, cfg::encoderCs);
    sensor.init(&SPI);
    sensor.min_elapsed_time = 0.01f;  // Reduce el ruido de cuantización a baja velocidad.
    motor.linkSensor(&sensor);
    motor.sensor_direction = Direction::CW;  // Sentido bruto del encoder.
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

    char motor_id = 'M';
    command.add(motor_id, doMotor, "motor");

    motor.useMonitoring(Serial);
    motor.monitor_start_char = motor_id;
    motor.monitor_end_char = motor_id;
    motor.monitor_downsample = 0;
    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL;

    command.verbose = VerboseMode::machine_readable;
}

void loop() {
    motor.loopFOC();
    motor.move();
    motor.monitor();
    command.run();
}
