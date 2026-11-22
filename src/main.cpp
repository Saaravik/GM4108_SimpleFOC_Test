#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <driver/gpio.h>

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

namespace {
enum class State { SAFE, SENSOR_TEST, READY, CALIBRATION, MOTOR_CONTROL };
enum class Calibration { Pending, Passed, Failed, Aborted };
enum class Confirmation { None, Calibration, SpeedTest };
enum class SpeedTestPhase { Inactive, InitialHold, Run, FinalHold };

constexpr float speedTestTarget = 0.2f;
constexpr uint32_t speedTestRunAtUs = 1000000;
constexpr uint32_t speedTestStopAtUs = speedTestRunAtUs + 5000000;
constexpr uint32_t speedTestEndAtUs = speedTestStopAtUs + 1000000;
constexpr uint32_t speedTestReportUs = 200000;

struct SpeedTest {
  SpeedTestPhase phase = SpeedTestPhase::Inactive;
  uint32_t startedAt = 0;
  uint32_t nextReportUs = speedTestReportUs;
  uint32_t measuredUs = 0;
  float initialAngle = 0;
  float displacement = 0;
  float peakSpeed = 0;

  bool active() const { return phase != SpeedTestPhase::Inactive; }
};

State state = State::SAFE;
Calibration calibration = Calibration::Pending;
BLDCMotor motor(cfg::polePairs);

bool driverReady = false;
bool sensorBusReady = false;
bool enableRequested = false;
Confirmation pendingConfirmation = Confirmation::None;
bool calibrationAborted = false;
bool calibrationSensorFault = false;
bool timeoutStopping = false;
float requestedSpeed = 0;
float rampedSpeed = 0;
uint32_t confirmationAt = 0;
uint32_t lastAuthorizationAt = 0;
uint32_t lastControlAt = 0;
uint32_t lastSensorAt = 0;
uint32_t lastReportAt = 0;
const char* lastFault = "-";
SpeedTest speedTest;

void pollSerial();

void lowerEnable() {
  digitalWrite(cfg::driverEnable, LOW);
  enableRequested = false;
}

bool calibrationInterrupted() {
  return state == State::CALIBRATION && (calibrationSensorFault || calibrationAborted);
}

class CalibrationDriver : public BLDCDriver3PWM {
 public:
  using BLDCDriver3PWM::BLDCDriver3PWM;

  void enable() override {
    if (!calibrationInterrupted()) BLDCDriver3PWM::enable();
  }

  void setPwm(float ua, float ub, float uc) override {
    // initFOC() puede seguir escribiendo tensiones tras fallar una lectura.
    if (calibrationInterrupted()) BLDCDriver3PWM::setPwm(0, 0, 0);
    else BLDCDriver3PWM::setPwm(ua, ub, uc);
  }
};

CalibrationDriver driver(cfg::pwmA, cfg::pwmB, cfg::pwmC, cfg::driverEnable);

void cutCalibrationPower() {
  lowerEnable();
  if (driverReady) motor.disable();
}

// Sensor de SimpleFOC para FOC, con bus y validación propios del AS5048A.
class CheckedAS5048A : public Sensor {
 public:
  void init() override {
    // SimpleFOC 2.4.0 llama cuatro veces a getSensorAngle() para sembrar su estado.
    initializing = true;
    Sensor::init();
    initializing = false;
  }

  float getSensorAngle() override {
    if (state == State::CALIBRATION) pollSerial();
    if (calibrationInterrupted()) return -1;

    SPI.beginTransaction(settings);
    // MISO lleva la respuesta al comando anterior. Guardamos también la
    // primera respuesta de cada muestra en la traza, aunque no sea un ángulo.
    transfer(readCommand(cfg::encoderAngleRegister));
    const uint16_t angle = transfer(readCommand(cfg::encoderDiagnosticsRegister));
    const uint16_t diagnostics = transfer(nop);

    const uint16_t frames = angle | diagnostics;
    const bool ef = frames & 0x4000;
    const bool parity = oddParity(angle) || oddParity(diagnostics);
    if (ef) {
      // Leer 0x0001 devuelve y borra el error del chip. Conservamos la
      // evidencia en software y rechazamos la muestra aunque EF se borre.
      transfer(readCommand(cfg::encoderClearErrorRegister));
      const uint16_t error = transfer(nop);
      errorRegister = oddParity(error) ? -1 : (error & 0x0007);
    }
    SPI.endTransaction();
    if (startupSamples < startupSampleLimit && ++startupSamples == startupSampleLimit) {
      startupTraceReady = true;
    }

    diagnosticRegister = diagnostics & 0x3FFF;
    if (parity || ef) {
      parityErrors += parity;
      flagErrors += ef;
      return reject("SPI: paridad o EF");
    }
    // OCF debe estar activo. COF y las alertas de campo impiden el control.
    if ((diagnosticRegister & 0x0F00) != 0x0100) {
      ++diagnosticErrors;
      return reject("AS5048A: OCF/COF/campo magnetico");
    }

    constexpr int countsPerTurn = 1 << cfg::encoderBits;
    constexpr float radiansPerCount = _2PI / countsPerTurn;
    const int count = angle & (countsPerTurn - 1);
    const uint32_t now = micros();
    if (goodSamples == 0) {
      manualOrigin = count * radiansPerCount;
      velocityAt = now;
      windowTravel = 0;
    } else {
      int delta = count - rawCount;
      if (delta > countsPerTurn / 2) delta -= countsPerTurn;
      if (delta < -countsPerTurn / 2) delta += countsPerTurn;
      const float travel = delta * radiansPerCount;
      const float sampleSeconds = (now - sampledAt) * 1e-6f;
      if (fabsf(travel) > cfg::sensorJumpMarginRad +
                              cfg::sensorPlausibilityRadPerSec * sampleSeconds) {
        ++jumpErrors;
        return reject("AS5048A: salto angular");
      }

      windowTravel += travel;
      const uint32_t windowUs = now - velocityAt;
      if (windowUs >= cfg::velocityWindowUs) {
        estimatedSpeed = windowTravel / (windowUs * 1e-6f);
        speedValid = true;
        windowTravel = 0;
        velocityAt = now;
      }
    }

    rawCount = count;
    sampledAt = now;
    if (goodSamples < cfg::minimumGoodSamples) ++goodSamples;
    const float radians = count * radiansPerCount;
    if (state == State::SENSOR_TEST) {
      const float travel = remainderf(radians - manualOrigin, _2PI);
      if (fabsf(travel) >= cfg::manualTravelRad) manualTravelSeen = true;
    }
    return radians;
  }

  float getVelocity() override { return speedValid ? estimatedSpeed : 0; }
  bool healthy() const { return goodSamples >= cfg::minimumGoodSamples && speedValid; }

  void printStartupTraceIfReady() {
    if (!startupTraceReady) return;
    startupTraceReady = false;
    printStartupTrace();
  }

  void printStartupTrace() const {
    Serial.printf("SPI_BOOT | modo=1 reloj=%lu Hz CS=%d | par=OK o IMPAR\n",
                  static_cast<unsigned long>(cfg::encoderClockHz), cfg::encoderCs);
    for (unsigned i = 0; i < startupFrameCount; ++i) {
      const StartupFrame& frame = startupFrames[i];
      char previous[7] = "--";
      if (i > 0) snprintf(previous, sizeof(previous), "0x%04X",
                          static_cast<unsigned>(startupFrames[i - 1].tx));
      Serial.printf("SPI_BOOT #%02u t=%luus %s muestra=%u TX=0x%04X parTX=%s | "
                    "RX=0x%04X respTX=%s parRX=%s EF=%u\n",
                    i + 1, static_cast<unsigned long>(frame.at), frame.fromInit ? "init" : "loop",
                    frame.sample, static_cast<unsigned>(frame.tx), oddParity(frame.tx) ? "IMPAR" : "OK",
                    static_cast<unsigned>(frame.rx), previous, oddParity(frame.rx) ? "IMPAR" : "OK",
                    static_cast<unsigned>((frame.rx >> 14) & 1));
    }
  }

  void resetManualTest() {
    goodSamples = 0;
    speedValid = false;
    manualTravelSeen = false;
  }

  bool manualTravelSeen = false;
  bool speedValid = false;
  unsigned goodSamples = 0;
  int rawCount = 0;
  int errorRegister = -1;
  uint16_t diagnosticRegister = 0;
  uint32_t parityErrors = 0;
  uint32_t flagErrors = 0;
  uint32_t diagnosticErrors = 0;
  uint32_t jumpErrors = 0;

 private:
  static constexpr uint16_t nop = 0x0000;
  static constexpr unsigned startupSampleLimit = 8;
  struct StartupFrame {
    uint32_t at;
    uint16_t tx;
    uint16_t rx;
    unsigned sample;
    bool fromInit;
  };

  static bool oddParity(uint16_t frame) { return __builtin_parity(frame); }

  static uint16_t readCommand(uint16_t address) {
    const uint16_t command = address | 0x4000;
    return command | (oddParity(command) ? 0x8000 : 0);
  }

  uint16_t transfer(uint16_t frame) {
    const bool trace = startupSamples < startupSampleLimit;
    const uint32_t at = trace ? micros() : 0;
    digitalWrite(cfg::encoderCs, LOW);
    delayMicroseconds(1);
    const uint16_t reply = SPI.transfer16(frame);
    digitalWrite(cfg::encoderCs, HIGH);
    delayMicroseconds(1);
    if (trace) {
      startupFrames[startupFrameCount++] = {at, frame, reply, startupSamples + 1, initializing};
    }
    return reply;
  }

  float reject(const char* reason) {
    lastFault = reason;
    resetManualTest();
    if (state == State::CALIBRATION) {
      calibrationSensorFault = true;
      cutCalibrationPower();
    } else if (state == State::MOTOR_CONTROL) {
      lowerEnable();
    }
    return -1;  // Sensor::update() descarta ángulos negativos.
  }

  SPISettings settings{cfg::encoderClockHz, MSBFIRST, SPI_MODE1};
  StartupFrame startupFrames[startupSampleLimit * 5];  // Tres tramas más dos si aparece EF.
  unsigned startupFrameCount = 0;
  unsigned startupSamples = 0;
  bool initializing = false;
  bool startupTraceReady = false;
  uint32_t sampledAt = 0;
  uint32_t velocityAt = 0;
  float estimatedSpeed = 0;
  float windowTravel = 0;
  float manualOrigin = 0;
};

CheckedAS5048A sensor;

const char* stateName() {
  switch (state) {
    case State::SAFE: return "SAFE";
    case State::SENSOR_TEST: return "SENSOR_TEST";
    case State::READY: return "READY";
    case State::CALIBRATION: return "CALIBRATION";
    case State::MOTOR_CONTROL: return "MOTOR_CONTROL";
  }
  return "?";
}

const char* calibrationName() {
  switch (calibration) {
    case Calibration::Pending: return "pendiente";
    case Calibration::Passed: return "OK";
    case Calibration::Failed: return "fallida";
    case Calibration::Aborted: return "abortada";
  }
  return "?";
}

void deenergize() {
  lowerEnable();
  if (driverReady) motor.disable();  // Requiere PWM inicializado y driver enlazado.
  requestedSpeed = 0;
  rampedSpeed = 0;
  motor.target = 0;
  motor.current_sp = 0;
  timeoutStopping = false;
  pendingConfirmation = Confirmation::None;
}

void reportSpeedTestResult(const char* reason) {
  if (!speedTest.active()) return;
  speedTest.phase = SpeedTestPhase::Inactive;
  const float mean = speedTest.measuredUs > 0
                         ? speedTest.displacement / (speedTest.measuredUs * 1e-6f) : 0;
  Serial.printf("TEST_SPEED FIN | desplazamiento=%.4f rad media=%.3f rad/s "
                "max_abs=%.3f rad/s | motivo=%s | %s EN=LOW\n",
                speedTest.displacement, mean, speedTest.peakSpeed, reason, stateName());
}

void failSafe(const char* reason) {
  deenergize();
  state = State::SAFE;
  lastFault = reason;
  reportSpeedTestResult(reason);
  Serial.printf("SAFE | %s\n", lastFault);
}

bool settingsValid() {
  return cfg::polePairs > 0 && cfg::supplyVolts >= 8 && cfg::supplyVolts <= 35 &&
         cfg::driverLimitVolts <= cfg::supplyVolts &&
         2 * cfg::motorLimitVolts <= cfg::driverLimitVolts &&
         cfg::alignVolts > 0 && cfg::alignVolts <= cfg::motorLimitVolts &&
         isfinite(cfg::speedLimit) && cfg::speedLimit > 0 &&
         isfinite(cfg::acceleration) && cfg::acceleration > 0;
}

void prepareDriver() {
  if (state != State::SENSOR_TEST || !sensor.healthy() || !sensor.manualTravelSeen) {
    Serial.println("INIT requiere SENSOR y giro manual con lecturas validas.");
    return;
  }
  if (!settingsValid()) {
    failSafe("Revisar parametros en config.h");
    return;
  }
  lowerEnable();
  if (!driverReady) {
    driver.voltage_power_supply = cfg::supplyVolts;
    driver.voltage_limit = cfg::driverLimitVolts;
    driver.pwm_frequency = cfg::pwmFrequencyHz;
    if (driver.init() != 1) {
      failSafe("Fallo driver.init()");
      return;
    }
    motor.linkDriver(&driver);
    driverReady = true;
  }

  deenergize();
  calibration = Calibration::Pending;
  state = State::READY;
  Serial.println("READY | PWM inicializado en ESP32, EN bajo. CALIBRATE solicita confirmacion.");
}

bool mayCalibrate() {
  if (state != State::READY || calibration == Calibration::Passed || !sensor.healthy()) {
    Serial.println("Calibracion requiere READY sin calibracion y encoder valido.");
    return false;
  }
  if (!cfg::polePairsConfirmed || !cfg::benchLimitsConfirmed) {
    Serial.println("Bloqueado: confirmar pares de polos y limites de banco en config.h.");
    return false;
  }
  return true;
}

void runCalibration() {
  state = State::CALIBRATION;
  calibrationAborted = false;
  calibrationSensorFault = false;
  pendingConfirmation = Confirmation::None;
  motor.linkSensor(&sensor);
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.voltage_limit = cfg::motorLimitVolts;
  motor.voltage_sensor_align = cfg::alignVolts;
  motor.velocity_limit = cfg::speedLimit;
  motor.PID_velocity.P = cfg::velocityP;
  motor.PID_velocity.I = cfg::velocityI;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = cfg::velocityFilterSeconds;
  motor.sensor_direction = Direction::UNKNOWN;
  motor.zero_electric_angle = NOT_SET;
  motor.pp_check_result = false;

  Serial.println("CALIBRATION | El rotor puede moverse. Corte fisico accesible.");
  sensor.update();
  int initialized = 0;
  if (!calibrationInterrupted()) {
    enableRequested = true;
    initialized = motor.init();  // En 2.4.0 esta llamada también activa EN.
    sensor.update();
  }
  const int aligned = initialized == 1 && !calibrationInterrupted()
                          ? motor.initFOC() : 0;
  deenergize();
  pollSerial();
  const bool alignmentAccepted = aligned == 1 && !calibrationInterrupted();
  Serial.printf("CALIBRATION | motor.init=%d initFOC=%s pares=%s\n", initialized,
                alignmentAccepted ? "OK" : "sin exito",
                alignmentAccepted && motor.pp_check_result ? "OK" : "sin validar");

  if (calibrationSensorFault) {
    calibration = Calibration::Failed;
    failSafe(lastFault);
  } else if (calibrationAborted) {
    calibration = Calibration::Aborted;
    failSafe("Calibracion abortada por consola");
  } else if (aligned != 1 || !motor.pp_check_result ||
             !isfinite(motor.zero_electric_angle) || motor.zero_electric_angle == NOT_SET) {
    calibration = Calibration::Failed;
    if (initialized != 1) failSafe("Fallo motor.init()");
    else if (aligned != 1) failSafe("Fallo motor.initFOC(): revisar movimiento y cableado");
    else failSafe("Alineacion rechazada: pares de polos o cero electrico");
  } else {
    calibration = Calibration::Passed;
    state = State::READY;
    Serial.printf("READY | FOC OK, pares=%d, sentido=%s, cero=%.4f rad, EN bajo. START autoriza.\n",
                  cfg::polePairs, motor.sensor_direction == Direction::CW ? "CW" : "CCW",
                  motor.zero_electric_angle);
  }
}

bool mayStartControl(bool automatic) {
  if (state != State::READY || calibration != Calibration::Passed || !sensor.healthy()) {
    Serial.printf("%s requiere READY, FOC OK y encoder valido.\n", automatic ? "TEST_SPEED" : "START");
    return false;
  }
  if (automatic && speedTestTarget > cfg::speedLimit) {
    Serial.println("TEST_SPEED excede el limite de velocidad configurado.");
    return false;
  }
  return true;
}

void startControl(bool automatic) {
  if (!mayStartControl(automatic)) return;
  deenergize();
  if (automatic) {
    speedTest = {};
    speedTest.initialAngle = sensor.getAngle();
    speedTest.phase = SpeedTestPhase::InitialHold;
    speedTest.startedAt = micros();
  }
  state = State::MOTOR_CONTROL;
  motor.enable();
  enableRequested = true;
  lastControlAt = micros();
  lastAuthorizationAt = millis();
  if (!automatic) {
    Serial.printf("MOTOR_CONTROL | objetivo 0. Renovar con V <rad/s> o KEEP antes de %lu ms.\n",
                  static_cast<unsigned long>(cfg::commandTimeoutMs));
  }
}

void updateSpeedTestMeasurements(uint32_t elapsedUs) {
  const float direction = static_cast<float>(motor.sensor_direction);
  const float speed = direction * sensor.getVelocity();
  speedTest.measuredUs = elapsedUs;
  speedTest.displacement = direction * (sensor.getAngle() - speedTest.initialAngle);
  speedTest.peakSpeed = fmaxf(speedTest.peakSpeed, fabsf(speed));
  if (elapsedUs < speedTest.nextReportUs) return;
  speedTest.nextReportUs = (elapsedUs / speedTestReportUs + 1) * speedTestReportUs;

  const char* phase = "cero_final";
  if (speedTest.phase == SpeedTestPhase::InitialHold) phase = "cero_inicial";
  else if (speedTest.phase == SpeedTestPhase::Run) phase = "velocidad";
  char message[176];
  const int length = snprintf(message, sizeof(message),
                              "TEST_SPEED | t=%lu ms fase=%s obj=%.2f rampa=%.2f vel=%.3f rad/s "
                              "ang=%.2f deg PWM=%s EN=%s\n",
                              static_cast<unsigned long>(elapsedUs / 1000), phase, requestedSpeed, rampedSpeed,
                              speed, sensor.rawCount * (360.0f / (1 << cfg::encoderBits)),
                              driverReady ? "listo" : "pendiente", enableRequested ? "HIGH" : "LOW");
  if (length > 0 && length < static_cast<int>(sizeof(message)) && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(message), length);
  }
}

void updateControl() {
  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlAt;
  lastControlAt = nowUs;
  if (elapsedUs > cfg::maxControlGapUs) {
    calibration = Calibration::Pending;
    failSafe("Intervalo de control excedido");
    return;
  }
  const uint32_t testElapsedUs = nowUs - speedTest.startedAt;
  if (speedTest.active()) {
    if (testElapsedUs >= speedTestEndAtUs) {
      if (rampedSpeed != 0) {
        calibration = Calibration::Pending;
        failSafe("TEST_SPEED: plazo agotado antes de completar la rampa");
      } else {
        deenergize();
        state = State::READY;
        reportSpeedTestResult("secuencia completada");
      }
      return;
    }
    if (testElapsedUs >= speedTestStopAtUs) speedTest.phase = SpeedTestPhase::FinalHold;
    else if (testElapsedUs >= speedTestRunAtUs) speedTest.phase = SpeedTestPhase::Run;
    requestedSpeed = speedTest.phase == SpeedTestPhase::Run ? speedTestTarget : 0;
  } else if (!timeoutStopping && millis() - lastAuthorizationAt >= cfg::commandTimeoutMs) {
    requestedSpeed = 0;
    timeoutStopping = true;
    lastFault = "Timeout de comandos";
  }

  const float maxStep = cfg::acceleration * (elapsedUs * 1e-6f);
  const float error = requestedSpeed - rampedSpeed;
  rampedSpeed += constrain(error, -maxStep, maxStep);
  motor.target = rampedSpeed;
  if (timeoutStopping && rampedSpeed == 0) {
    deenergize();
    state = State::READY;
    Serial.println("READY | timeout: objetivo 0, EN bajo. START para rearmar.");
    return;
  }

  motor.loopFOC();
  if (!sensor.healthy()) {
    calibration = Calibration::Pending;
    failSafe(lastFault);
    return;
  }
  motor.move();
  if (speedTest.active()) updateSpeedTestMeasurements(testElapsedUs);
}

void report(bool periodic) {
  char angle[16] = "N/D";
  char raw[8] = "N/D";
  char speed[16] = "N/D";
  if (sensor.goodSamples > 0) {
    snprintf(angle, sizeof(angle), "%.2f", sensor.rawCount * (360.0f / (1 << cfg::encoderBits)));
    snprintf(raw, sizeof(raw), "%d", sensor.rawCount);
  }
  if (sensor.healthy()) snprintf(speed, sizeof(speed), "%.3f", sensor.getVelocity());

  char message[416];
  const int length = snprintf(
      message, sizeof(message),
      "%s | ang=%s deg bruto=%s vel=%s rad/s obj=%.2f rampa=%.2f | "
      "bus=%.2f V lim motor=%.2f V driver=%.2f V alin=%.2f V | PWM=%s EN pedido=%s | FOC=%s | "
      "giro=%s paridad=%lu EF=%lu diag=%lu salto=%lu regDiag=0x%04X regErr=%d | ultimo=%s\n",
      stateName(), angle, raw, speed, requestedSpeed, rampedSpeed, cfg::supplyVolts,
      cfg::motorLimitVolts, cfg::driverLimitVolts, cfg::alignVolts,
      driverReady ? "listo" : "pendiente", enableRequested ? "HIGH" : "LOW",
      calibrationName(), sensor.manualTravelSeen ? "si" : "pendiente",
      static_cast<unsigned long>(sensor.parityErrors), static_cast<unsigned long>(sensor.flagErrors),
      static_cast<unsigned long>(sensor.diagnosticErrors), static_cast<unsigned long>(sensor.jumpErrors),
      static_cast<unsigned>(sensor.diagnosticRegister), sensor.errorRegister, lastFault);
  if (length > 0 && length < static_cast<int>(sizeof(message)) &&
      (!periodic || Serial.availableForWrite() >= length)) {
    Serial.write(reinterpret_cast<const uint8_t*>(message), length);
  }
}

void printCommandMessage(const char* message) {
  // Durante TEST_SPEED, una consola saturada no puede retrasar el lazo FOC.
  if (!speedTest.active() || Serial.availableForWrite() >= static_cast<int>(strlen(message))) {
    Serial.print(message);
  }
}

void printHelp() {
  printCommandMessage("HELP | STATUS | SPI_BOOT | SENSOR | INIT | CALIBRATE | CONFIRM | START\n"
                      "V <rad/s> | KEEP | TEST_SPEED | STOP | DISABLE\n"
                      "Manual: START, V valido o KEEP autorizan 5 s; al caducar, rampa a cero.\n"
                      "Test: desde READY con FOC OK, TEST_SPEED y luego CONFIRM. Dura 7 s.\n"
                      "Para empezar: salida KORAD apagada, SENSOR y giro manual.\n");
}

void requestConfirmation(Confirmation action) {
  pendingConfirmation = action;
  confirmationAt = millis();
  Serial.printf("%s | CONFIRM antes de %.1f s solo si el motor esta sujeto, libre, sin rueda ni carga,\n"
                "la fuente esta correctamente configurada y ha activado su salida intencionadamente.\n",
                action == Confirmation::Calibration ? "CALIBRATE" : "TEST_SPEED",
                cfg::confirmationTimeoutMs * 0.001);
}

void setSpeed(const char* argument) {
  char* end = nullptr;
  const float speed = strtof(argument, &end);
  if (end == argument || *end != '\0' || !isfinite(speed) || fabsf(speed) > cfg::speedLimit) {
    Serial.printf("Velocidad invalida. Rango: +/-%.2f rad/s.\n", cfg::speedLimit);
    return;
  }
  requestedSpeed = speed;
  lastAuthorizationAt = millis();
}

void handleCommand(char* command) {
  while (*command == ' ' || *command == '\t') ++command;
  char* end = command + strlen(command);
  while (end > command && (end[-1] == ' ' || end[-1] == '\t')) *--end = '\0';
  if (*command == '\0') return;
  for (char* c = command; *c; ++c) *c = toupper(static_cast<unsigned char>(*c));

  const bool stop = strcmp(command, "STOP") == 0;
  const bool disable = strcmp(command, "DISABLE") == 0;
  if (state == State::CALIBRATION) {
    if (stop || disable) {
      calibrationAborted = true;
      cutCalibrationPower();
    }
    return;
  }
  if (stop || disable) {
    deenergize();
    if (disable) {
      calibration = Calibration::Pending;
      state = State::SAFE;
    } else if (state == State::MOTOR_CONTROL) {
      state = State::READY;
    }
    reportSpeedTestResult(disable ? "DISABLE" : "STOP");
    Serial.printf("%s | objetivo 0, EN bajo.\n", stateName());
  } else if (strcmp(command, "HELP") == 0) {
    printHelp();
  } else if (strcmp(command, "STATUS") == 0) {
    report(speedTest.active());
  } else if (speedTest.active()) {
    printCommandMessage("TEST_SPEED activo. Use STOP o DISABLE para interrumpir.\n");
  } else if (strcmp(command, "SPI_BOOT") == 0 && state != State::MOTOR_CONTROL) {
    if (sensorBusReady) sensor.printStartupTrace();
    else Serial.println("SPI sin inicializar.");
  } else if (strcmp(command, "SENSOR") == 0 && state != State::MOTOR_CONTROL) {
    if (!sensorBusReady) {
      Serial.println("SPI sin inicializar. Revise el bus y reinicie.");
      return;
    }
    deenergize();
    calibration = Calibration::Pending;
    sensor.resetManualTest();
    state = State::SENSOR_TEST;
    Serial.printf("SENSOR_TEST | EN bajo. Gire el rotor a mano al menos %.1f grados.\n",
                  cfg::manualTravelRad * (360.0f / _2PI));
  } else if (strcmp(command, "INIT") == 0) {
    prepareDriver();
  } else if (strcmp(command, "CALIBRATE") == 0) {
    if (mayCalibrate()) requestConfirmation(Confirmation::Calibration);
  } else if (strcmp(command, "TEST_SPEED") == 0) {
    if (mayStartControl(true)) requestConfirmation(Confirmation::SpeedTest);
  } else if (strcmp(command, "CONFIRM") == 0) {
    const Confirmation action = pendingConfirmation;
    pendingConfirmation = Confirmation::None;
    if (action == Confirmation::None || millis() - confirmationAt >= cfg::confirmationTimeoutMs) {
      Serial.println("Confirmacion ausente o caducada. Use CALIBRATE o TEST_SPEED.");
    } else if (action == Confirmation::SpeedTest) {
      startControl(true);
    } else if (mayCalibrate()) {
      runCalibration();
    }
  } else if (strcmp(command, "START") == 0) {
    startControl(false);
  } else if ((command[0] == 'V' && (command[1] == ' ' || command[1] == '\t')) ||
             strcmp(command, "KEEP") == 0) {
    if (state != State::MOTOR_CONTROL || timeoutStopping) {
      Serial.println("Autorizacion inactiva. Use START desde READY con FOC OK.");
    } else if (command[0] == 'V') {
      setSpeed(command + 2);
    } else {
      lastAuthorizationAt = millis();
    }
  } else {
    Serial.println("Comando no disponible. Use HELP.");
  }
}

void pollSerial() {
  static char line[80];
  static size_t used = 0;
  static bool discard = false;
  for (unsigned budget = 0; budget < sizeof(line) && Serial.available(); ++budget) {
    const int c = Serial.read();
    if (c == '\r' || c == '\n') {
      const bool valid = !discard && used != 0;
      if (!discard) line[used] = '\0';
      else if (state != State::CALIBRATION) printCommandMessage("Linea serie invalida o demasiado larga.\n");
      used = 0;
      discard = false;
      if (valid) {
        handleCommand(line);
        return;
      }
    } else if (!discard) {
      if ((c != '\t' && (c < 32 || c > 126)) || used == sizeof(line) - 1) discard = true;
      else line[used++] = static_cast<char>(c);
    }
  }
}
}  // namespace

void setup() {
  gpio_set_level(static_cast<gpio_num_t>(cfg::driverEnable), 0);
  pinMode(cfg::driverEnable, OUTPUT);
  lowerEnable();
  // Precarga el latch del GPIO antes de habilitar la salida, con CS inactivo.
  gpio_set_level(static_cast<gpio_num_t>(cfg::encoderCs), 1);
  pinMode(cfg::encoderCs, OUTPUT);
  Serial.setTxBufferSize(1024);
  Serial.begin(cfg::serialBaud);
  SimpleFOCDebug::enable(&Serial);
  Serial.println("\nGM4108_SimpleFOC_Test | SAFE | EN bajo, objetivo cero.");
  sensorBusReady = SPI.begin(cfg::encoderSck, cfg::encoderMiso, cfg::encoderMosi, -1);
  if (sensorBusReady) sensor.init();
  else failSafe("Fallo SPI.begin()");
  printHelp();
}

void loop() {
  // Comprueba caducidad y latencia antes de aceptar otra autorización.
  if (state == State::MOTOR_CONTROL) updateControl();
  const uint32_t now = millis();
  if (sensorBusReady && state != State::MOTOR_CONTROL && now - lastSensorAt >= cfg::sensorPeriodMs) {
    lastSensorAt = now;
    sensor.update();
    sensor.printStartupTraceIfReady();
    if (state == State::READY && !sensor.healthy()) {
      calibration = Calibration::Pending;
      failSafe(lastFault);
    }
  }
  pollSerial();

  const uint32_t interval = state == State::SENSOR_TEST ? cfg::diagnosticIntervalMs : cfg::monitorIntervalMs;
  if (!speedTest.active() && millis() - lastReportAt >= interval) {
    lastReportAt = millis();
    report(true);
  }
}
