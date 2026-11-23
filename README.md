# GM4108 con SimpleFOCWebController

Firmware exclusivo para diagnosticar la comunicación USB con SimpleFOCWebController en un ESP32-DevKitC con ESP32-WROOM-32E, un encoder AS5048A y un banco con iPower GM4108 y SimpleFOC Mini v2.3. Usa Commander nativo de Arduino SimpleFOC 2.4.0. La etapa de potencia permanece deshabilitada.

## Motor y caracterización

| Parámetro | Valor | Procedencia |
|---|---|---|
| Tipo y geometría | BLDC gimbal, 24 dientes, 22 polos | Datos aportados del GM4108 |
| Pares de polos | 11 | Confirmado por la calibración anterior |
| Resistencia entre terminales | 9,5 Ω | A–B, B–C y A–C dieron 9,6 Ω. Se descontaron 0,1 Ω de las puntas |
| Topología | Estrella (Wye) | Información de bobinado aportada. La topología se toma como dato de partida |
| Resistencia por bobinado | 4,75 Ω | En estrella se miden dos bobinados en serie. 9,5 Ω / 2 = 4,75 Ω |
| Kv de referencia | 27–30 RPM/V, nominal ≈28 RPM/V | Orientativo de la variante GM4108H-120T, pendiente de identificar en este motor |
| Kt de referencia | 0,32–0,35 Nm/A | Orientativo de la 120T y dependiente de las convenciones de corriente y tensión |
| Inductancia de referencia | 1,5–3,5 mH | Rango orientativo sin medición de Ld ni Lq |

El multímetro indicó `OL` entre cada terminal y la carcasa. La conexión estrella procede de la información aportada sobre el bobinado y queda pendiente de verificación por inspección interna. El valor típico aportado de 11,1 Ω entre terminales para ciertas variantes cede ante la medición corregida de este motor. [Guía de medición de resistencia de fase](https://docs.simplefoc.com/phase_resistance)

En el código de **2.4.0**, `BLDCMotor(11, 4.75f)` guarda el segundo argumento en `phase_resistance` y sigue inicializando `torque_controller` en `voltage`. El firmware también selecciona ese modo explícitamente. En esa rama de `loopFOC()`, la orden del PID de velocidad se acota por `motor.voltage_limit` en voltios. La resistencia no participa en esa conversión, en el límite de tensión ni en el cálculo de corriente. La rama independiente `estimated_current` sí usa la resistencia para calcular la tensión a partir de la consigna de corriente. Kv, Kt e inductancia quedan como referencias y se omiten de la configuración de control. [Constructor 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/BLDCMotor.cpp) · [Lazos y límites 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.cpp) · [Modo tensión](https://docs.simplefoc.com/voltage_torque_mode) · [Modo de corriente estimada](https://docs.simplefoc.com/estimated_current_mode)

## Cableado

| Origen | Destino |
|---|---|
| AS5048A VCC / GND | ESP32 3V3 / GND |
| AS5048A SCK / MISO / MOSI / CS | GPIO18 / GPIO19 / GPIO23 / GPIO27 |
| Mini IN1 / IN2 / IN3 / EN / GND | GPIO25 / GPIO26 / GPIO32 / GPIO33 / GND |
| Mini M1 / M2 / M3 | Motor fase A / B / C |
| KORAD positivo / negativo | Alimentación de potencia del Mini + / − |

Alimenta el ESP32 por USB y comparte masa entre el ESP32, el encoder, el Mini y el negativo de la fuente. El AS5048A usa el preset oficial `AS5048_SPI`, modo 1 a 1 MHz, 14 bits y registro angular `0x3FFF`. EN tiene un pull-down externo de 10 kΩ. Verifica su nivel en el Mini durante arranque y reset con la salida de potencia desconectada. Deja CS1, CS2, CS3 y la salida de 3,3 V del Mini sin conectar. [Cableado del Mini v2.3](https://docs.simplefoc.com/mini_v23_connect_hardware) · [Preset SPI 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/sensors/MagneticSensorSPI.cpp)

## PlatformIO

`platformio.ini` usa `espressif32@8.0.0`, placa `esp32dev`, framework Arduino, `askuric/Simple FOC@2.4.0` y puerto serie a 115200 baudios.

```sh
pio run -e esp32dev
pio run -e esp32dev -t upload
```

La segunda orden carga por USB. Mantén la salida de la KORAD apagada durante la carga y las pruebas.

## Diagnóstico con la etapa de potencia deshabilitada

GPIO33 (EN) se configura como salida de drenador abierto a nivel bajo. El pull-down externo de 10 kΩ mantiene EN bajo durante el reset, antes de que el firmware configure el pin. Las entradas PWM del Mini conservan su cableado, pero el firmware no las inicializa. El objeto de driver que usa SimpleFOC conserva los valores nominales del banco en memoria y carece de acceso a los pines de potencia.

La compilación exige `alignOnBoot = false`. El arranque inicializa el encoder SPI y prescinde de `motor.init()` y de la alineación FOC. Las solicitudes de realineación de Commander tampoco ejecutan la rutina de alineación. `ME1` puede cambiar el estado lógico que muestra WebController, aunque EN permanece bajo y las salidas PWM siguen inactivas.

El motor empieza en modo de velocidad con par por tensión. Los valores de arranque se conservan como referencia para las consultas.

| Parámetro | Valor inicial |
|---|---:|
| Tensión nominal de alimentación | 8,0 V |
| Límite de tensión del driver | 3,2 V |
| Límite de tensión del motor | 1,5 V |
| Tensión de alineación | 1,5 V |
| Límite de velocidad | 1,0 rad/s |
| PID de velocidad P / I / D | 0,20 / 0,10 / 0 |
| Filtro de velocidad Tf | 0,05 s |
| Rampa de salida del PID | 2,0 V/s |

## Comprobación por USB

El bucle llama directamente a `motor.monitor()` y `command.run()`. Commander recibe el puerto serie y despacha el comando `M` directamente a `command.motor(&motor, cmd)`. WebController termina sus envíos con LF. Al conectar, configura cinco decimales con `#5` y el modo legible por máquina con `@3`. Los cambios enviados al motor operan en memoria y se pierden al reiniciar. [Ejemplo oficial de WebController](https://docs.simplefoc.com/webcontroller#arduino-code) · [Envío serie](https://github.com/simplefoc/simplefoc-webcontroller/blob/8833df1004b50cdb3a970becc3d0b1b7411556e8/src/simpleFoc/serial.ts#L128-L131) · [Inicialización de la conexión](https://github.com/simplefoc/simplefoc-webcontroller/blob/8833df1004b50cdb3a970becc3d0b1b7411556e8/src/components/SerialManager.tsx#L76-L87)

1. Con la KORAD apagada y el monitor serie de PlatformIO cerrado, abre [SimpleFOCWebController](https://webcontroller.simplefoc.com/) en un navegador con Web Serial y selecciona el ESP32 a **115200** baudios.
2. En la terminal serie del navegador comprueba los envíos iniciales `#5` y `@3`, con respuestas `#5` y `@machine`. La detección debe mostrar el motor **M / motor**.
3. En una conexión recién iniciada, consulta los valores que aparecen en la tabla.
4. En **Monitoring**, selecciona **Angle** y **Velocity**, fija **Downsample** en 100 y pulsa **Start**. Gira el rotor a mano y comprueba que ambas trazas cambian. `MMG6` y `MMG5` permiten consultar el ángulo y la velocidad respectivamente. El ángulo usa el sentido bruto del encoder.
5. Comprueba físicamente el nivel de EN durante el arranque y la sesión. Los valores de tensión o de corriente de la interfaz carecen de una medición de fase en este diagnóstico.

| Comando | Respuesta esperada al arrancar |
|---|---|
| `?` | `?M:motor` |
| `ME` | `ME0` |
| `MVP` | `MVP0.20000` |
| `MVI` | `MVI0.10000` |
| `MVD` | `MVD0.00000` |
| `MVF` | `MVF0.05000` |

La detección, las consultas y la monitorización en el navegador quedan pendientes de comprobar con el ESP32 por USB.
