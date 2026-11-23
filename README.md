# GM4108 con SimpleFOCStudio

Firmware de banco para controlar un iPower GM4108 en **Velocity** o **Torque Voltage** con Arduino SimpleFOC 2.4.0, un ESP32-DevKitC con ESP32-WROOM-32E, un encoder AS5048A y un SimpleFOC Mini v2.3. Arranca en Velocity, con EN bajo y consigna cero. La calibración y la habilitación requieren órdenes independientes.

La etiqueta `diagnostico-studio-validado` conserva el firmware de diagnóstico validado en `c1d0f40`. Esa versión usa objetos de diagnóstico sin acceso al PWM. Para recuperar su código sin cambiar de rama de trabajo, usa `git worktree add --detach ../GM4108_diagnostico diagnostico-studio-validado`.

## Motor, cableado y alimentación

| Parámetro | Valor |
|---|---|
| Geometría | 24 dientes, 22 polos, 11 pares de polos |
| Resistencia entre terminales | 9,5 Ω, medida. A–B, B–C y A–C dieron 9,6 Ω, con 0,1 Ω de las puntas descontados |
| Bobinado | Estrella (Wye), según la información aportada |
| Resistencia por fase | 4,75 Ω. Dos bobinados en serie entre terminales, 9,5 Ω / 2 |
| Kv orientativo | ≈28 RPM/V para la variante 120T. Kv e inductancia quedan sin configurar |

El multímetro indicó `OL` entre cada terminal y la carcasa. `BLDCMotor(11, 4.75f)` registra la resistencia de fase. La combinación `MotionControlType::torque` y `TorqueControlType::voltage` usa la consigna de Commander directamente en voltios Q, sin intervención del PID de velocidad. [Constructor en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/BLDCMotor.cpp) · [Control por tensión](https://docs.simplefoc.com/voltage_torque_mode)

| Señal | Conexión |
|---|---|
| AS5048A VCC / GND | ESP32 3V3 / GND |
| AS5048A SCK / MISO / MOSI / CS | GPIO18 / GPIO19 / GPIO23 / GPIO27 |
| Mini IN1 / IN2 / IN3 / EN / GND | GPIO25 / GPIO26 / GPIO32 / GPIO33 / GND |
| Mini M1 / M2 / M3 | Fases A / B / C del motor |
| Potencia del Mini | Salida + / − de la KORAD, ajustada manualmente a 12,0 V y hasta 1,5 A |

Alimenta el ESP32 por USB y comparte masa con el encoder, el Mini y el negativo de la KORAD. EN es activo en alto y lleva un pull-down externo de 10 kΩ para el reset. Deja CS1, CS2, CS3 y la salida de 3,3 V del Mini sin conectar. El AS5048A usa `AS5048_SPI`, SPI modo 1 a 1 MHz, con 14 bits de resolución. Los ajustes de la KORAD son experimentales. Su límite de hasta 1,5 A corresponde a la corriente de entrada del Mini. El montaje carece de medición de corriente de fase. [Cableado del Mini v2.3](https://docs.simplefoc.com/mini_v23_connect_hardware) · [Configuración SPI en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/sensors/MagneticSensorSPI.cpp)

| Ajuste inicial | Valor |
|---|---:|
| Alimentación del Mini | 12,0 V |
| Límite del driver / límite del motor | 3,0 V / 3,0 V |
| Tensión de alineación | 1,5 V |
| Primera consigna Q manual | 1,0 V |
| Límite de consigna Velocity | 1,0 rad/s |
| Umbral independiente de sobrevelocidad | 10,0 rad/s, experimental |
| PID de velocidad P / I / D | 0,20 / 0,10 / 0 |
| Rampa del PID / LPF de velocidad Tf | 2,0 V/s / 0,05 s |
| PWM | 25 kHz |

Los 12 V son la alimentación usada para calcular el ciclo PWM. El firmware acota la consigna de Velocity a ±`MLV` y la de Torque Voltage a ±`MLU`. En ambos modos `loopFOC()` acota la orden Q a ±`MLU`, con un techo de 3 V y compensación de tensión cero. El driver mantiene su límite de 0–3 V por fase. Con modulación sinusoidal centrada en 1,5 V, alguna fase puede recortarse cuando la orden Q supera 1,5 V en magnitud. [Lazo FOC en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.cpp) · [Inicialización y modulación de BLDCMotor](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/BLDCMotor.cpp) · [Límite por fase del driver](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/drivers/BLDCDriver3PWM.cpp)

## Compilación y conexión USB

```sh
pio run -e esp32dev
```

Para cargar manualmente, con la salida de la KORAD apagada, ejecuta `pio run -e esp32dev -t upload`. El entorno fija `espressif32@8.0.0` y `askuric/Simple FOC@2.4.0`.

En SimpleFOCStudio con Python 3.9, configura `/dev/cu.usbserial-0001` a 115200 baudios y 8N1. Añade un dispositivo con ID `M` y cierra otros clientes del puerto. Envía las órdenes manuales desde la terminal integrada, con terminación LF. Commander procesa `M` en modo legible por máquina. [Uso y terminal de SimpleFOCStudio](https://github.com/JorgeMaker/SimpleFOCStudio#usage)

## Selección de modo

Envía `ME0` y selecciona **Torque** o **Velocity** en el selector de control de Studio. La terminal integrada admite las mismas órdenes nativas. Ambos modos usan `TorqueControlType::voltage`. [Códigos de modo en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.h) · [Selección en Commander 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/communication/Commander.cpp)

Para sincronizar la versión Python de Studio, envía `@2` desde su terminal tras conectar y pulsa **Pull Params**. Las consultas `MC` y `MT` actualizan los selectores. El parser de Studio espera las respuestas nativas `Motion:torque`, `Motion:vel` y `Torque:volt`. El firmware arranca en `@3`, que produce las respuestas compactas de la tabla. Puedes volver a `@3` para trabajar desde la terminal. La telemetría por tabuladores funciona con ambos ajustes. [Parser de Studio](https://github.com/JorgeMaker/SimpleFOCStudio/blob/master/src/simpleFOCConnector.py) · [Modos de respuesta de Commander](https://docs.simplefoc.com/commander_interface)

| Comando | Modo | Respuesta nativa | Unidad de Target | Ejemplo |
|---|---|---|---|---|
| `MC0` | Torque Voltage | `MCtorque` | V | `M1.0` solicita +1,0 V en Q |
| `MC1` | Velocity | `MCvel` | rad/s | `M0.5` solicita +0,5 rad/s |
| `MC` | Consulta del modo vigente | `MCtorque` o `MCvel` | Según el modo | |

El callback de Commander permite una transición únicamente con EN bajo, consigna cero y fuera de la calibración y de un fallo enclavado. Si rechaza una transición, devuelve la respuesta nativa del modo vigente. Una transición válida borra la salida previa, reinicia el estado del PID, conserva la calibración y cancela cualquier autorización. Para mover el motor en el nuevo modo, vuelve a enviar `A1` y `ME1`.

| Consulta | Parámetro |
|---|---|
| `ME` | Habilitación, inicialmente `ME0` |
| `MT` | Modo de par (`volt`) |
| `MVP`, `MVI`, `MVD`, `MVF` | P, I, D y LPF de velocidad. Ajustables desde Studio |
| `MMG0`, `MMG1`, `MMG5`, `MMG6` | Consigna (según modo), tensión Q calculada (V), velocidad (rad/s) y ángulo (rad) |

## Ajustes de banco y comandos

| Comando | Función | Estado requerido |
|---|---|---|
| `MC0` | Selecciona Torque Voltage | `ME0` y consigna cero |
| `MC1` | Selecciona Velocity | `ME0` y consigna cero |
| `MLV` / `MLV10` | Consulta o fija el límite de la consigna Velocity. Rango de 0 a 20 rad/s, inicial 1 rad/s | `ME0` para escribir |
| `S` / `S10` / `S15` | Consulta o fija el umbral de sobrevelocidad en ambos modos. Rango de 1 a 20 rad/s, inicial 10 rad/s | `ME0` para escribir |
| `MLU` / `MLU2` / `MLU3` | Consulta o fija el límite de tensión Q en ambos modos. Rango de 0 a 3 V, inicial 3 V | `ME0` para escribir |
| `MVP`, `MVI`, `MVD`, `MVR`, `MVF`, `MVL` con valor | Ajustan el PID y el LPF de velocidad. Los valores deben ser finitos, la rampa y Tf deben ser ≥0 y `MVL` admite de 0 a `MLU` | `ME0` para escribir |
| `C1` | Inicia la calibración supervisada a 1,5 V | `ME0` y `MLU` ≥1,5 V |
| `A1` | Autoriza movimiento por 30 s | Calibración válida y `ME0` |
| `ME1` | Habilita con consigna cero | Calibración y autorización vigentes |
| `ME0` | Baja EN y cancela la autorización | Cualquier estado que esté atendiendo Commander |
| `R` | Consulta estado, calibración, habilitación y fallo | Cualquier estado que esté atendiendo Commander |
| `R1` | Solicita recuperar un fallo recuperable | EN bajo, consigna cero, calibración vigente y reposo confirmado |

Usa los campos **Velocity Limit** y **Voltage Limit** de Studio o envía `MLV` y `MLU` desde la terminal integrada. `S` es un ID independiente de Commander. `MS…` sigue siendo el prefijo de ajustes del sensor dentro del ID `M`. Se admiten consultas durante el movimiento. Cada escritura aceptada cancela la autorización anterior y requiere un nuevo `A1` antes de `ME1`. Una escritura rechazada devuelve el valor vigente sin enclavar un fallo. Envía las consignas Velocity sin un límite de tensión adjunto y ajusta la tensión por separado con `MLU`. Los ajustes se conservan en RAM hasta el reinicio. [Comandos nativos de SimpleFOC](https://docs.simplefoc.com/commander_motor)

La monitorización arranca detenida. En el gráfico, selecciona **Target**, **Voltage Q**, **Velocity** y **Angle**, fija **Downsample** en **1** y pulsa **Start**. Para diagnóstico basta con **Velocity** y **Angle**. Las filas las emite `motor.monitor()`, separadas por tabuladores y con ambos delimitadores en `'\0'`. El firmware admite esas cuatro columnas, hasta seis decimales y una oportunidad de emisión cada 20 ms si hay espacio en el TX. **Downsample** divide esa cadencia. Por ejemplo, con el 100 predeterminado de Studio sale como máximo una muestra cada dos segundos. Deja las casillas de corriente y tensión D desmarcadas para que las columnas esperadas por Studio coincidan con las emitidas. Las consultas nativas de corriente siguen disponibles y sus campos carecen de datos de medición. [Formato nativo de monitorización](https://docs.simplefoc.com/monitoring)

## Validación física pendiente

### A. USB y encoder con la KORAD apagada

1. Arranca el ESP32 y comprueba EN en el conector del Mini durante reset, arranque y conexión de Studio.
2. Consulta `MC`, `MT`, `MLU`, `MLV` y `S`. Con el driver deshabilitado, comprueba que `MLU2`, `MLV10` y `S15` se aplican y que las escrituras fuera de rango conservan el valor previo. Restablece `MLU3`, `MLV1` y `S10` si quieres probar con los ajustes iniciales. Prueba `MC0` y `MC1`. Gira el rotor a mano y comprueba ángulo y velocidad. Antes de calibrar se muestra el sentido bruto del encoder.
3. Envía `ME1` y `A1`. Comprueba `ME0` y verifica que EN sigue bajo. Reserva `C1` para la fase B.

### B. Calibración supervisada

1. Sujeta el motor firmemente, retira la rueda, deja el rotor libre y ten a mano el corte físico de la KORAD. Ajusta manualmente 12,0 V y el límite de entrada a un máximo de 1,5 A. Activa la salida de la fuente cuando el banco esté preparado.
2. Envía `C1` una sola vez desde la terminal. Durante esta orden `motor.init()` activa EN y `motor.initFOC()` mueve el rotor para medir la dirección y el cero eléctrico. La calibración es síncrona y durante esos segundos el corte físico es la vía de parada.
3. Espera el mensaje `FOC: CCW, 11 pares OK, motor deshabilitado.` y comprueba `ME0` y EN bajo. El firmware exige éxito de `initFOC()`, prueba de pares favorable, dirección CCW y cero eléctrico válido. La librería puede informar `No current sense.` durante una alineación correcta en modo tensión. Si falla la calibración, corta la potencia y revisa las conexiones antes de reiniciar. [Secuencia oficial de alineación](https://docs.simplefoc.com/bldcmotor#step-6-align-motor-and-all-the-sensors-field-oriented-control-init) · [Comprobación de pares en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.cpp)

### C. Primera prueba a 1 V de tensión Q

1. Tras validar A y B, abre el gráfico y envía `MC0` con el motor deshabilitado. Comprueba que la respuesta indica `torque` y envía `A1` y luego `ME1`. El plazo de 30 s empieza con `A1`. Consulta `ME` y `MMG0` para comprobar habilitación y consigna inicial cero.
2. Envía `M1.0` desde la terminal. **Target** debe representar 1,0 V de consigna Q. Observa **Voltage Q** y **Velocity**. Detén la prueba con `ME0`. `M0` pone la consigna a 0 V y mantiene el driver habilitado.
3. Comprueba que la autorización caduca aunque Studio siga enviando consultas. Al caducar baja EN y la consigna vuelve a cero. Repetir `ME1` o `A1` durante una autorización activa no prolonga el plazo. Para otra prueba, repite `A1` y `ME1` después de la parada.
4. En Torque Voltage el rotor puede acelerar hasta disparar el umbral `S` antes de los 30 s. Ante oscilación fuerte, movimiento inesperado, calentamiento o corriente excesiva, corta físicamente la potencia.
5. Con el driver deshabilitado tras `ME0`, envía `MC1` y comprueba que la respuesta indica `vel`. Verifica que `ME1` por sí solo deja el driver deshabilitado. Para probar Velocity, envía una nueva secuencia `A1`, `ME1` y `M0.5`. Comprueba **Target** en rad/s y deshabilita con `ME0`.

## Parada y recuperación en la misma sesión

- `ME0` baja EN al procesarse, deshabilita el driver y cancela la autorización. Cerrar Studio o perder USB con el ESP32 aún alimentado deja correr el plazo de 30 s desde `A1`. La detección depende del bucle de firmware y del procesamiento serie. Usa el corte físico si falla la comunicación o el firmware deja de responder.
- Commander puede escribir parámetros públicos y llamar directamente a `enable()` e `initFOC()`. Se protegen esos puntos de entrada con los métodos virtuales de `BLDCMotor`, conservando el protocolo nativo. `ME1` exige calibración y autorización vigentes. `MFR` deshabilita y se queda sin acceso a la alineación con potencia. Commander responde `MFRdone` incluso cuando la función rechaza la orden. La secuencia supervisada se inicia con `C1`.
- Tras `ME0` o la caducidad, la consigna queda en cero y se conservan la calibración, el modo, el PID y los límites. Repite `A1` y `ME1` para la siguiente prueba en cualquiera de los dos modos.
- Tras un fallo, consulta `R` en la terminal de Studio. Por ejemplo, `FOC: estado=FAULT calibrated=true enabled=false fault=OVERSPEED`. Cuando el rotor se haya detenido, envía `R1`. Se exige consigna cero, EN bajo en el GPIO, parámetros válidos, una lectura nueva del encoder y al menos 500 ms con velocidad estimada de hasta 0,2 rad/s. Un hueco de muestreo superior a 20 ms interrumpe esa comprobación. Tras recuperar, `R` indica `READY` y el driver permanece deshabilitado. Envía una nueva secuencia `A1`, `ME1` y la consigna.
- `R1` permite limpiar `OVERSPEED`, una consigna inválida ya borrada (`TARGET`) o una configuración inválida ya corregida (`CONFIGURATION`). Las escrituras rechazadas por Commander conservan el valor anterior y permiten seguir operando. Los fallos `DRIVER`, `CALIBRATION`, `ENCODER`, `INTERNAL` y `ENABLE_PIN` son críticos. Corta la potencia, corrige la causa y reinicia para inicializar y calibrar bajo supervisión. Si cambia la posición relativa entre el motor y el imán del encoder, reinicia y calibra antes de volver a mover el motor.

## Límites de la protección

- La protección calcula la velocidad con diferencias de ángulo del AS5048A en intervalos de al menos 10 ms. Enclava tras tres intervalos consecutivos por encima de `S`. Esto descarta un pico aislado y añade un retardo del orden de decenas de milisegundos. El LPF de Studio sigue afectando al PID y a la gráfica. Los valores 10 y 20 rad/s son ajustes experimentales de este banco.
- El sondeo comprueba la paridad y el bit de error SPI de diagnóstico y ángulo, además del estado de compensación, CORDIC y campo magnético. SimpleFOC sigue leyendo el encoder para el lazo FOC. Un encoder bloqueado que siga emitiendo tramas coherentes o un imán desplazado pueden pasar el sondeo. EN se lee en el GPIO del ESP32. Verifica en banco el nivel en el Mini y el corte físico. Durante la calibración síncrona se comprueba el encoder antes y después. El firmware carece de medición y protección por corriente de fase. [Hoja de datos del AS5048A](https://look.ams-osram.com/m/287d7ad97d1ca22e/original/AS5048-DS000298%20Datasheet) · [Lectura de ángulo en SimpleFOC 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/sensors/MagneticSensorSPI.cpp)
- La compilación comprueba el firmware para `esp32dev`. EN en el pin, compatibilidad física con Studio, calibración, estabilidad, respuesta de parada y comportamiento del sensor se verifican en banco antes de operar con carga.
