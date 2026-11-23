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
| Límite de consigna Velocity y umbral de sobrevelocidad | 1,0 rad/s |
| PID de velocidad P / I / D | 0,20 / 0,10 / 0 |
| Rampa del PID / LPF de velocidad Tf | 2,0 V/s / 0,05 s |
| PWM | 25 kHz |

Los 12 V son la alimentación usada para calcular el ciclo PWM. La inicialización conserva los límites de 3 V y la alineación de 1,5 V. El firmware acota la consigna a ±1 rad/s como máximo en Velocity y a ±3 V en Torque Voltage. En ambos modos `loopFOC()` acota la orden Q a ±3 V con `motor.voltage_limit` y compensación de tensión cero. El driver acota cada orden de fase a 0–3 V. Con modulación sinusoidal centrada en 1,5 V, alguna fase puede recortarse cuando la orden Q supera 1,5 V en magnitud. [Lazo FOC en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.cpp) · [Inicialización y modulación de BLDCMotor](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/BLDCMotor.cpp) · [Límite por fase del driver](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/drivers/BLDCDriver3PWM.cpp)

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
| `MLU` | Límite fijo de tensión Q (3 V) |
| `MVP`, `MVI`, `MVD`, `MVF` | P, I, D y LPF de velocidad. Ajustables desde Studio |
| `MMG0`, `MMG1`, `MMG5`, `MMG6` | Consigna (según modo), tensión Q calculada (V), velocidad (rad/s) y ángulo (rad) |

La monitorización arranca detenida. En el gráfico, selecciona **Target**, **Voltage Q**, **Velocity** y **Angle**, fija **Downsample** en **1** y pulsa **Start**. Para diagnóstico basta con **Velocity** y **Angle**. Las filas las emite `motor.monitor()`, separadas por tabuladores y con ambos delimitadores en `'\0'`. El firmware admite esas cuatro columnas, hasta seis decimales y una oportunidad de emisión cada 20 ms si hay espacio en el TX. **Downsample** divide esa cadencia. Por ejemplo, con el 100 predeterminado de Studio sale como máximo una muestra cada dos segundos. Deja las casillas de corriente y tensión D desmarcadas para que las columnas esperadas por Studio coincidan con las emitidas. Las consultas nativas de corriente siguen disponibles y sus campos carecen de datos de medición. [Formato nativo de monitorización](https://docs.simplefoc.com/monitoring)

## Validación física pendiente

### A. USB y encoder con la KORAD apagada

1. Arranca el ESP32 y comprueba EN en el conector del Mini durante reset, arranque y conexión de Studio.
2. Consulta `MC`, `MT` y `MLU`. Prueba `MC0`, `MC1` y un modo fuera de los dos admitidos, como `MC2`. Comprueba en Studio el modo vigente y que EN sigue bajo. Gira el rotor a mano y comprueba ángulo y velocidad. Antes de calibrar se muestra el sentido bruto del encoder.
3. Envía `ME1` y `A1`. Comprueba `ME0` y verifica que EN sigue bajo. Reserva `C1` para la fase B.

### B. Calibración supervisada

1. Sujeta el motor firmemente, retira la rueda, deja el rotor libre y ten a mano el corte físico de la KORAD. Ajusta manualmente 12,0 V y el límite de entrada a un máximo de 1,5 A. Activa la salida de la fuente cuando el banco esté preparado.
2. Envía `C1` una sola vez desde la terminal. Durante esta orden `motor.init()` activa EN y `motor.initFOC()` mueve el rotor para medir la dirección y el cero eléctrico. La calibración es síncrona y durante esos segundos el corte físico es la vía de parada.
3. Espera el mensaje `FOC: CCW, 11 pares OK, motor deshabilitado.` y comprueba `ME0` y EN bajo. El firmware exige éxito de `initFOC()`, prueba de pares favorable, dirección CCW y cero eléctrico válido. La librería puede informar `No current sense.` durante una alineación correcta en modo tensión. Si falla la calibración, corta la potencia y revisa las conexiones antes de reiniciar. [Secuencia oficial de alineación](https://docs.simplefoc.com/bldcmotor#step-6-align-motor-and-all-the-sensors-field-oriented-control-init) · [Comprobación de pares en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.cpp)

### C. Primera prueba a 1 V de tensión Q

1. Tras validar A y B, abre el gráfico y envía `MC0` con el motor deshabilitado. Comprueba que la respuesta indica `torque` y envía `A1` y luego `ME1`. El plazo de 30 s empieza con `A1`. Consulta `ME` y `MMG0` para comprobar habilitación y consigna inicial cero.
2. Envía `M1.0` desde la terminal. **Target** debe representar 1,0 V de consigna Q. Observa **Voltage Q** y **Velocity**. Detén la prueba con `ME0`. `M0` pone la consigna a 0 V y mantiene el driver habilitado.
3. Comprueba que la autorización caduca aunque Studio siga enviando consultas. Al caducar baja EN y la consigna vuelve a cero. Repetir `ME1` o `A1` durante una autorización activa no prolonga el plazo. Para otra prueba, repite `A1` y `ME1` después de la parada.
4. Si el rotor acelera rápidamente, el umbral de 1,0 rad/s puede terminar la prueba antes de los 30 s. Ante oscilación fuerte, movimiento inesperado, calentamiento o corriente excesiva, corta físicamente la potencia.
5. Con el driver deshabilitado tras `ME0`, envía `MC1` y comprueba que la respuesta indica `vel`. Verifica que `ME1` por sí solo deja el driver deshabilitado. Para probar Velocity, envía una nueva secuencia `A1`, `ME1` y `M0.5`. Comprueba **Target** en rad/s y deshabilita con `ME0`.

## Parada, recuperación y límites de la protección

- `ME0` baja EN al procesarse, deshabilita el driver y cancela la autorización. Cerrar Studio o perder USB con el ESP32 aún alimentado deja correr el plazo de 30 s desde `A1`. La detección depende del bucle de firmware y del procesamiento serie. Usa el corte físico si falla la comunicación o el firmware deja de responder.
- Commander puede escribir parámetros públicos y llamar directamente a `enable()` e `initFOC()`. Se protegen esos puntos de entrada con los métodos virtuales de `BLDCMotor`, conservando el protocolo nativo. `ME1` exige calibración y autorización vigentes. `MFR` deshabilita y se queda sin acceso a la alineación con potencia. Commander responde `MFRdone` incluso cuando la función rechaza la orden. La secuencia supervisada se inicia con `C1`.
- Después de cada orden `M`, el firmware verifica el modo, la modulación, la dirección y el cero eléctrico, el límite fijo de 3 V y la validez de la consigna y de los parámetros del PID. Una escritura `MLU`, o un límite de tensión adjunto a una consigna Velocity, que cambie los 3 V restaura el límite y enclava un fallo antes del siguiente ciclo de control. La respuesta nativa a la escritura precede a la comprobación. Una consigna o realimentación no finita, una tensión Q fuera del intervalo, una calibración fallida o una velocidad filtrada superior a 1,0 rad/s también deshabilitan el driver y enclavan un fallo. Ante un fallo, corta potencia, corrige la causa y reinicia. El reset exige calibrar de nuevo y autorizar explícitamente el movimiento.
- La protección de sobrevelocidad usa la estimación del encoder con un filtro fijo de 0,05 s. El disparo depende del retardo y la resolución de la estimación. La lectura SPI nativa de 2.4.0 descarta el bit de error y la paridad de la respuesta, por lo que un encoder desconectado o congelado puede devolver una lectura finita. **Voltage Q** muestra la orden calculada. El firmware carece de medición y protección por corriente de fase. [Lectura del AS5048A en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/sensors/MagneticSensorSPI.cpp)
- La compilación comprueba el firmware para `esp32dev`. EN en el pin, compatibilidad física con Studio, calibración, estabilidad, respuesta de parada y comportamiento del sensor se verifican en banco antes de operar con carga.
