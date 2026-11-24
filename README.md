# GM4108 con SimpleFOCStudio

Firmware de banco para controlar un iPower GM4108 en **Velocity**, **Torque Voltage** o **Angle** con Arduino SimpleFOC 2.4.0, un ESP32-DevKitC con ESP32-WROOM-32E, un encoder AS5048A y un SimpleFOC Mini v2.3. Arranca en Velocity, con EN bajo y consigna cero. La calibración y la habilitación requieren órdenes independientes.

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
| Límite del driver / límite del motor | 4,0 V / 4,0 V |
| Tensión de alineación | 1,5 V |
| Primera consigna Q manual | 1,0 V |
| Límite inicial de velocidad para Velocity y Angle | 5,0 rad/s |
| Ganancia P inicial de Angle | 10 |
| Umbral independiente de sobrevelocidad | 40,0 rad/s (≈382 rpm), experimental |
| PID de velocidad P / I / D | 0,20 / 0,10 / 0 |
| Rampa del PID / LPF de velocidad Tf | 2,0 V/s / 0,05 s |
| PWM | 25 kHz |

Los 12 V son la alimentación usada para calcular el ciclo PWM. El firmware acota la consigna de Velocity a ±`MLV` y la de Torque Voltage a ±`MLU`. En Angle, la consigna es una posición en radianes y `MLV` limita la velocidad que solicita el controlador en cascada. `loopFOC()` acota la orden Q a ±`MLU` en los tres modos, con un techo de 4 V y compensación de tensión cero. El driver mantiene su límite de 0–4 V por fase. Con modulación sinusoidal centrada en 2 V, alguna fase puede recortarse cuando la orden Q supera 2 V en magnitud. En reposo, 4 V / 4,75 Ω da una referencia teórica de 0,84 A por fase. La KORAD limita la corriente de alimentación a 1,5 A mediante su ajuste físico. [Lazo FOC en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.cpp) · [Inicialización y modulación de BLDCMotor](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/BLDCMotor.cpp) · [Límite por fase del driver](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/drivers/BLDCDriver3PWM.cpp)

## Compilación y conexión USB

```sh
pio run -e esp32dev
```

Para cargar manualmente, con la salida de la KORAD apagada, ejecuta `pio run -e esp32dev -t upload`. El entorno fija `espressif32@8.0.0` y `askuric/Simple FOC@2.4.0`.

En SimpleFOCStudio con Python 3.9, configura `/dev/cu.usbserial-0001` a 115200 baudios y 8N1. Añade un dispositivo con ID `M` y cierra otros clientes del puerto. Envía las órdenes manuales desde la terminal integrada, con terminación LF. Commander procesa `M` en modo legible por máquina. [Uso y terminal de SimpleFOCStudio](https://github.com/JorgeMaker/SimpleFOCStudio#usage)

## Selección de modo

Envía `ME0` y selecciona **Torque**, **Velocity** o **Angle** en el selector de control de Studio. La terminal integrada admite las mismas órdenes nativas. Los tres modos usan `TorqueControlType::voltage`. [Códigos de modo en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.h) · [Selección en Commander 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/communication/Commander.cpp)

Para sincronizar la versión Python de Studio, envía `@2` desde su terminal tras conectar y pulsa **Pull Params**. Las consultas `MC` y `MT` actualizan los selectores. El parser de Studio espera las respuestas nativas `Motion:torque`, `Motion:vel`, `Motion:angle` y `Torque:volt`. El firmware arranca en `@3`, que produce las respuestas compactas de la tabla. Puedes volver a `@3` para trabajar desde la terminal. La telemetría por tabuladores funciona con ambos ajustes. [Parser de Studio](https://github.com/JorgeMaker/SimpleFOCStudio/blob/master/src/simpleFOCConnector.py) · [Modos de respuesta de Commander](https://docs.simplefoc.com/commander_interface)

| Comando | Modo | Respuesta nativa | Unidad de Target | Ejemplo |
|---|---|---|---|---|
| `MC0` | Torque Voltage | `MCtorque` | V | `M1.0` solicita +1,0 V en Q |
| `MC1` | Velocity | `MCvel` | rad/s | `M0.5` solicita +0,5 rad/s |
| `MC2` | Angle | `MCangle` | rad | `M0.524` solicita +0,524 rad |
| `MC` | Consulta del modo vigente | `MCtorque`, `MCvel` o `MCangle` | Según el modo | |

El callback de Commander permite una transición únicamente con EN bajo, consigna cero y fuera de la calibración y de un fallo enclavado. Si rechaza una transición, devuelve la respuesta nativa del modo vigente. Una transición válida borra la salida previa, reinicia el estado del PID, conserva la calibración y cancela cualquier autorización. Para mover el motor en el nuevo modo, vuelve a enviar `A1` y `ME1`.

En Angle, **Target** admite posiciones positivas y negativas de varias vueltas respecto al origen relativo. Una vuelta equivale a 2π rad. `MLV` y `MLU` limitan la velocidad solicitada por el lazo de posición y la tensión Q, respectivamente.

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
| `MC2` | Selecciona Angle | `ME0` y consigna cero |
| `MLV` / `MLV40` | Consulta o fija el límite de velocidad para Velocity y Angle. Rango de 0 a 40 rad/s, inicial 5 rad/s | `ME0` para escribir |
| `S` / `S30` / `S40` / `S50` | Consulta o fija el umbral de sobrevelocidad para los tres modos. Rango de 1 a 60 rad/s, inicial 40 rad/s | `ME0` para escribir |
| `MLU` / `MLU4` | Consulta o fija el límite de tensión Q para los tres modos. Rango de 0 a 4 V, inicial 4 V | `ME0` para escribir |
| `MAP` / `MAP10` | Consulta o fija la ganancia del lazo de posición. `MAL` permite fijar su límite de salida hasta `MLV` | `ME0` para escribir |
| `Z1` | Fija el cero mecánico relativo en la posición actual | Calibración válida y `ME0` |
| `MVP`, `MVI`, `MVD`, `MVR`, `MVF`, `MVL` con valor | Ajustan el PID y el LPF de velocidad. Los valores deben ser finitos, la rampa y Tf deben ser ≥0 y `MVL` admite de 0 a `MLU` | `ME0` para escribir |
| `C1` | Inicia la calibración supervisada a 1,5 V | `ME0` y `MLU` ≥1,5 V |
| `A1` | Autoriza movimiento por 300 s | Calibración válida y `ME0` |
| `ME1` | Habilita con consigna cero en Torque Voltage y Velocity, o con la posición actual en Angle | Calibración y autorización vigentes |
| `ME0` | Baja EN y cancela la autorización | Cualquier estado que esté atendiendo Commander |
| `R` | Consulta estado, calibración, habilitación y fallo | Cualquier estado que esté atendiendo Commander |
| `R1` | Solicita recuperar un fallo recuperable | EN bajo, consigna cero, calibración vigente y reposo confirmado |

Usa los campos **Velocity Limit** y **Voltage Limit** de Studio o envía `MLV` y `MLU` desde la terminal integrada. `S` es un ID independiente de Commander. `MS…` sigue siendo el prefijo de ajustes del sensor dentro del ID `M`. Se admiten consultas durante el movimiento. Cada escritura aceptada cancela la autorización anterior y requiere un nuevo `A1` antes de `ME1`. Una escritura rechazada devuelve el valor vigente sin enclavar un fallo. Envía las consignas Velocity y Angle sin límites adjuntos. Usa `MLU` y `MLV` para los límites. `Z1` cambia el origen angular mostrado y conserva el cero eléctrico de FOC. Los ajustes se conservan en RAM hasta el reinicio. [Comandos nativos de SimpleFOC](https://docs.simplefoc.com/commander_motor)

La monitorización arranca detenida. En el gráfico, selecciona **Target**, **Voltage Q**, **Velocity** y **Angle**, fija **Downsample** en **1** y pulsa **Start**. Para diagnóstico basta con **Velocity** y **Angle**. Las filas las emite `motor.monitor()`, separadas por tabuladores y con ambos delimitadores en `'\0'`. El firmware admite esas cuatro columnas, hasta seis decimales y una oportunidad de emisión cada 20 ms si hay espacio en el TX. **Downsample** divide esa cadencia. Por ejemplo, con el 100 predeterminado de Studio sale como máximo una muestra cada dos segundos. Deja las casillas de corriente y tensión D desmarcadas para que las columnas esperadas por Studio coincidan con las emitidas. Las consultas nativas de corriente siguen disponibles y sus campos carecen de datos de medición. [Formato nativo de monitorización](https://docs.simplefoc.com/monitoring)

## Validación física pendiente

### A. USB y encoder con la KORAD apagada

1. Arranca el ESP32 y comprueba EN en el conector del Mini durante reset, arranque y conexión de Studio.
2. Consulta `MC`, `MT`, `MLU`, `MLV` y `S`. Con el driver deshabilitado, comprueba `MLU4`, `MLV40`, `S30`, `S40` y `S50`. Verifica que `S0` y `S61` conservan el valor previo. Restablece `MLV5` y `S40` para los primeros ensayos. Prueba `MC0`, `MC1` y `MC2`. Gira el rotor a mano y comprueba ángulo y velocidad. Antes de calibrar se muestra el sentido bruto del encoder.
3. Envía `ME1` y `A1`. Comprueba `ME0` y verifica que EN sigue bajo. Reserva `C1` para la fase B.

### B. Calibración supervisada

1. Sujeta el motor firmemente, retira la rueda, deja el rotor libre y ten a mano el corte físico de la KORAD. Ajusta manualmente 12,0 V y el límite de entrada a un máximo de 1,5 A. Activa la salida de la fuente cuando el banco esté preparado.
2. Envía `C1` desde la terminal. Durante esta orden `motor.init()` activa EN y `motor.initFOC()` mueve el rotor para medir la dirección y el cero eléctrico. La calibración es síncrona y durante esos segundos el corte físico es la vía de parada.
3. Espera el mensaje `FOC: CCW, 11 pares OK, motor deshabilitado.` y comprueba `ME0` y EN bajo. El firmware exige éxito de `initFOC()`, dirección CCW, movimiento coherente y cero eléctrico válido. Mantiene 11 pares y calcula el error eléctrico con las dos lecturas del barrido de SimpleFOC 2.4.0. Acepta hasta 0,7 rad eléctricos y avisa si el control nativo de 0,5 rad había fallado. El firmware recoge las lecturas desde el sensor, sin cambiar el barrido ni los archivos instalados. Movimiento insuficiente o desplazamiento fuera de margen dejan EN bajo y permiten repetir `C1` tras revisar el rotor. Un fallo crítico exige corregir la causa y reiniciar. La librería puede informar `No current sense.` durante una alineación correcta. [Secuencia oficial de alineación](https://docs.simplefoc.com/bldcmotor#step-6-align-motor-and-all-the-sensors-field-oriented-control-init) · [Comprobación de pares en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.cpp)

### C. Primera prueba a 1 V de tensión Q

1. Tras validar A y B, abre el gráfico y envía `MC0` con el motor deshabilitado. Comprueba que la respuesta indica `torque` y envía `A1` y luego `ME1`. El plazo de 300 s empieza con `A1`. Consulta `ME` y `MMG0` para comprobar habilitación y consigna inicial cero.
2. Envía `M1.0` desde la terminal. **Target** debe representar 1,0 V de consigna Q. Observa **Voltage Q** y **Velocity**. Detén la prueba con `ME0`. `M0` pone la consigna a 0 V y mantiene el driver habilitado.
3. Comprueba que la autorización caduca aunque Studio siga enviando consultas. Al caducar baja EN y la consigna vuelve a cero. Repetir `ME1` o `A1` durante una autorización activa no prolonga el plazo. Para otra prueba, repite `A1` y `ME1` después de la parada.
4. En Torque Voltage el rotor puede acelerar hasta disparar el umbral `S` antes de los 300 s. Ante oscilación fuerte, movimiento inesperado, calentamiento o corriente excesiva, corta físicamente la potencia.
5. Con el driver deshabilitado tras `ME0`, envía `MC1` y comprueba que la respuesta indica `vel`. Verifica que `ME1` por sí solo deja el driver deshabilitado. Para probar Velocity, envía una nueva secuencia `A1`, `ME1` y `M0.5`. Comprueba **Target** en rad/s y deshabilita con `ME0`.

### D. Primera prueba en Angle

1. Envía `ME0`, selecciona `MC2` y coloca el rotor en el origen mecánico deseado. Envía `Z1` y comprueba que **Angle** indica aproximadamente 0 rad.
2. Envía `A1` y luego `ME1`. Comprueba que **Target** coincide con el ángulo del rotor al habilitar. Envía `M0.524`, `M1.571` y `M-1.571` por separado, supervisando **Angle**, **Velocity** y el espacio disponible para cada recorrido.
3. Deshabilita con `ME0`. Para cambiar de modo o ajustar el PID, mantén el driver deshabilitado y vuelve a enviar `A1` y `ME1` antes del siguiente movimiento. En Angle, `M0` ordena volver al origen relativo.

## Parada y recuperación en la misma sesión

- `ME0` baja EN al procesarse, deshabilita el driver y cancela la autorización. Cerrar Studio o perder USB con el ESP32 aún alimentado deja correr el plazo de 300 s desde `A1`. La detección depende del bucle de firmware y del procesamiento serie. Usa el corte físico si falla la comunicación o el firmware deja de responder.
- Commander puede escribir parámetros públicos y llamar directamente a `enable()` e `initFOC()`. Se protegen esos puntos de entrada con los métodos virtuales de `BLDCMotor`, conservando el protocolo nativo. `ME1` exige calibración y autorización vigentes. `MFR` deshabilita y se queda sin acceso a la alineación con potencia. Commander responde `MFRdone` incluso cuando la función rechaza la orden. La secuencia supervisada se inicia con `C1`.
- Tras `ME0` o la caducidad, la consigna queda en cero y se conservan la calibración, el modo, el PID y los límites. Repite `A1` y `ME1` para la siguiente prueba en cualquiera de los tres modos.
- Tras un fallo, consulta `R` en la terminal de Studio. Por ejemplo, `FOC: estado=FAULT calibrated=true enabled=false fault=OVERSPEED`. Cuando el rotor se haya detenido, envía `R1`. Se exige consigna cero, EN bajo en el GPIO, parámetros válidos, una lectura nueva del encoder y al menos 500 ms con velocidad estimada de hasta 0,2 rad/s. Un hueco de muestreo superior a 20 ms interrumpe esa comprobación. Tras recuperar, `R` indica `READY` y el driver permanece deshabilitado. Envía una nueva secuencia `A1`, `ME1` y la consigna.
- `R1` permite limpiar `OVERSPEED`, una consigna inválida ya borrada (`TARGET`) o una configuración inválida ya corregida (`CONFIGURATION`). Las escrituras rechazadas por Commander conservan el valor anterior y permiten seguir operando. Los fallos `DRIVER`, `CALIBRATION`, `ENCODER`, `INTERNAL` y `ENABLE_PIN` son críticos. Corta la potencia, corrige la causa y reinicia para inicializar y calibrar bajo supervisión. Si cambia la posición relativa entre el motor y el imán del encoder, reinicia y calibra antes de volver a mover el motor.

## Límites de la protección

- La protección calcula la velocidad con diferencias de ángulo del AS5048A en intervalos de al menos 10 ms. Enclava tras tres intervalos consecutivos por encima de `S`. Esto descarta un pico aislado y añade un retardo del orden de decenas de milisegundos. El LPF de Studio sigue afectando al PID y a la gráfica. El umbral inicial de 40 rad/s y el máximo de 60 rad/s son límites experimentales de protección. La velocidad mecánica admisible del conjunto queda pendiente de validación.
- La calibración, la sobrevelocidad y `R1` usan el ángulo de `MagneticSensorSPI` de SimpleFOC 2.4.0. La lectura nativa solicita `ANGLE` (`0x3FFF`) y recoge la respuesta de 14 bits en la trama siguiente. La comprobación de ángulo finito sigue activa. Como la biblioteca descarta la paridad y el indicador `EF`, un valor finito puede ocultar un fallo de comunicación o un encoder congelado. EN se comprueba en el GPIO del ESP32. Verifica en banco el nivel en el Mini y el corte físico. El firmware carece de medición y protección por corriente de fase. [Lectura de ángulo en SimpleFOC 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/sensors/MagneticSensorSPI.cpp)
- Se retiró el sondeo SPI adicional que podía bloquear `C1`. Su secuencia `DIAAGC` → `ANGLE` → `NOP` recogía cada respuesta en la trama siguiente. Rechazaba `EF` (bit 14) o paridad par incorrecta (bit 15) en cualquiera de las dos respuestas, además de exigir `(DIAAGC & 0x0F00) == 0x0100`. Esta última condición requiere `OCF=1`, `COF=0` y ambos avisos `COMP` apagados. Los avisos `COMP` indican el nivel del campo, mientras que `COF` señala datos angulares inválidos. `EF` puede persistir hasta leer el registro de borrado de errores `0x0001`. Sin una captura SPI del banco, queda por identificar cuál de las comprobaciones rechazó el encoder. [Hoja de datos del AS5048A, figs. 20, 22 y 25–26](https://www.mouser.com/datasheet/2/588/asset_pdf_25493108-3418798.pdf)
- La compilación comprueba el firmware para `esp32dev`. EN en el pin, compatibilidad física con Studio, calibración, estabilidad, respuesta de parada y comportamiento del sensor se verifican en banco antes de operar con carga.
