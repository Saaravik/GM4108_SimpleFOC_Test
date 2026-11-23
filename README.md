# GM4108 con SimpleFOCStudio

Firmware de banco para controlar un iPower GM4108 en lazo cerrado con Arduino SimpleFOC 2.4.0, un ESP32-DevKitC con ESP32-WROOM-32E, un encoder AS5048A y un SimpleFOC Mini v2.3. Arranca con EN bajo y expone el encoder y los ajustes por USB. La calibración y la habilitación requieren órdenes independientes del operador.

La etiqueta `diagnostico-studio-validado` conserva el firmware de diagnóstico validado en `c1d0f40`. Esa versión usa objetos de diagnóstico sin acceso al PWM. Para recuperar su código sin cambiar de rama de trabajo, usa `git worktree add --detach ../GM4108_diagnostico diagnostico-studio-validado`.

## Motor, cableado y alimentación

| Parámetro | Valor |
|---|---|
| Geometría | 24 dientes, 22 polos, 11 pares de polos |
| Resistencia entre terminales | 9,5 Ω, medida. A–B, B–C y A–C dieron 9,6 Ω, con 0,1 Ω de las puntas descontados |
| Bobinado | Estrella (Wye), según la información aportada |
| Resistencia por fase | 4,75 Ω. Dos bobinados en serie entre terminales, 9,5 Ω / 2 |
| Kv orientativo | ≈28 RPM/V para la variante 120T. Kv e inductancia quedan sin configurar |

El multímetro indicó `OL` entre cada terminal y la carcasa. `BLDCMotor(11, 4.75f)` registra la resistencia de fase. En modo `voltage`, el PID de velocidad entrega una orden de tensión Q. Su límite se expresa en voltios. [Constructor en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/BLDCMotor.cpp) · [Control por tensión](https://docs.simplefoc.com/voltage_torque_mode)

| Señal | Conexión |
|---|---|
| AS5048A VCC / GND | ESP32 3V3 / GND |
| AS5048A SCK / MISO / MOSI / CS | GPIO18 / GPIO19 / GPIO23 / GPIO27 |
| Mini IN1 / IN2 / IN3 / EN / GND | GPIO25 / GPIO26 / GPIO32 / GPIO33 / GND |
| Mini M1 / M2 / M3 | Fases A / B / C del motor |
| Potencia del Mini | Salida + / − de la KORAD, ajustada a 8,0 V y 0,5 A |

Alimenta el ESP32 por USB y comparte masa con el encoder, el Mini y el negativo de la KORAD. EN es activo en alto y lleva un pull-down externo de 10 kΩ para el reset. Deja CS1, CS2, CS3 y la salida de 3,3 V del Mini sin conectar. El AS5048A usa `AS5048_SPI`, SPI modo 1 a 1 MHz, con 14 bits de resolución. Los 0,5 A limitan la corriente de entrada de la fuente. El montaje carece de medición de corriente de fase. [Cableado del Mini v2.3](https://docs.simplefoc.com/mini_v23_connect_hardware) · [Configuración SPI en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/sensors/MagneticSensorSPI.cpp)

| Ajuste inicial | Valor |
|---|---:|
| Alimentación nominal / límite del driver | 8,0 V / 3,2 V |
| Límite de tensión del motor / tensión de alineación | 1,5 V / 1,5 V |
| Límite de velocidad | 1,0 rad/s |
| PID de velocidad P / I / D | 0,20 / 0,10 / 0 |
| Rampa del PID / LPF de velocidad Tf | 2,0 V/s / 0,05 s |
| PWM | 25 kHz |

Los límites se establecen antes de inicializar el driver y el motor. `BLDCDriver3PWM::init()` conserva 3,2 V al ser menor que 8 V. `BLDCMotor::init()` conserva 1,5 V tanto para el motor como para la alineación y actualiza el límite del PID de velocidad en modo tensión. [Inicialización y límites de BLDCMotor](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/BLDCMotor.cpp) · [Inicialización del driver](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/drivers/BLDCDriver3PWM.cpp)

## Compilación y conexión USB

```sh
pio run -e esp32dev
```

Para cargar manualmente, con la salida de la KORAD apagada, ejecuta `pio run -e esp32dev -t upload`. El entorno fija `espressif32@8.0.0` y `askuric/Simple FOC@2.4.0`.

En SimpleFOCStudio con Python 3.9, configura `/dev/cu.usbserial-0001` a 115200 baudios y 8N1. Añade un dispositivo con ID `M` y cierra otros clientes del puerto. Envía las órdenes manuales desde la terminal integrada, con terminación LF. Commander procesa `M` de forma nativa en modo legible por máquina. Comprueba `MVP` → `MVP0.200` con tres decimales. Puedes probar `MVP0.25`, volver a leer `MVP` y restaurar `MVP0.20`. Ajusta un parámetro cada vez. Los cambios se aplican en RAM y el reset recupera los valores iniciales. [Uso y terminal de SimpleFOCStudio](https://github.com/JorgeMaker/SimpleFOCStudio#usage) · [Comandos de motor](https://docs.simplefoc.com/commander_motor)

| Consulta | Parámetro |
|---|---|
| `ME` | Habilitación, inicialmente `ME0` |
| `MVP`, `MVI`, `MVD` | Ganancias P, I y D de velocidad |
| `MVR`, `MVF` | Rampa de salida y constante de tiempo del filtro de velocidad |
| `MLU`, `MLV` | Límites de tensión y de velocidad |
| `MMG0`, `MMG1`, `MMG5`, `MMG6` | Consigna, tensión Q ordenada, velocidad y ángulo |

La monitorización arranca detenida. En el gráfico, selecciona **Target**, **Voltage Q**, **Velocity** y **Angle**, fija **Downsample** en **1** y pulsa **Start**. Para diagnóstico basta con **Velocity** y **Angle**. Las filas las emite `motor.monitor()`, separadas por tabuladores y con ambos delimitadores en `'\0'`. El firmware admite esas cuatro columnas, hasta seis decimales y una oportunidad de emisión cada 20 ms si hay espacio en el TX. **Downsample** divide esa cadencia. Por ejemplo, con el 100 predeterminado de Studio sale como máximo una muestra cada dos segundos. Deja las casillas de corriente y tensión D desmarcadas para que las columnas esperadas por Studio coincidan con las emitidas. Las consultas nativas de corriente siguen disponibles y sus campos carecen de datos de medición. [Formato nativo de monitorización](https://docs.simplefoc.com/monitoring)

## Validación física pendiente

### A. USB y encoder con la KORAD apagada

1. Arranca el ESP32 y comprueba EN en el conector del Mini durante reset, arranque y conexión de Studio.
2. Consulta el PID, prueba la escritura de P y restáuralo. Gira el rotor a mano y comprueba ángulo y velocidad en el gráfico. Antes de la calibración se muestra el sentido bruto del encoder.
3. Envía `ME1` y `A1`. Comprueba `ME0` y verifica que EN sigue bajo. Reserva `C1` para la fase B.

### B. Calibración supervisada

1. Sujeta el motor firmemente, retira la rueda, deja el rotor libre y ten a mano el corte físico de la KORAD. Ajusta 8,0 V y 0,5 A. Activa la salida de la fuente cuando el banco esté preparado.
2. Envía `C1` una sola vez desde la terminal. Durante esta orden `motor.init()` activa EN y `motor.initFOC()` mueve el rotor para medir la dirección y el cero eléctrico. La calibración es síncrona y durante esos segundos el corte físico es la vía de parada.
3. Espera el mensaje `FOC: CCW, 11 pares OK, motor deshabilitado.` y comprueba `ME0` y EN bajo. El firmware exige éxito de `initFOC()`, prueba de pares favorable, dirección CCW y cero eléctrico válido. La librería puede informar `No current sense.` durante una alineación correcta en modo tensión. Si falla la calibración, corta la potencia y revisa las conexiones antes de reiniciar. [Secuencia oficial de alineación](https://docs.simplefoc.com/bldcmotor#step-6-align-motor-and-all-the-sensors-field-oriented-control-init) · [Comprobación de pares en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.cpp)

### C. Primera prueba de velocidad

1. Tras validar A y B, abre el gráfico y envía `A1` y luego `ME1`. El plazo de 15 s empieza con `A1`. Al habilitar se borra la consigna anterior y se reinicia el PID. Verifica `ME1` y consigna cero.
2. Envía `M0.1` y observa la velocidad y la tensión Q. Comprueba la respuesta a `M0` y luego deshabilita con `ME0`. `M0` mantiene el lazo de velocidad activo y puede generar par.
3. Comprueba que la autorización caduca aunque Studio siga enviando consultas. Al caducar baja EN y la consigna vuelve a cero. Repetir `ME1` o `A1` durante una autorización activa no prolonga el plazo. Para otra prueba, repite `A1` y `ME1` después de la parada.
4. Ante oscilación, movimiento inesperado o pérdida de control, corta la potencia físicamente. El ajuste histórico P = 2,00 e I = 0,20 produjo oscilaciones fuertes. La estabilidad con los valores iniciales queda pendiente de comprobar.

## Parada, recuperación y límites de la protección

- `ME0` baja EN al procesarse, deshabilita el driver y cancela la autorización. Cerrar Studio o perder USB con el ESP32 aún alimentado deja correr el plazo de 15 s desde `A1`. La detección depende del bucle de firmware y del procesamiento serie, por lo que el plazo no es una garantía de parada de emergencia. Usa el corte físico si falla la comunicación o el firmware deja de responder.
- Commander puede escribir parámetros públicos y llamar directamente a `enable()` e `initFOC()`. Se protegen esos puntos de entrada con los métodos virtuales de `BLDCMotor`, conservando el protocolo nativo. `ME1` exige calibración y autorización vigentes. `MFR` deshabilita y se queda sin acceso a la alineación con potencia. Commander responde `MFRdone` incluso cuando la función rechaza la orden. La secuencia supervisada se inicia con `C1`.
- Después de cada orden `M`, el firmware verifica modo de velocidad por tensión, modulación, cero eléctrico, límites y parámetros activos del PID. Acota consignas finitas a ±el límite configurado, con 1,0 rad/s de máximo. Los cambios que invalidan esas condiciones, una salida del PID o realimentación no finita, una calibración fallida o una velocidad filtrada superior a 1,0 rad/s deshabilitan EN y enclavan un fallo. La respuesta nativa al escribir un parámetro precede a la comprobación. Ante un fallo, corta potencia, corrige la causa y reinicia. El reset exige calibrar de nuevo y autorizar explícitamente el movimiento.
- La protección de sobrevelocidad usa la estimación del encoder con un filtro fijo de 0,05 s, independiente del LPF ajustable del PID. El disparo depende del retardo y la resolución de la estimación. La lectura SPI nativa de 2.4.0 descarta el bit de error y la paridad de la respuesta, por lo que un encoder desconectado o congelado puede devolver una lectura finita. La tensión Q es una orden y el banco carece de protección independiente de velocidad y de corriente de fase en el firmware. [Lectura del AS5048A en 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/sensors/MagneticSensorSPI.cpp)
- La compilación comprueba el firmware para `esp32dev`. EN en el pin, compatibilidad física con Studio, calibración, estabilidad, respuesta de parada y comportamiento del sensor se verifican en banco antes de operar con carga.
