# GM4108 con SimpleFOCWebController

Firmware de banco para un iPower GM4108, un AS5048A, un SimpleFOC Mini v2.3 y un ESP32-DevKitC con ESP32-WROOM-32E. Usa Arduino SimpleFOC 2.4.0 en lazo cerrado de velocidad con par por tensión. El ajuste de PID, filtro y consigna y la visualización se hacen por USB desde SimpleFOCWebController.

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

La segunda orden carga por USB. Para la primera carga mantén desconectada la salida de potencia de la KORAD y conserva `alignOnBoot = false` en `include/config.h`.

## Primera comprobación por USB

1. Cierra el monitor serie de PlatformIO y abre [SimpleFOCWebController](https://webcontroller.simplefoc.com/) en un navegador con Web Serial.
2. Selecciona **115200** y el puerto USB del ESP32. Debe aparecer el motor **M / GM4108**. En la terminal, `?` lista el motor y `ME` debe devolver `ME0`.
3. Activa la gráfica de **Angle** o consulta `MMG6` en la terminal. Gira despacio el rotor a mano en ambos sentidos y comprueba que el ángulo cambia. Verifica físicamente que EN permanece bajo y que `ME1` se rechaza.
4. Comprueba que el navegador puede consultar y ajustar el PID y el filtro, y que el firmware rechaza las escrituras de límites y los cambios de modo. Los valores editados durante esta comprobación se pierden al reiniciar.

Con la configuración entregada se inicializan SPI y PWM con EN bajo. Se omiten `motor.init()` e `initFOC()`, y la lectura de ángulo emplea el sentido bruto del encoder.

## Alineación supervisada y primer movimiento

1. Sujeta el motor sin rueda ni carga y comprueba que gira libremente. Ten a mano el corte físico de potencia. Ajusta la KORAD a **8,0 V / 0,5 A** y verifica la tensión efectiva en los bornes del Mini. La guía del Mini v2.3 indica 8 V como mínimo, por lo que hay que comprobar el margen bajo carga. [Alimentación del Mini v2.3](https://docs.simplefoc.com/mini_v23_connect_hardware)
2. Para una sesión con potencia, cambia `alignOnBoot` a `true`, recompila y carga con la salida de la fuente apagada. Tras preparar el banco, aplica la alimentación de potencia y reinicia deliberadamente el ESP32 para alinear.
3. **En cada reinicio de esa compilación, `motor.init()` habilita EN e `initFOC()` energiza el motor y puede mover el rotor con consigna cero.** Durante la alineación el firmware permanece dentro de las llamadas bloqueantes de SimpleFOC. Ten disponible el corte físico, también al conectar Web Serial si el puerto provoca un reinicio. [Inicialización de BLDCMotor](https://docs.simplefoc.com/bldcmotor)
4. Una alineación correcta deja EN bajo y consigna cero. Habilita mediante el interruptor de WebController o `ME1`. Envía después una consigna pequeña, por ejemplo `M0.2`. Con el control habilitado, la consigna cero puede aplicar par para corregir movimiento.
5. Si falla la inicialización, la alineación o la comprobación de pares de polos, el firmware baja EN y bloquea la habilitación hasta un reinicio deliberado. Ante un fallo, corta la potencia y revisa su causa antes de reiniciar.

## Ajuste y gráficas

| Ajuste | Inicial | Rango de escritura | Comando de ejemplo |
|---|---:|---:|---|
| V-P | 0,20 | 0–1,00 | `MVP0.20` |
| V-I | 0,10 | 0–0,20 | `MVI0.10` |
| V-D | 0 | 0–0,01 | `MVD0` |
| V-Filter (Tf) | 0,05 s | 0,01–0,10 s | `MVF0.05` |
| Consigna de velocidad | 0 rad/s | −1,0 a +1,0 rad/s con EN habilitado | `M0.2` |

P=0,20 e I=0,10 reproducen el punto prudente de TEST-012, donde antes no hubo movimiento. TEST-013 giró débilmente con P=1,00 e I=0,20. TEST-014 osciló con P=2,00 e I=0,20. D comienza en cero para evitar amplificar el ruido de la velocidad. Tf parte de los 50 ms del banco anterior y la salida del PID tiene una rampa nativa fija de 2 V/s. Son valores de puesta en marcha pendientes de ajuste. Cambia un parámetro cada vez y observa la respuesta al subir, bajar y volver a cero. [Comandos de motor](https://docs.simplefoc.com/commander_motor)

En **Monitoring**, selecciona **Target**, **Velocity** y **Vq**, deja inicialmente **Downsample** en 100 y pulsa **Start**. La consigna y la velocidad están en rad/s y Vq es la orden de tensión del eje q, en V. Las casillas de corriente carecen de una medición de fase en este banco. Pulsa **Pause** para detener las tramas. El arranque tiene la emisión desactivada y el navegador usa el formato de monitorización nativo delimitado por `M`. [Configuración de WebController](https://docs.simplefoc.com/webcontroller) · [Código oficial de WebController](https://github.com/simplefoc/simplefoc-webcontroller)

Los ajustes viven en RAM. Para conservarlos, registra los valores que devuelven `MVP`, `MVI`, `MVD` y `MVF`. Puedes reintroducirlos en una sesión posterior o actualizarlos en `include/config.h`, compilar y cargar al consolidar un ajuste.

## Límites y parada

| Parámetro | Valor | Aplicación |
|---|---:|---|
| Bus configurado | 8,0 V | Referencia del driver para calcular el PWM |
| Límite del driver | 3,2 V | Orden máxima por salida PWM, equivalente al 40 % con bus configurado a 8 V |
| Límite del motor | 1,5 V | Magnitud máxima de la orden Vq en control |
| Tensión de alineación | 1,5 V | Magnitud de la orden durante la alineación |
| Límite de consigna | ±1,0 rad/s | Comprobado antes de delegar en Commander |

Los **0,5 A** son el límite de salida de la KORAD hacia la entrada del Mini. La corriente real de las fases y el par quedan indeterminados. La protección reactiva baja EN y bloquea el rearme si la velocidad del encoder, antes del LPF ajustable, supera 1,0 rad/s durante 100 ms, si se detecta un valor no finito en la velocidad o la salida del PID, o si el lazo vuelve a ejecutarse tras una pausa superior a 100 ms. Los límites, la rampa y las cotas del PID reducen el riesgo de oscilación. Una oscilación por debajo del umbral todavía requiere intervención del operador.

Commander 2.4.0 permite escribir modos, límites, offsets y parámetros de motor, relanzar FOC y pasar un límite de tensión como segundo número en una consigna. El firmware filtra líneas y valida el comando antes de llamar al despachador oficial. Solo acepta cambios de PID y LPF de velocidad, consignas acotadas, monitorización y habilitación o deshabilitación. Los modos, límites y demás parámetros del control se pueden consultar. Las escrituras bloqueadas pueden seguir apareciendo como controles editables en el navegador. La terminal muestra `Merr` si se rechazan. [Implementación de Commander 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/communication/Commander.cpp)

Para una parada normal, envía `M0` y después `ME0` o deshabilita el motor con el interruptor de WebController. `ME0` baja EN y borra la consigna y la orden de tensión. Ante vibración, movimiento inesperado, pérdida de comunicación o durante una alineación problemática, corta físicamente la alimentación del Mini. Cerrar el navegador o soltar el USB de datos con el ESP32 todavía alimentado puede dejar activa la última consigna.

La lectura nativa del AS5048A en 2.4.0 enmascara los bits de paridad y error de la respuesta. Una lectura SPI corrupta o congelada puede parecer válida. El estimador nativo de velocidad también conserva el último valor mientras el encoder deja de cambiar, incluso con el rotor detenido. Los cortes de software dependen de las lecturas que entregue la biblioteca y de que el bucle siga ejecutándose. Quedan pendientes las comprobaciones físicas de EN, el enlace Web Serial, el giro en ambos sentidos, la alineación y la respuesta del motor al ajuste y a la parada. [Lector SPI 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/sensors/MagneticSensorSPI.cpp) · [Estimador de velocidad 2.4.0](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/Sensor.cpp)

El firmware se basa en el [patrón de control oficial](https://docs.simplefoc.com/code) y en el [ejemplo de ESP32 con sensor magnético](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/examples/hardware_specific_examples/ESP32/magnetic_sensor/esp32_position_control/esp32_position_control.ino).
