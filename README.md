# GM4108_SimpleFOC_Test

Banco independiente para un iPower GM4108 con AS5048A, un SimpleFOC Mini v2.3 y un ESP32-DevKitC con ESP32-WROOM-32E. La primera sesión cubre **A, B y C con la salida de la KORAD apagada**. El firmware entregado permite comprobar el encoder y preparar el PWM del ESP32 manteniendo EN bajo. La calibración y el movimiento quedan bloqueados mediante dos permisos en `include/config.h`.

## Cableado

| Origen | Destino |
|---|---|
| AS5048A VCC / GND | ESP32 3V3 / GND |
| AS5048A SCK / MISO / MOSI / CS | GPIO18 / GPIO19 / GPIO23 / GPIO27 |
| Mini IN1 / IN2 / IN3 / EN / GND | GPIO25 / GPIO26 / GPIO32 / GPIO33 / GND |
| Mini M1 / M2 / M3 | Motor fase A / B / C |
| KORAD positivo / negativo | Mini + / − |

Alimenta el ESP32 por USB. Conecta la masa del ESP32, el encoder, el Mini y el negativo de la KORAD. Los 8 V van exclusivamente a los bornes de potencia del Mini. Deja CS1, CS2, CS3 y la salida de 3,3 V del Mini sin conectar. El AS5048A usa SPI modo 1 a 1 MHz, con 14 bits de resolución y el registro angular `0x3FFF`.

Instala y verifica un **pull-down físico en EN** adecuado para el módulo y el montaje. Durante reset y antes de que el ESP32 configure el GPIO, el software carece de control sobre EN. Comprueba el nivel en el pin del Mini durante arranque y reset, con la etapa de potencia desactivada. Ten accesible un interruptor físico que corte la alimentación de potencia. Úsalo ante movimientos imprevistos o pérdida de control.

## Parámetros que deben confirmarse antes de las pruebas con potencia

- **Pares de polos.** El iPower GM4108H está documentado como 24N/22P. Los 22 polos del rotor corresponden a `polePairs = 11` y `polePairsConfirmed = true`. El chequeo durante FOC es una comprobación adicional. [Ficha de iFlight](https://shop.iflight.com/cn/iPower-GM4108-Motor-Pro951)
- **Tensión en los bornes del Mini.** La referencia inicial de la KORAD KA3005D es **8,00 V**, con la salida inicialmente apagada. Hay una discrepancia de aproximadamente **3,2 %** con el multímetro y queda por establecer la exactitud de ambos instrumentos. La guía del Mini v2.3 indica un rango de **8–35 V**. Verifica la tensión efectiva con una referencia fiable, incluida la caída bajo carga prevista y el margen sobre el mínimo. Mantén suspendidas las pruebas con el motor si la tensión real o su margen siguen sin verificarse. [Guía oficial del Mini v2.3](https://docs.simplefoc.com/mini_v23_connect_hardware)
- **Límites de la prueba.** La referencia de límite de la fuente es **0,500 A de entrada**. La fuente limita corriente en su salida y la corriente de fase puede ser distinta. La lectura de **0 Ω** en los tres pares es inconcluyente. Mantén la resistencia sin especificar en SimpleFOC y valida los límites eléctricos del motor con datos fiables de esta variante o mediciones adecuadas. Revisa los límites de la tabla y activa `benchLimitsConfirmed` solo después de validar que los ajustes son adecuados para este banco. El firmware carece de mediciones de corriente de fase, temperatura, tensión de bus y señales de fallo del driver.
- **Montaje.** Sujeta el motor y comprueba que el rotor puede girar libremente, sin rueda ni carga. Verifica polaridad, masa común, fases, modo 3-PWM, EN y corte físico antes de activar potencia. Recompila y vuelve a cargar tras confirmar o ajustar parámetros en `config.h`.

| Parámetro SimpleFOC | Valor inicial | Interpretación |
|---|---:|---|
| `driver.voltage_power_supply` | 8,00 V | Estimación del bus usada como divisor para calcular el ciclo útil. Ajustar a la tensión validada. |
| `driver.voltage_limit` | 3,20 V | Máxima orden por salida PWM en el modelo del driver. Con bus configurado a 8,00 V equivale a 40 % de ciclo útil. |
| `motor.voltage_limit` | 1,50 V | Magnitud máxima de la orden de eje q durante el control de velocidad. |
| `motor.voltage_sensor_align` | 1,50 V | Magnitud de la orden usada al alinear. Una alineación fallida requiere revisar el banco antes de otro intento. |

Los límites de tensión son valores provisionales de puesta en marcha. El PWM aplica pulsos del bus y las órdenes de tensión se traducen a ciclos útiles. Se usa PWM sinusoidal centrado, control FOC de par por tensión y lazo de velocidad, con un objetivo máximo inicial de **±1,0 rad/s** y una rampa de **0,50 rad/s²**. La velocidad del objetivo y la velocidad física se observan por separado.

## Compilar y abrir el monitor

Abre esta carpeta como proyecto propio en VS Code con PlatformIO. El entorno `esp32dev` usa Arduino, Espressif32 **8.0.0** y SimpleFOC oficial **2.4.0**. Compilación comprobada con Arduino-ESP32 **3.3.12**.

```sh
pio run
pio run -t upload
pio device monitor -b 115200
```

Carga por USB con la salida de la KORAD apagada. Selecciona el puerto de la DevKitC si hay varios dispositivos. En el monitor usa terminación LF, CR o CRLF. Escribe los números con punto decimal. La calibración queda en RAM y se pierde al reiniciar.

## Estados y comandos

El arranque entra en **SAFE** con EN bajo. `SENSOR` pasa a **SENSOR_TEST**. Tras detectar al menos 20 muestras válidas consecutivas y un desplazamiento manual de aproximadamente 5°, `INIT` configura el PWM y pasa a **READY** con EN bajo. Su éxito comprueba la inicialización de PWM del ESP32. Comprueba físicamente la etapa de potencia por separado.

`CALIBRATE` solicita una confirmación en READY. Un `CONFIRM` válido entra en **CALIBRATION**, llama a `motor.init()` y después a `motor.initFOC()`. El firmware comprueba el retorno, los datos del AS5048A, el chequeo de pares de polos y el cero eléctrico. Deshabilita el driver al terminar. El éxito vuelve a **READY** con FOC válido y el fallo o aborto vuelve a **SAFE**. Un nuevo intento requiere revisar la causa y repetir los comandos desde `SENSOR`. `START` desde READY con FOC válido entra en **MOTOR_CONTROL** con objetivo cero.

| Comando | Acción |
|---|---|
| `HELP` | Lista los comandos y el primer paso. |
| `STATUS` | Muestra estado, lecturas, límites, PWM, orden EN, resultado FOC y último error. |
| `SENSOR` | Inicia el diagnóstico, deshabilita EN y descarta la calibración. Desde MOTOR_CONTROL, primero usa `STOP`. |
| `INIT` | Prepara el PWM tras la prueba manual del encoder. |
| `CALIBRATE` | Pide autorización para alinear si los permisos de configuración están confirmados. |
| `CONFIRM` | Dentro de los siguientes 30 s, confirma que el motor está sujeto, libre, sin rueda ni carga, la fuente está correctamente configurada y la salida de potencia se ha activado intencionadamente. |
| `START` | Habilita el control manual con objetivo cero y abre la autorización de 5 s. El control activo a cero puede aplicar par. |
| `V 1` / `V -1` / `V 0` | Cambia el objetivo en rad/s y renueva la autorización. `V 0` mantiene el control activo. |
| `KEEP` | Renueva la autorización sin cambiar el objetivo. |
| `TEST_SPEED` | Desde READY con FOC válido y encoder saludable, solicita `CONFIRM` para ejecutar una prueba automática de 7 s. |
| `STOP` | Anula el objetivo y baja EN de inmediato al procesarse. Desde MOTOR_CONTROL conserva el FOC válido en READY. |
| `DISABLE` | Baja EN, descarta la calibración y vuelve a SAFE. |

Cada `V` válido o `KEEP` abre otros **5 s** de control manual. Al vencer la autorización, el objetivo baja a cero según la rampa y el driver se deshabilita al llegar a cero. A ±1,0 rad/s la rampa dura como máximo otros 2 s. Las renovaciones durante esa parada se rechazan. Para rearmar, espera READY y envía un nuevo `START`.

Para la prueba automática, envía `TEST_SPEED` y luego `CONFIRM` dentro de 30 s. El motor mantiene objetivo 0 rad/s durante 1 s, recibe una consigna de +0,2 rad/s durante 5 s y vuelve a consigna cero durante 1 s. El objetivo efectivo usa la rampa de 0,50 rad/s² en ambos cambios. Al llegar el plazo de 7 s el lazo corta EN y vuelve a READY. La prueba funciona sin `KEEP`. `STOP` corta inmediatamente y conserva FOC en READY. `DISABLE` corta, invalida la calibración y vuelve a SAFE. Un fallo del encoder o del lazo también corta y vuelve a SAFE.

La prueba emite telemetría cada 200 ms con consigna, rampa, velocidad estimada en el sentido calibrado, ángulo bruto del encoder, estado del PWM y de EN, y fase. Si se llena el búfer de serie, se omite esa muestra para mantener el lazo FOC. El resumen indica el desplazamiento angular neto acumulado, la velocidad media con signo, el pico absoluto de velocidad y el motivo de finalización.

Durante la calibración, una lectura rechazada del encoder o la recepción de `STOP` o `DISABLE` baja EN, pone el PWM a cero y bloquea la energización durante el resto del intento. `motor.init()` y `motor.initFOC()` contienen esperas bloqueantes en SimpleFOC 2.4.0. Si la incidencia ocurre durante una espera, se detecta al retomar las lecturas del encoder o la atención de la consola. Ante cualquier anomalía utiliza el **corte físico de potencia** inmediatamente.

La velocidad mostrada sigue el sentido bruto del encoder. El lazo de control usa el sentido encontrado durante la alineación. El monitor indica `N/D` para velocidad si la lectura falla o todavía no hay una ventana válida. Los contadores cubren paridad, bandera de error SPI (EF), diagnóstico magnético y saltos angulares. `regErr` conserva el último código recuperado del registro de errores (`-1` si aún no hay uno fiable) y `regDiag` muestra la última trama de diagnóstico. `EN pedido` refleja la orden de software y se verifica físicamente en el banco. [Hoja de datos oficial del AS5048A](https://look.ams-osram.com/m/287d7ad97d1ca22e/original/AS5048-DS000298%20Datasheet)

## Pruebas progresivas

### A — Firmware y comunicación

1. Mantén apagada la salida de potencia de la KORAD y alimenta el ESP32 solo por USB. Compila, carga y abre el monitor a 115200.
2. Reinicia el ESP32. Confirma el mensaje SAFE, objetivo cero y EN bajo. Verifica con `STATUS` que el PWM sigue pendiente. Comprueba que `START` y `V 1` se rechazan.

### B — Encoder

1. Envía `SENSOR` con la salida de la KORAD apagada. Gira despacio el rotor a mano en ambos sentidos y comprueba el cambio del ángulo y de la cuenta bruta de 0 a 16383. Comprueba la transición entre 359° y 0° y que la velocidad vuelve a aproximadamente cero al detenerlo.
2. Compara con el comportamiento ya observado en el proyecto de encoder anterior. Confirma `giro=si` y que los contadores de errores no aumentan durante un giro normal. Un valor fijo de 0° por sí solo carece de valor diagnóstico.
3. Si aparecen errores, revisa SPI, masa, alimentación del AS5048A e imán, y repite `SENSOR`. Durante todo el paso verifica que EN permanece bajo.

### C — Preparación del driver

1. Con potencia apagada, revisa polaridad, cableado, GPIO, masa común, configuración de la KORAD, pull-down y corte físico.
2. Desde SENSOR_TEST con la prueba manual superada, envía `INIT`. Verifica READY, PWM listo y EN bajo en el Mini. Comprueba que el rotor sigue inmóvil. En la compilación inicial `CALIBRATE` queda bloqueado y `START` exige un FOC válido.

### D — Calibración supervisada

1. Continúa solo tras completar A–C y confirmar los parámetros de la sección de potencia. Tras actualizar permisos, cargar y repetir B–C, sujeta el motor sin rueda ni carga. Verifica que puede girar libremente, que el corte físico está accesible y que la fuente tiene los ajustes validados.
2. Activa intencionadamente la salida y comprueba que el rotor permanece quieto en READY. Envía `CALIBRATE` y, tras volver a comprobar las condiciones, `CONFIRM`.
3. Registra el resultado de `motor.init()`, `initFOC()`, el chequeo de pares de polos y, si hay éxito, el sentido y el cero eléctrico. Verifica READY, FOC OK y EN bajo al acabar. Usa el corte de potencia inmediatamente si hay un movimiento anómalo. Ante un fallo, apaga la salida y revisa la causa antes de otro intento.

### E — Primer movimiento supervisado

1. Con FOC OK y sin carga, envía `START` y luego `V 0.2` antes del timeout. Renueva con `KEEP` cada segundo. Observa ángulo, velocidad estimada, corriente de entrada y tensión de la KORAD.
2. Envía `STOP`. Tras verificar la parada, rearma con `START` y prueba `V -0.2`. Registra el comportamiento en ambos sentidos. Si no hay movimiento, revisa el resultado y el banco antes de cambiar límites.
3. Detén el motor y desconecta la potencia antes de comprobar calentamiento. Suspende la prueba ante vibración anómala, calentamiento, fallos de encoder o entrada de la fuente en limitación.

### F — Parada y reinicio

1. A velocidad muy baja, envía `STOP` y comprueba objetivo cero, EN bajo y retorno a READY. Rearma y repite con `DISABLE`. Verifica SAFE, EN bajo y rechazo de `START` y `V 1`.
2. Tras recuperar FOC OK mediante la secuencia supervisada, arranca a velocidad muy baja y deja de renovar la autorización. Comprueba la rampa a cero, el mensaje de timeout, EN bajo y la necesidad de un nuevo `START`.
3. Desactiva la salida de potencia y reinicia el ESP32. Comprueba el arranque en SAFE, objetivo cero, calibración pendiente y EN bajo durante y después del reset. Contrasta la lectura del monitor con la medición física de EN.

## Referencias de implementación

- Registro oficial con la dependencia fijada. [SimpleFOC en PlatformIO](https://registry.platformio.org/libraries/askuric/Simple%20FOC) · [Versión oficial 2.4.0](https://github.com/simplefoc/Arduino-FOC/releases/tag/v2.4.0)
- Constructor e inicialización SPI. Se inicializa primero el bus ESP32 con los pines explícitos. La subclase de `MagneticSensorSPI` comprueba paridad, EF y diagnóstico en la misma lectura que se entrega a FOC, ya que el lector base de esta versión enmascara esos bits. [Guía SPI de SimpleFOC](https://docs.simplefoc.com/magnetic_sensor_spi) · [Código oficial de MagneticSensorSPI](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/sensors/MagneticSensorSPI.cpp)
- El código de la versión fija documenta los efectos de `init()`, `enable()` y `disable()`, y la comprobación de pares que acompaña a `initFOC()`. [BLDCMotor](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/BLDCMotor.cpp) · [BLDCDriver3PWM](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/drivers/BLDCDriver3PWM.cpp) · [FOCMotor](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/base_classes/FOCMotor.cpp)
