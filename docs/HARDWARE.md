# Hardware

Este inventario reúne lo que aparece en `README.md`, `resources/`, los datasheets de `schemes/` y las fotografías de `v-photos/`. La presencia física no implica que el cableado o la alimentación estén confirmados.

| Componente | Función prevista | Modelo | Alimentación documentada | Interfaz | Conexión conocida | Fuente | Estado |
|---|---|---|---|---|---|---|---|
| Computador de placa única | Ejecutar el software de alto nivel y procesar la cámara | Raspberry Pi 4 Model B, 4 GB | 5 V/3 A por USB-C según README | USB, cámara y serial | Cámara USB y enlace USB con Arduino previstos | README, `resources/Raspberry Pi 4.png` | Documentado |
| Microcontrolador | Ejecutar el firmware y controlar las salidas | Arduino Uno R3 | 5 V de operación según README | GPIO, USB serial | Servo, motor/driver y HC-SR04 según `.ino` | README, `src/Ino Code/Arduino_Code.ino` | Confirmado por código para las señales |
| Cámara | Entregar imágenes a Raspberry Pi | Logitech C922 | USB | USB 2.0 | Conectada a Raspberry Pi; montaje visible al frente | README, `resources/Logitech C922.png`, fotografías | Documentado y observado |
| Driver de motor | Etapa de potencia del motor de tracción | L298N | README: Vs 5–35 V; confirmar módulo real | Entradas digitales/PWM y salida al motor | El `.ino` define señales de dirección y enable; cableado físico pendiente | README, `resources/Driver_L298N.png`, `.ino` | Documentado; conexión física pendiente |
| Sensor de distancia | Medir distancia frontal | HC-SR04 | No confirmada para el montaje real | Trig/Echo | Trig D3, Echo D11 | README, `resources/HC-SR04_Ultrasonic_Sensor.png`, `.ino`, fotografías | Pines confirmados por código; alimentación pendiente |
| Chasis | Soporte y estructura móvil | Fischertechnik Maker Kit Car | Pendiente | Mecánica | Ruedas, dirección, motor y diferencial del kit | README, `resources/Fischertechnik_Maker_Kit_Car.png`, datasheets, fotografías | Documentado y observado |
| Dirección | Mover el mecanismo de dirección | Servomotor incluido en el kit; modelo exacto pendiente | Pendiente | Señal servo | Señal en D8 según `.ino`; montaje visible | README, datasheet, `.ino`, fotografías | Señal confirmada; modelo y alimentación pendientes |
| Tracción | Mover el vehículo; incluye encoder según kit | Motor con encoder incluido en el kit | Pendiente | Motor y encoder | Motor asociado al L298N; pines del encoder no definidos en `.ino` | README, datasheet, fotografías | Motor documentado; encoder pendiente |
| Transmisión | Transferir la tracción a las ruedas | Diferencial del chasis | Mecánica | Mecánica | Ubicación y relación exactas pendientes | README, datasheet, fotografías | Documentado; medidas pendientes |
| UPS de potencia | Alimentar etapas del robot | Dos módulos LX-2BUPS: uno indicado como 5 V y otro como 12 V | Salidas indicadas en el contexto; confirmar modelos y valores reales | Alimentación | Cableado no confirmado | Contexto solicitado, README, `resources/LX-2BUPS.png`, fotografías | Documentado por contexto; cantidad contradicha por README |
| Baterías | Almacenar energía | 18650, 3.7 V nominal | 3.7 V nominal por batería | Alimentación | Dos por UPS según contexto solicitado | README, `resources/Ultrafire_TR18650_9800mAh_3.7V.png`, fotografías | Documentado; configuración y capacidad real pendientes |
| Protoboard y cableado | Interconexión y distribución | Protoboard/módulos auxiliares | Depende de la alimentación | Cables Dupont y conectores visibles | Conexiones individuales no trazables con certeza en las fotos | `resources/Micro_Protoboard.png`, fotografías | Observado; pendiente de confirmar |

La cifra de las baterías debe registrarse como **9800 mAh anunciados, capacidad real sin verificar**. No debe tratarse como capacidad medida.

## Construcción observada

Las seis fotografías muestran un chasis rojo de Fischertechnik con cuatro ruedas, una Raspberry Pi montada en la parte superior, Arduino y módulos electrónicos sujetos al chasis, cámara Logitech C922 en la zona frontal, sensor ultrasónico frontal, cableado expuesto y elementos de alimentación sujetos con bridas. Las fotos sirven para describir disposición general, no para deducir cada conexión.

## Medidas físicas registradas

Estas medidas fueron proporcionadas por el usuario y todavía deben verificarse con una medición repetida:

| Medida | Valor | Estado |
|---|---:|---|
| Distancia entre extremos delantero y trasero | 21.15 cm | Documentada por el usuario |
| Distancia entre centro de rueda trasera y delantera | 14.8 cm | Documentada por el usuario |
| Diámetro de las ruedas | 6.3 cm | Documentada por el usuario |
| Distancia entre centros de las ruedas delanteras | 14.6 cm | Documentada por el usuario |
| Ángulo vertical de la cámara | Ajustable | Documentado por el usuario; valor pendiente |

El archivo `src/physical_setup.py` permite registrar estas medidas, el centro provisional del servo y sus límites de prueba sin conectarse al hardware.
