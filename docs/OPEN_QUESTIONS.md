# Preguntas abiertas

Estas preguntas requieren inspección, medición o confirmación del equipo. Hasta resolverlas, los datos deben permanecer como **PENDIENTE DE CONFIRMAR**.

## Electricidad y cableado

- ¿Cuál es el cableado real entre Raspberry Pi, Arduino, L298N, motor, servo y HC-SR04?
- ¿Cuáles son los pines reales del encoder y qué niveles eléctricos entrega?
- ¿Cuál será el pin PWM definitivo para el enable del motor?
- ¿Cuál es el modelo exacto del servomotor y su tensión/corriente recomendadas?
- ¿Qué voltajes se miden bajo carga en la Raspberry Pi, Arduino, L298N, servo y motor?
- ¿Existe un GND común entre las fuentes y las señales?
- ¿El servo tiene alimentación separada? ¿Su señal llega directamente al Arduino?
- ¿Hay dos `LX-2BUPS`? El README lista cantidad 1, mientras que el inventario solicitado indica uno de 5 V y otro de 12 V.
- ¿Qué salida real entrega cada UPS y qué componentes alimenta?
- ¿La configuración de las cuatro baterías es dos por UPS? ¿Cómo se conectan y protegen?
- ¿Cuál es la capacidad real de las baterías? La referencia disponible solo permite registrar “9800 mAh anunciados, capacidad real sin verificar”.
- ¿Qué conexiones de la protoboard y de los módulos auxiliares están actualmente en uso?

## Actuadores y sensores

- ¿Cuál es el rango mecánico seguro del servo?
- ¿Cuál es el centro real de dirección, comparado con el valor provisional de 86°?
- ¿Cuál es el sentido del motor para cada combinación de IN1/IN2?
- ¿El L298N controla únicamente el motor de tracción? ¿Dónde están conectadas sus salidas?
- ¿Dónde está ubicado y hacia dónde apunta exactamente el HC-SR04?
- ¿Qué debe ocurrir ante una lectura inválida o pérdida de telemetría?
- ¿Existe un botón de inicio físico? Si existe, ¿a qué pin y con qué circuito está conectado?

## Mecánica

- ¿Cuáles son las dimensiones del vehículo?
- ¿Cuál es la distancia entre ejes?
- ¿Cuál es el ancho de vía?
- ¿Qué piezas exactas del Fischertechnik Maker Kit Car forman la transmisión y el diferencial?
- ¿Qué parte del motor con encoder se puede usar para medir movimiento una vez integrados sus pines?

## Cámara y software futuro

- ¿Qué resolución y FPS se utilizarán con la Logitech C922?
- ¿Cuál es la orientación y altura definitivas de la cámara?
- ¿Cuál será el puerto serial real en la Raspberry Pi?
- ¿Qué sistema operativo, versión de Python y dependencias estarán instalados?
- ¿Qué interfaz documentada y validada se usará para que Python comunique comandos al firmware?

