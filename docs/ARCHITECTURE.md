# Arquitectura base

La arquitectura inicial separa percepción, decisión de alto nivel y ejecución física, sin fijar todavía el algoritmo de navegación.

```text
Logitech C922 -> Raspberry Pi -> USB serial -> Arduino Uno -> servo / L298N / motor
                                                        |
                                                        -> HC-SR04 -> telemetría serial
```

- La Logitech C922 entrega imágenes a la Raspberry Pi.
- La Raspberry Pi ejecutará la futura visión y la toma de decisiones.
- La Raspberry Pi enviará comandos al Arduino por serial USB.
- El Arduino controlará el servo de dirección y el motor mediante el L298N, según la interfaz actualmente definida en el firmware.
- El Arduino leerá el HC-SR04 y devolverá su telemetría.
- El encoder queda pendiente de integración: el motor lo incluye según la documentación, pero no está integrado en el `.ino` actual.
- La Raspberry Pi y la etapa de potencia usan fuentes diferenciadas según la documentación existente; el reparto y el GND común todavía deben verificarse.

Esta distribución es una base inicial. El algoritmo de percepción, planificación y control se diseñará desde cero y no se deriva de los archivos Python heredados.

