# Conexiones y protocolo actual

Esta tabla extrae únicamente constantes y comportamiento observables en `src/Ino Code/Arduino_Code.ino`. No constituye todavía un diagrama eléctrico validado.

| Elemento | Arduino | Evidencia en `.ino` | Estado |
|---|---:|---|---|
| Señal del servo | D8 | `pinServo = 8`, `direccion.attach(pinServo)` | Confirmado por código |
| Motor Enable | D7 | `pinMotorEnable = 7`; PWM por software en `main.ino` | Confirmado por código; no requiere cambiar cableado |
| Motor IN1 | D9 | `pinMotorDir1 = 9` | Confirmado por código |
| Motor IN2 | D10 | `pinMotorDir2 = 10` | Confirmado por código |
| HC-SR04 Trig | D3 | `pinTrig = 3` | Confirmado por código |
| HC-SR04 Echo | D11 | `pinEcho = 11` | Confirmado por código |
| Raspberry Pi–Arduino | USB serial, 115200 baudios | `Serial.begin(115200)` | Baudio confirmado; dispositivo físico pendiente |

## Parámetros del firmware actual

- Centro lógico y físico verificado del servo: **92°**.
- Límites físicos configurados en el firmware normal: **60°–120°**. Con el
  centro físico y lógico verificado en 92°, el rango lógico normal coincide con
  **60°–120°**.
- Comando recibido: `<velocidad,angulo_logico>`; el firmware busca `<` y `>` y separa los valores con coma.
- La velocidad positiva indica avance, la velocidad negativa indica reversa y cero indica parada.
- El ángulo lógico 92 corresponde al centro físico recto de 92°.
- Si no llega un comando válido durante 500 ms, el motor se detiene automáticamente.
- Telemetría enviada: `DIST:<centimetros>`.
- Lectura nominal del ultrasónico: cada **50 ms**.
- Máximo de velocidad aceptado: **255**.
- D7 se usa como `enable` del motor. Como no es PWM por hardware en Arduino Uno, `main.ino` aplica PWM por software con período de 5 ms; el valor de velocidad de `<velocidad,ángulo>` controla el duty cycle.

## Prueba de ángulo de rueda

`software/raspberry_pi/src/steering_angle_test.py` solo envía comandos con
velocidad `0`. Sin `--execute` muestra el comando; con `--execute --port ...`
manda el ángulo al servo y envía el centro lógico 92° al terminar. En modo
interactivo mantiene la posición sin límite de tiempo hasta que introduzcas la
medición. El modo `--wheel-angle-deg` requiere pares físicos medidos en
`config/steering_wheel_calibration.json` y solo interpola dentro de ese rango.
El modo `--servo-command-deg` permite tomar esas mediciones iniciales con el
robot elevado. Para medir varios comandos en una sola sesión usa:

```bash
python3 software/raspberry_pi/src/steering_angle_test.py --interactive \
  --execute --port /dev/ttyUSB0
```

El programa solicita el comando lógico, mantiene la posición hasta que termines
de medir, pide el ángulo observado y guarda cada par en la tabla.

Para observar el giro con las ruedas apoyadas, añade `--on-ground`. Para
avanzar en un solo sentido en pasos de 5°:

```bash
python3 software/raspberry_pi/src/steering_angle_test.py --probe-limits \
  --probe-direction right --on-ground --execute --port /dev/ttyUSB0
```

Cada Enter aplica el siguiente paso y nunca cambia automáticamente al lado
contrario. `s` detiene la secuencia. El script mantiene el motor detenido
enviando velocidad `0`; coloca la mano cerca del corte de alimentación y
detén la prueba ante ruido, vibración o esfuerzo. Para este test temporal el
firmware debe estar compilado con `SERVO_LIMIT_TEST_MODE 1` y cargado al
Arduino; después de medir, vuelve a `0` y haz otro `Upload` para restaurar el
rango normal `60…120°`.

## Observaciones y riesgos pendientes

1. D7 no tiene PWM por hardware en Arduino Uno. El firmware usa PWM por software; se debe medir el duty mínimo que vence la fricción del motor y comprobar que el L298N responde correctamente.
2. El encoder se documenta como parte del motor, pero el `.ino` no define sus pines ni lee sus pulsos.
3. El botón de inicio mencionado en documentación anterior no aparece en el `.ino` actual.
4. El cableado real de alimentación y la existencia de un GND común deben confirmarse con inspección y medición.
5. Debe confirmarse si el servo recibe alimentación separada y si su señal llega directamente al Arduino.
6. El L298N parece controlar el motor de tracción; no afirmar que alimenta o controla el servo sin comprobar el cableado.

| Aspecto | Evidencia actual | Confirmación necesaria |
|---|---|---|
| D7 como Enable | PWM por software de 5 ms en el firmware | Medir velocidad real, revisar calentamiento y determinar el duty mínimo útil |
| Encoder | Descrito en README/datasheet | Seguir físicamente sus cables y registrar pines |
| Alimentación | Dos fuentes están indicadas en el contexto; README lista una unidad de LX-2BUPS | Identificar cada UPS, su salida real y qué carga alimenta |
| GND común | No aparece explícitamente en el `.ino` ni en un esquema completo | Medir continuidad y documentar el retorno común |
| Servo | Señal D8 confirmada; alimentación no | Identificar V+, GND y fuente del servo |
| L298N | Señales de control definidas; salidas no trazadas | Verificar motor conectado y alimentación del módulo |
| Inicio | No hay pin ni lógica en el `.ino` | Confirmar si existe botón físico y su conexión |
