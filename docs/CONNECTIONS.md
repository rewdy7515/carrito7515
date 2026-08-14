# Conexiones y protocolo actual

Esta tabla extrae únicamente constantes y comportamiento observables en `src/Ino Code/Arduino_Code.ino`. No constituye todavía un diagrama eléctrico validado.

| Elemento | Arduino | Evidencia en `.ino` | Estado |
|---|---:|---|---|
| Señal del servo | D8 | `pinServo = 8`, `direccion.attach(pinServo)` | Confirmado por código |
| Motor PWM/Enable | D7 | `pinMotorPWM = 7`, `analogWrite(D7, velocidadAuto)` | Confirmado por código; idoneidad PWM pendiente |
| Motor IN1 | D9 | `pinMotorDir1 = 9` | Confirmado por código |
| Motor IN2 | D10 | `pinMotorDir2 = 10` | Confirmado por código |
| HC-SR04 Trig | D3 | `pinTrig = 3` | Confirmado por código |
| HC-SR04 Echo | D11 | `pinEcho = 11` | Confirmado por código |
| Raspberry Pi–Arduino | USB serial, 115200 baudios | `Serial.begin(115200)` | Baudio confirmado; dispositivo físico pendiente |

## Parámetros del firmware actual

- Centro provisional del servo: **86°**.
- Rango aceptado por el firmware: **60°–120°**.
- Comando recibido: `<velocidad,angulo>`; el firmware busca `<` y `>` y separa los valores con coma.
- Telemetría enviada: `DIST:<centimetros>`.
- Lectura nominal del ultrasónico: cada **50 ms**.
- Máximo de velocidad aceptado: **255**.
- Las velocidades negativas se convierten a cero; el firmware actual no implementa reversa.

## Observaciones y riesgos pendientes

1. D7 no tiene PWM por hardware en Arduino Uno. Aunque el código ejecuta `analogWrite(D7, valor)`, se debe confirmar o corregir el pin antes de asumir control gradual de velocidad.
2. El encoder se documenta como parte del motor, pero el `.ino` no define sus pines ni lee sus pulsos.
3. El botón de inicio mencionado en documentación anterior no aparece en el `.ino` actual.
4. El cableado real de alimentación y la existencia de un GND común deben confirmarse con inspección y medición.
5. Debe confirmarse si el servo recibe alimentación separada y si su señal llega directamente al Arduino.
6. El L298N parece controlar el motor de tracción; no afirmar que alimenta o controla el servo sin comprobar el cableado.

| Aspecto | Evidencia actual | Confirmación necesaria |
|---|---|---|
| D7 como PWM | Usado como `pinMotorPWM` en el código | Verificar el comportamiento eléctrico y seleccionar un pin PWM válido si corresponde |
| Encoder | Descrito en README/datasheet | Seguir físicamente sus cables y registrar pines |
| Alimentación | Dos fuentes están indicadas en el contexto; README lista una unidad de LX-2BUPS | Identificar cada UPS, su salida real y qué carga alimenta |
| GND común | No aparece explícitamente en el `.ino` ni en un esquema completo | Medir continuidad y documentar el retorno común |
| Servo | Señal D8 confirmada; alimentación no | Identificar V+, GND y fuente del servo |
| L298N | Señales de control definidas; salidas no trazadas | Verificar motor conectado y alimentación del módulo |
| Inicio | No hay pin ni lógica en el `.ino` | Confirmar si existe botón físico y su conexión |

