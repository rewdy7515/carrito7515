# Simulador 2D WRO 2026

Este directorio es independiente del software que se ejecuta en la Raspberry Pi
o el Arduino. No abre el puerto serial ni envía comandos al robot físico.

## Medidas usadas

| Elemento | Valor en el simulador | Fuente |
| --- | ---: | --- |
| Tapete | 320 x 320 cm | PDF WRO 2026, 13.1; `docs/TRACK_CONTEXT.md` |
| Área interior de pista | 300 x 300 cm | PDF WRO 2026, 13.1; `docs/TRACK_CONTEXT.md` |
| Muro interno, contorno exterior | 100 x 100 cm | Picos de las líneas del plano aportado por el equipo |
| Panel central dentro del muro | 80 x 80 cm | Plano de pista aportado por el equipo |
| Zona fija de salida | 50 x 40 cm (500 x 400 mm), recta inferior izquierda | Configuración solicitada por el equipo |
| Guía amarilla de perímetro | 0.3 cm de grosor | Plano de pista aportado por el equipo |
| Líneas azul y naranja | 2 cm de ancho | PDF WRO 2026, 13.9; `docs/TRACK_CONTEXT.md` |
| Pilar rojo o verde | 5 x 5 cm en planta, 10 cm de alto | PDF WRO 2026, 13.1; `docs/TRACK_CONTEXT.md` |
| Asiento de señal | 5 x 5 cm | Plano de pista aportado por el equipo |
| Círculo de evaluación del asiento | 8.5 cm de diámetro | Plano de pista aportado por el equipo |
| Carro, largo | 21.15 cm | `docs/HARDWARE.md` |
| Distancia entre ejes | 15.0 cm | `docs/HARDWARE.md` |
| Ancho usado para el rectángulo | 17.2 cm | Envolvente derivada: 14.9 cm entre centros + 2.3 cm de rueda |
| Giro de ruedas usado en calibración | ±20.7° | Límite histórico del modelo de bicicleta; las últimas mediciones se registran por rueda y requieren radio real antes de reemplazarlo |
| Radio mínimo de giro a la derecha | 32.2 cm | Medición física aportada por el equipo; equivale a +24.98° en el modelo de bicicleta |
| Radio mínimo de giro a la izquierda | 43.0 cm | Medición física aportada por el equipo; equivale a −19.23° en el modelo de bicicleta |
| Proyección frontal de aproximación | 30 cm | Regla fija del planner; se mide desde el borde frontal del carro |
| Cámara | Logitech C922 Pro Stream Webcam | Confirmado por el equipo |
| FOV diagonal de cámara | 78° | Especificación de Logitech aportada por el equipo |
| FOV horizontal usado | 70.4° | Derivado geométricamente para relación 16:9 |

El mapa usa centímetros como coordenadas internas. La escala inicial es 2 px/cm.
El campo reproduce el plano aportado: el muro interno está a 100 cm de cada
borde externo y sus picos/esquinas son (100,100), (200,100), (100,200) y
(200,200) cm; por ello queda centrado. El panel de 80 cm se dibuja dentro de
ese muro. Ocho líneas diagonales llegan a dichos picos.
Las diagonales de esquina se calculan con los ángulos de 30 grados indicados.
El modo que se valida aquí es el corredor de obstáculos de 1000 mm entre muros;
en este mapa equivale a 100 cm por lado entre el muro interior y el exterior.
Los obstáculos de prueba y los añadidos con `1`/`2` se ajustan al centro de un
asiento válido; cada asiento se dibuja con su círculo de evaluación.
El patrón de asientos se repite en las cuatro rectas: 3 posiciones a lo largo
del tramo y 2 en su anchura, para 24 asientos en total.
Por cada tramo longitudinal de 500 mm se admite un solo pilar; la escena fija
coloca dos pilares por lado en tramos distintos.
El espesor de los muros no está especificado por WRO, por lo que sus colisiones
se evalúan contra el límite geométrico, sin asignarles grosor ficticio.

## Datos que siguen pendientes

- La equivalencia completa entre cada comando intermedio del servo y los dos
  ángulos de rueda. Los extremos izquierdo/derecho sí están medidos y el
  simulador conserva su asimetría; la interpolación intermedia debe validarse.
- FOV **horizontal** efectivo en el montaje final. El valor inicial de 70.4°
  se deriva de 78° diagonales con 16:9 (1080p/30 fps o 720p/60 fps); debe
  sustituirse con `--fov-deg` si una calibración óptica lo corrige.
- Radio de las esquinas redondeadas visibles del panel central. No se usa para
  colisión: el muro se representa como el cuadrado de 100 cm definido por los
  picos, sin inventar un radio.
- Espesor de muros, calibración de velocidad, holgura mecánica y margen de
  seguridad validado físicamente. El margen inicial se elige con
  `--safety-margin-cm`; no es una medida oficial.

## Estructura

```text
simulator/
  wro_simulator.py     # escena, render Pygame e integración del planner
  track_config.py      # fuente única de pista, salida, ruta y asientos
  planner_rules.py     # FixedRules: reglas físicas y del reto
  planner_tuning.py    # PlannerTuning: parámetros editables y carga JSON
  scenario.py          # Scenario y obstáculos compartidos
  geometric_planner.py # geometría, Ackermann, primitivas, swept collision y scoring
  autonomous_controller.py # PlannerInput -> PlannerResult y commitment
  simulator_adapter.py # percepción simulada y aplicación del ControlCommand
  planner_test_runner.py # escenarios headless y registros JSON/CSV
  test_geometric_planner.py # pruebas unitarias estándar
  requirements.txt     # dependencia exclusiva del simulador
  README.md            # medidas, límites y ejecución

config/simulator_steering_calibration.json  # generado con muestras de giro
config/simulator_manual_runs/                # recorridos manuales JSON + CSV
config/simulator_planner_tuning.json         # parámetros editables del planner
```

El modo automático usa tres capas:

- `geometric_planner.py`: dataclasses, geometría, cinemática Ackermann,
  primitivas, simulación swept, validación y scoring.
- `autonomous_controller.py`: interfaz pura `PlannerInput -> PlannerResult` y
  commitment/hysteresis. No importa Pygame ni hardware.
- `simulator_adapter.py`: convierte pose, FOV, obstáculos, muros y dirección
  detectada por las líneas en un `PlannerInput`, y aplica el `ControlCommand`
  al vehículo simulado.

Las primitivas disponibles son `STRAIGHT`, `REVERSE`, `ARC_LEFT` y
`ARC_RIGHT`. Si la proyección recta completa es segura, el planner no genera
maniobras innecesarias. Si está bloqueada, construye perfiles
`CONSERVATIVE`, `NOMINAL` y `TIGHT` con radios derivados de las medidas
físicas. Cada candidato se integra en pasos de `simulation_dt_s`; en cada pose
se valida el rectángulo rotado completo contra obstáculos, muros, límite de
pista y clearance obligatorio.

El planner solo utiliza `visible_obstacles` y `visible_walls` entregados por
la capa de percepción. El sentido `CLOCKWISE` o `COUNTERCLOCKWISE` también
llega en `PlannerInput`; el controller nunca procesa imágenes ni colores de
líneas. Los colores rojo/verde de obstáculos sí se conservan como una propiedad
semántica ya detectada para exigir rojo por la derecha y verde por la izquierda.

Los límites asimétricos se toman de `config/physical_measurements.json`:
radio mínimo derecho de 32.2 cm, izquierdo de 43.0 cm y ángulos medidos de cada
rueda. No se duplican esas medidas en el planner.
## Ejecutar en VS Code

1. Abre la carpeta raíz `carrito7515` en VS Code y selecciona un intérprete de
   Python 3.10 o superior.
2. En la terminal integrada crea y activa un entorno virtual:

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   python3 -m pip install -r simulator/requirements.txt
   ```

3. El límite de ruedas usado es ±20.7° medido/provisional y el FOV horizontal es
   70.4°. Inicia el simulador:

   ```bash
   python3 simulator/wro_simulator.py
   ```

Controles: `↑` para avance manual, `↓` para retroceso manual y `←`/`→` para
steering. `A` activa/desactiva automático,
`R` reinicia, `1` selecciona la pista sin obstáculos, `2` selecciona la pista
con obstáculos y `Esc`
sale. `[`/`,` reduce y `]`/`.` aumenta en 1° el comando lógico de servo, entre
60° y 120°; inicia en 90°. La escena fija contiene dos pilares por cada lado
de la pista; sus posiciones son una configuración de prueba, no posiciones
fijas del reglamento.

El carro inicia centrado en `(125, 280)` cm dentro de la zona inferior izquierda
de salida `(100..150, 260..300)` cm, mirando hacia la esquina inferior
izquierda. Al activar `A`, avanza hasta detectar la primera línea: naranja fija
sentido horario y azul fija antihorario. `O` y `B` permiten forzar esas mismas
lecturas durante una prueba. El sentido se muestra en el panel y queda incluido
en cada punto de `trajectory`.

Pulsa `1` para reiniciar el escenario sin obstáculos o `2` para reiniciar el
escenario con los ocho obstáculos configurados en la pista. Pulsa `C` mientras
pruebas una dirección para guardar la muestra en
`config/simulator_steering_calibration.json`: fecha, comando lógico de servo
seleccionado, ángulo de rueda, radio de giro y pose. Cada muestra incluye además
`trajectory`, el historial completo desde `R` de cada frame: tiempo simulado,
posición/orientación/velocidad del carro, estado de la FSM y la posición,
color y estado de todos los obstáculos. El comando de servo se registra como
referencia: el simulador no inventa su equivalencia con el ángulo de rueda.
Úsalo como tabla de calibración; no sustituye una prueba física segura ni
escribe en el firmware.

### Recorrido manual y datos para calibración

Para registrar una prueba completa, pulsa `R`, mantén el modo `MANUAL`, conduce
con las flechas y pulsa `S` al terminar. Cada frame guarda pose, ángulos real y
objetivo de las ruedas, comando lógico de servo, velocidad, aceleración,
distancias al obstáculo y a cada muro, estados `VISIBLE`/`TEMPORARILY_LOST`/
`PASSED`/`STALE` y la decisión registrada. Se generan dos archivos en
`config/simulator_manual_runs/`:

- `manual_run_*.json`: registro detallado y metadatos.
- `manual_run_*.csv`: una fila por frame, listo para pandas, Excel o un ajuste
  de parámetros.

El panel muestra el número de frames y el nombre del último archivo guardado.
`C` sigue guardando una muestra de calibración del steering en el archivo JSON
histórico; `S` es la opción recomendada para registrar el recorrido completo.

La sección `calibration_status` conserva cuál muestra debe usarse como
referencia y si falta validación física, incluso después de guardar muestras
nuevas.

La muestra manual 10 permanece guardada como evidencia de la vuelta completa,
pero ya no se reproduce como una ruta: sus correcciones ondeantes no son un
objetivo autónomo. Los dos escenarios usan una ruta geométrica redondeada por
el centro del corredor de las cuatro rectas. El modo automático usa el máximo
físico confirmado de `20.7°` por lado.
Los obstáculos de la recta inferior de salida están activos desde el primer
frame. Si aparecen dentro del FOV, se seleccionan con la misma regla de color
(`green → izquierda`, `red → derecha`) y participan en la colisión de todo el
cuerpo. Las trayectorias normales siempre avanzan a la velocidad fija. El
único retroceso permitido es la secuencia de recuperación de seguridad
`BRAKE → REVERSE_10CM → FULL_TURN → ADVANCE_AVOID`; no existe una regla que
ignore obstáculos por estar en la salida.
La primera diagonal únicamente bloquea el sentido. Los siguientes giros no se
ejecutan por tiempo: se anticipan sobre la ruta redondeada, con puntos de
seguimiento que mantienen el cuerpo completo dentro del corredor entre muros.
La FSM aplica la regla de esquive `green → izquierda` y `red → derecha`, usando
la posición estimada del obstáculo detectada por cámara para construir un
desvío lateral con margen de seguridad.
El sentido queda bloqueado por la primera línea: una detección naranja fija
`horario` y una azul fija `antihorario`; no se vuelve a cambiar durante esa
vuelta.

El orden de rectas también queda bloqueado. Horario exige
`bottom → left → top → right → bottom`; antihorario exige
`bottom → right → top → left → bottom`. Si el carro entra en una recta que no
corresponde al siguiente paso, pasa a `EMERGENCY_STOP` en vez de aceptar un
cambio de sentido. El signo del primer giro se deriva del lado obligatorio
(`-` izquierda, `+` derecha), pero el contragiro puede usar el signo opuesto
sin invalidar la maniobra.

Un obstáculo próximo a una esquina no cancela la ruta: el lado de paso es la
restricción inmediata y la tangente del siguiente tramo es el objetivo
posterior. La puntuación favorece las trayectorias que, después de superar el
obstáculo, quedan preparadas para la siguiente recta o esquina.

Cada registro de trayectoria incluye `planning_phase` con la secuencia. Las
fases no tienen duraciones base: `duration_s` solo representa una ranura de
integración del horizonte y nunca autoriza avanzar de fase.

1. `TURN_OUT`: termina cuando la carrocería alcanza el desplazamiento lateral
   requerido.
2. `PASS_HOLD`: termina cuando el borde trasero completo supera el obstáculo.
3. `COUNTER_STEER`: termina cuando el heading vuelve a acercarse a la
   tangente de pista.
4. `REJOIN_CENTER`: termina cuando el carro entra al corredor objetivo y queda
   alineado.

El contragiro puede tener el signo opuesto al giro inicial. Eso no cambia el
lado reglamentario: rojo se cruza por la derecha y verde por la izquierda; el
cumplimiento se determina por la posición geométrica del rectángulo del carro
respecto al obstáculo, no por el signo instantáneo del steering.

Si el frente del rectángulo del carro alcanza el `mandatory_clearance` de un
obstáculo o muro, se activa una recuperación determinista independiente de las
cuatro fases normales:

1. `BRAKE`: orden de velocidad objetivo cero hasta detener la velocidad actual.
2. `REVERSE_10CM`: retroceso controlado de aproximadamente 10 cm, con el
   steering en el signo opuesto al giro frontal elegido. Así el desplazamiento
   lateral de la marcha atrás abre espacio para el giro delantero.
3. `FULL_TURN`: steering completo hacia el lado fijado por el color del
   obstáculo. Si la causa es un muro, el giro se calcula con la dirección de
   pista detectada en la primera línea; no se fuerza siempre el sentido horario.
   Si el obstáculo aparece durante `BRAKE` o `REVERSE_10CM`, su lado obligatorio
   (`green → izquierda`, `red → derecha`) reemplaza el lado provisional del muro.
4. `ADVANCE_AVOID`: avance con ese giro hasta abrir el corredor y confirmar el
   paso. Si el frente vuelve a entrar en el límite crítico, la secuencia
   empieza otra vez en `BRAKE`.

El estado visible durante esta secuencia es `SAFETY_RECOVERY`. El retroceso es
una orden negativa válida; no se interpreta como `EMERGENCY_STOP`. El registro
JSON/CSV incluye `recovery_phase`, `recovery_side`, `front_clearance_cm`,
`recovery_reverse_distance_cm` y `recovery_advance_distance_cm`.

En una pista horaria, el adaptador Pygame añade además la secuencia de líneas
de cambio de recta: `ALIGN_TO_LINE` busca la tangente de entrada con el frente
del carro sin retroceder; al cruzar la línea con el frente completo activa
`FULL_TURN_LINE` en el sentido detectado (naranja/horario o azul/antihorario);
cuando el heading queda alineado con la siguiente recta, vuelve al seguimiento
normal. La evasión de obstáculos y la recuperación de seguridad tienen
prioridad sobre esta corrección de ruta.

`PASSED` se confirma con la proyección del borde trasero completo de la
carrocería sobre la tangente local, no con la desaparición visual del pilar.
Ese estado pertenece a una vuelta concreta: al cambiar `PlannerInput.lap_index`
se limpia la memoria de obstáculos superados para que el siguiente recorrido
vuelva a planificar cada obstáculo.
Una colisión física, los márgenes `hard` y los márgenes `preferred` son métricas
distintas. La configuración inicial es `hard: front=6, side=3, rear=3 cm` y
`preferred: front=12, side=5, rear=5 cm`. Si ningún candidato cumple todas las
restricciones, el estado queda en `NO_SAFE_TRAJECTORY`, pero el carro no se
detiene por ese diagnóstico: mientras la pose actual no tenga colisión física,
continúa con una orden de continuidad sin colisión prevista y conserva la
velocidad fija. La parada queda reservada para una colisión física actual o una
emergencia explícita.

La adquisición inicial de obstáculos usa únicamente el FOV horizontal de la
cámara configurado en el simulador. No existe un corredor lateral invisible ni
se simulan sensores ultrasónicos. Después de una detección válida por cámara,
la FSM conserva la última posición estimada aunque el obstáculo salga del FOV.
La cámara solo aporta objetos dentro del FOV; la geometría y la tangente local
deciden si el objeto está delante y puede ser objetivo. No existe un corredor
lateral invisible ni sensor de alcance adicional.
La vuelta solo se marca como completa cuando el carro ha entrado en las cuatro
rectas (`top`, `right`, `bottom` y `left`) y luego regresa a la misma zona/recta
de salida; una colisión detiene la prueba y no cuenta como vuelta válida.
`--max-steering-deg` permite probar otro límite explícitamente. Esto ajusta
solo el modelo del simulador; el centro lógico confirmado del servo físico es
`92°`.

## Datos del carro que faltan para mayor precisión

- Modelo exacto del servomotor y geometría del brazo de dirección.
- Ángulo real de cada rueda en toda la carrera, incluyendo histéresis al mover
  desde izquierda y desde derecha.
- Posición del eje de giro de cada rueda delantera y radio de giro medido.
- Convergencia/divergencia (`toe`), caída (`camber`) y avance (`caster`) de las
  ruedas; actualmente el modelo de bicicleta no representa esas inclinaciones.
- Ancho trasero real, voladizo delantero/trasero y centro de masa/altura.
- Tiempo de respuesta del servo, velocidad real, aceleración y frenado.

Con los datos actuales el rectángulo usa 21.15 cm de largo, 17.2 cm de ancho
de envolvente de ruedas y 15.0 cm de distancia entre ejes. Las inclinaciones
por rueda están registradas, pero todavía falta el radio real de giro para
integrarlas en el modelo de bicicleta.

## Planner geométrico y pruebas headless

`geometric_planner.py` no importa Pygame y expone `PlannerInput` y
`ControlCommand`. Pygame solo recoge entradas, construye `PlannerInput`, aplica
el `ControlCommand`, renderiza y muestra diagnosticos; no sustituye el
steering decidido por el planner. Usa un `simulation_dt` de `0.05 s`, un periodo de
replanificación independiente de `0.20 s`, un horizonte de validación de
`2 s` y un rango visual configurable de `5 s` (`--preview-horizon-s`). Las
líneas extendidas se dibujan tenue y no convierten una previsión en una
trayectoria segura. También aplica aceleración y desaceleración limitadas, y
un máximo de `256` candidatos o `20 ms` por ciclo. La magnitud de velocidad
objetivo de recorrido es única y positiva (`fixed_speed_cm_s`, por defecto
`24 cm/s`) en las cuatro fases y en los cambios de recta. Las únicas
excepciones son `BRAKE` (cero) y `REVERSE_10CM` (negativa) dentro de la
recuperación de seguridad. El punto de
referencia es el centro geométrico; los offsets simétricos de `7.4 cm` a cada
eje son una hipótesis del simulador hasta contar con una medición física.

Cuando hay un obstáculo cerca de un muro, `minimum_corridor_clearance_cm` es la
holgura mínima simultánea entre la carrocería completa, cualquier muro y
cualquier obstáculo alcanzado por la trayectoria. El lado obligatorio se valida
para cada obstáculo, no solo para el objetivo activo. Si la holgura baja del
margen deseado pero todavía no existe colisión física, el planner conserva una
orden de avance validada; no crea una maniobra de retroceso adicional. El
retroceso solo aparece cuando la zona frontal de seguridad activa la secuencia
`BRAKE → REVERSE_10CM → FULL_TURN → ADVANCE_AVOID`. La detención queda
reservada para `BRAKE`, una colisión física actual o una emergencia.

Las pruebas unitarias se ejecutan con:

```bash
python3 -m unittest discover -s simulator -p 'test_*.py' -v
```

El runner usa exactamente `track_config.py`, igual que Pygame: misma salida,
misma ruta, mismo muro y mismos 24 asientos. Puede generar obstáculos en todos
los asientos válidos y no descarta un obstáculo verde en una recta vertical por
considerarlo "imposible"; todo objeto entregado por el sensor debe ser resuelto
por la regla fija `red -> derecha` / `green -> izquierda`. El tiempo por defecto
es `20 s`, suficiente para alcanzar la siguiente recta. Para validar el objetivo
actual de una vuelta completa usa `--duration-s 60` o más. El resumen incluye
`straight_progress`, `lap_completed`, `minimum_corridor_clearance_cm`,
`scenarios_reaching_next_straight` y
`route_progress_valid`, además de guardar resultados reproducibles en JSON/CSV
con P50, P90, P95, P99 y máximo del tiempo de planificación:

```bash
python3 simulator/planner_test_runner.py \
  --scenarios 100 \
  --seed 20260815 \
  --output-dir /tmp/wro-planner-results
```

El modo de incertidumbre es opcional:

```bash
python3 simulator/planner_test_runner.py \
  --noise-position-cm 1.0 \
  --noise-heading-deg 2.0 \
  --latency-s 0.15 \
  --dropout-probability 0.10
```

Para comparar parámetros de forma reproducible usa el barrido separado:

```bash
python3 simulator/planner_parameter_sweep.py \
  --scenarios 20 \
  --seed 20260815 \
  --fixed-speed-cm-s 18 \
  --output-dir /tmp/wro-planner-sweep
```

El barrido prueba combinaciones de margen obligatorio/deseado, periodo de
replanificación, horizonte de validación y velocidad de cambio del steering.
También puede comparar varias velocidades fijas con `--fixed-speed-cm-s 15,18,21`.
La aceleración y la desaceleración también aceptan listas con
`--acceleration-cm-s2` y `--deceleration-cm-s2`.
Incluye `lap_rate` en la puntuación y guarda `sweep_results.json`,
`sweep_results.csv` y los resúmenes por escenario.
La selección es lexicográfica en la práctica: primero se penalizan las
colisiones, después se priorizan progreso válido, maniobras completadas y paso
por el lado correcto; `NO_SAFE_TRAJECTORY` y el P95 de cálculo sirven como
desempate. La configuración ganadora solo es una configuración del simulador:
debe repetirse con más semillas y ruido antes de considerarla estable, y no se
conecta automáticamente al robot físico.

### Separación de reglas, tuning y escenarios

Las reglas no optimizables están en `planner_rules.py`: dimensiones físicas,
muros, FOV, lado rojo/verde, timestep y las cuatro fases. Los parámetros de
comportamiento están en `config/simulator_planner_tuning.json`. Puedes editar
allí los ángulos, márgenes y límites dinámicos sin tocar
`geometric_planner.py`:

```json
{
  "turn_angles_deg": [15, 10, 5, 0],
  "counter_steer_angles_deg": [15, 10, 5, 0],
  "safety_margins": {
    "hard": {"front_cm": 6, "side_cm": 3, "rear_cm": 3},
    "preferred": {"front_cm": 12, "side_cm": 5, "rear_cm": 5}
  }
}
```

El archivo se puede usar en Pygame y en el runner:

```bash
.venv/bin/python simulator/wro_simulator.py \
  --planner-config config/simulator_planner_tuning.json

.venv/bin/python simulator/planner_test_runner.py \
  --planner-config config/simulator_planner_tuning.json
```

Los escenarios se generan desde `scenario.py`; Pygame y el runner utilizan la
misma definición de asientos, colores, dimensiones y salida.

### Calibración recomendada desde cero

1. Primero calibra sin obstáculos y cambia una variable a la vez. Por ejemplo,
   prueba la velocidad fija y el límite real de dirección en Pygame:

   ```bash
   .venv/bin/python simulator/wro_simulator.py \
     --fixed-speed-cm-s 18 \
     --max-steering-deg 20.7 \
     --max-steering-rate-deg-s 90 \
     --replanning-period-s 0.20
   ```

2. Después valida obstáculos con una sola configuración base en el runner:

   ```bash
   .venv/bin/python simulator/planner_test_runner.py \
     --scenarios 20 \
     --duration-s 60 \
     --fixed-speed-cm-s 18 \
     --seed 20260815 \
     --output-dir /tmp/wro-base
   ```

3. Finalmente compara variaciones reproducibles. Cada conjunto queda guardado
   en `sweep_results.json` dentro de `--output-dir`, incluyendo todos sus
   parámetros y métricas:

   ```bash
   .venv/bin/python simulator/planner_parameter_sweep.py \
     --scenarios 20 \
     --duration-s 60 \
     --fixed-speed-cm-s 15,18,21 \
     --mandatory-clearance-cm 8,10,12 \
     --desired-clearance-cm 13,15,18 \
     --replanning-period-s 0.15,0.20,0.25 \
     --planning-horizon-s 2,3 \
     --steering-rate-deg-s 75,90,110 \
     --acceleration-cm-s2 35,45 \
     --deceleration-cm-s2 60,70 \
     --seed 20260815 \
     --output-dir /tmp/wro-sweep
   ```

No se deben variar durante la búsqueda las reglas fijas: dimensiones del carro,
geometría de la pista, FOV medido, `red -> derecha`, `green -> izquierda` y la
prohibición de ejecutar una trayectoria con colisión física. Elige la mejor
configuración por cero colisiones, vueltas completas y paso correcto; usa el
tiempo de planificación solo como desempate.
