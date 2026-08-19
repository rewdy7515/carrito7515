# Contexto oficial WRO 2026 – Future Engineers

Fuente consultada:

[WRO 2026 Future Engineers – Reglas generales, versión en español (PDF)](https://wrovenezuela.org.ve/wp-content/uploads/2026/05/ESPANOL-WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules-Final.pdf)

Este documento resume las reglas que afectan directamente al diseño mecánico,
la visión, la navegación y el control del robot. El PDF oficial y las
preguntas y respuestas (Q&A) de WRO tienen prioridad sobre este resumen.
También deben verificarse las posibles adaptaciones del organizador nacional.

## 1. Misión

La competencia tiene dos retos:

### Reto Abierto

- Completar tres vueltas de forma autónoma.
- La posición de los muros interiores cambia aleatoriamente.
- La distancia entre los bordes de la pista puede ser de 1000 mm o 600 mm,
  con una tolerancia de ±100 mm en la Final Internacional.
- No hay señales de tránsito.
- El robot debe respetar la dirección de circulación definida para la ronda.
- Después de tres vueltas debe regresar a la sección de arranque/meta y detenerse
  autónomamente dentro de ella para obtener los puntos correspondientes.

### Reto con Obstáculos

- Completar tres vueltas de forma autónoma.
- La distancia entre los bordes de la pista es de 1000 mm (±10 mm en la Final
  Internacional).
- Se colocan pilares rojos y verdes en posiciones aleatorias.
- El pilar rojo debe rebasarse por la derecha.
- El pilar verde debe rebasarse por la izquierda.
- Después de las tres vueltas, el robot debe detenerse en la sección correcta o
  realizar el estacionamiento.
- El cajón de estacionamiento siempre se coloca en la sección de arranque.

Ambos retos tienen un tiempo máximo de tres minutos por ronda.

## 2. Configuración del campo

El campo tiene ocho secciones:

- cuatro secciones de curva;
- cuatro secciones rectas.

Cada sección recta se divide en seis zonas internas que pueden utilizarse como
posiciones iniciales. Los asientos de las señales se organizan mediante:

- cuatro intersecciones en forma de `T`;
- dos intersecciones en forma de `X`.

La dirección de circulación, la posición de arranque y la configuración del
campo se determinan antes de cada ronda y permanecen iguales para todos los
equipos durante esa ronda.

## 3. Inicio de una ronda

El comportamiento esperado del software es:

1. El robot se coloca completamente dentro de la zona de arranque y apagado.
2. Las dos ruedas del eje delantero deben quedar orientadas hacia la siguiente
   sección de curva en la dirección de circulación.
3. El robot se enciende mediante un único interruptor.
4. El sistema entra en estado de espera.
5. El robot espera un único botón `Start`.
6. Al presionar `Start`, comienza el movimiento autónomo y el cronómetro.

No se permite usar ajustes físicos, interruptores o cambios de orientación para
introducir datos de la configuración de la ronda después de la revisión técnica.

## 4. Reglas de movimiento y autonomía

- El robot debe ser completamente autónomo durante la misión.
- No se permite control remoto ni comunicación inalámbrica durante el
  funcionamiento de competencia.
- Las comunicaciones entre componentes deben ser cableadas.
- La Raspberry Pi y el Arduino pueden comunicarse por USB serial cableado.
- El robot debe seguir la dirección de circulación definida para la ronda.
- Puede circular en sentido contrario únicamente en la sección donde cambia la
  dirección y en la sección vecina, respetando los límites definidos por las
  reglas.
- La conducción de atrás hacia adelante está permitida cuando el robot continúa
  desplazándose en la dirección de circulación de la ronda.

## 5. Señales de tránsito

Cada señal es un paralelepípedo de:

```text
50 × 50 × 100 mm
```

Puede haber hasta siete señales rojas y hasta siete verdes.

| Señal | RGB oficial | Acción |
|---|---|---|
| Roja | (238, 39, 55) | Rebasar por la derecha |
| Verde | (68, 214, 44) | Rebasar por la izquierda |

El robot puede tocar, mover o derribar una señal únicamente si la proyección de
la señal permanece dentro del círculo dibujado alrededor de su asiento. Moverla
fuera de ese círculo termina la ronda con penalización.

Durante el trayecto posterior hacia el estacionamiento, después de las tres
vueltas oficiales, las señales pueden rebasarse por cualquier lado, pero no
pueden moverse fuera de su círculo.

## 6. Cajón de estacionamiento

El cajón se coloca en la sección de arranque del Reto con Obstáculos.

```text
ancho del cajón:    200 mm
longitud del cajón: 1.5 × longitud del robot
```

Está delimitado por dos piezas magenta de:

```text
200 × 20 × 100 mm
```

El estacionamiento es completo cuando:

- la proyección completa del robot queda dentro del rectángulo del cajón;
- el robot queda paralelo al muro del campo;
- la diferencia entre las distancias de las dos ruedas del mismo lado al muro
  no supera 20 mm;
- el robot no toca los delimitadores.

Para la visión y el control, el estacionamiento requiere estimar tanto la
posición del robot dentro del cajón como su orientación respecto al muro.

## 7. Final de la ronda

### Reto Abierto

Después de tres vueltas, el robot debe detenerse autónomamente dentro de la
sección de meta. Si continúa moviéndose durante demasiado tiempo, la detención
puede considerarse ambigua.

### Reto con Obstáculos

Después de completar correctamente tres vueltas, la ronda puede terminar cuando
el robot se detiene en la sección correcta o en el cajón de estacionamiento.

Para obtener puntos adicionales, el robot debe regresar a la sección de
arranque después de las tres vueltas.

## 8. Requisitos del vehículo que afectan al software

| Requisito | Límite o condición |
|---|---|
| Dimensiones máximas | 300 × 200 mm y 300 mm de altura |
| Peso máximo | 1,5 kg |
| Configuración | Cuatro ruedas |
| Tracción | Un eje motriz; se permiten hasta dos motores de tracción |
| Dirección | Actuador de dirección |
| Tracción diferencial | No permitida |
| Ruedas omnidireccionales/caster | No permitidas |
| Sensores | Marca, tipo y cantidad libres |
| Cámara | Permitida como sensor |
| Controlador | SBC o microcontrolador, sin marca específica |
| Lenguaje | Sin restricción específica |

El sistema actual Raspberry Pi + Arduino Uno es compatible con el modelo de
controlador permitido, siempre que la comunicación usada durante la ronda sea
cableada y el robot opere de forma autónoma.

## 9. Implicaciones para la arquitectura del software

La máquina de estados debe incluir como mínimo:

```text
POWER_ON
WAIT_START
CALIBRATION_READY
DETECT_DIRECTION
FOLLOW_TRACK
HANDLE_RED_SIGN
HANDLE_GREEN_SIGN
COUNT_LAP
RETURN_TO_START
LOCATE_PARKING
PARK_PARALLEL
STOPPED
EMERGENCY_STOP
```

La visión debe distinguir entre:

- secciones rectas y curvas;
- dirección de circulación;
- muros interiores y exteriores;
- líneas de referencia azules y naranjas;
- pilares rojos y verdes;
- asientos y círculos de señales;
- delimitadores magenta del estacionamiento;
- sección de arranque/meta.

El controlador debe mantener una separación suficiente de los muros y señales,
pero no puede asumir una única anchura de pista en el Reto Abierto.

## 10. Restricciones de competencia y documentación

- El código y la construcción deben ser trabajo propio del equipo.
- Los jueces pueden revisar el programa del vehículo.
- El repositorio público debe contener el código de todos los componentes
  programados para la competencia.
- Para la Final Internacional, la documentación de GitHub debe estar en inglés.
- El repositorio debe documentar la movilidad, potencia, sensores, gestión de
  obstáculos, arquitectura de software y proceso de compilación/ejecución.
- Las reglas pueden recibir aclaraciones mediante el Q&A oficial de WRO.
