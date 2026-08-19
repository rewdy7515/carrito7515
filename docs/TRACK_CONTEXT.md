# Contexto de la pista y sus obstáculos

Este documento reúne las dimensiones, colores y restricciones de la pista
proporcionadas para el proyecto. Las medidas están expresadas en milímetros,
salvo que se indique otra unidad.

## 1. Mesa de juego y pista

| Elemento | Especificación |
|---|---|
| Tamaño total del tapete | 3200 × 3200 mm (±5 mm) |
| Tamaño interior de la pista | 3000 × 3000 mm (±5 mm) |
| Color principal de la pista | Blanco |
| Muros exteriores | Altura interior de 100 mm |
| Color interior de los muros exteriores | Negro |
| Color exterior de los muros exteriores | No definido |
| Muros interiores adicionales | Rodean la sección interna de la pista |
| Altura de los muros interiores | 100 mm |
| Color exterior de los muros interiores | Negro |
| Color interior de los muros interiores | Negro |
| Color del borde superior de los muros interiores | Negro |
| Espesor de los muros | No definido |
| Distancia entre muros exteriores e interiores | Depende del tipo de ronda |

La distancia entre los muros exteriores e interiores debe tratarse como una
variable de la pista. El robot no debe depender de una única configuración
fija.

## 2. Líneas y marcas de la pista

| Marca | Especificación |
|---|---|
| Líneas naranjas | Grosor de 20 mm; CMYK (0, 60, 100, 0) |
| Líneas azules | Grosor de 20 mm; CMYK (100, 80, 0, 0) |
| Líneas punteadas de arranque | Grosor de 1 mm; CMYK (0, 0, 0, 30) |
| Zonas de salida representadas en el plano del equipo | 500 × 400 mm (configuración vigente del simulador) |
| Líneas de los asientos de señales | Grosor de 1 mm; CMYK (0, 0, 0, 30) |
| Tamaño de cada asiento de señal | 50 × 50 mm |
| Círculo de evaluación de señal movida | Diámetro de 85 mm; grosor de 0,5 mm; CMYK (20, 0, 100, 0) |

Las líneas naranjas y azules son referencias visuales para la navegación. La
conversión de estos colores a umbrales de OpenCV debe validarse con imágenes
tomadas bajo la iluminación real de la pista.

Los cuadros de **50 × 50 mm** son los asientos válidos para los pilares rojos
y verdes. Cada pilar debe quedar centrado en uno de esos cuadros. El círculo
de **85 mm de diámetro** que rodea el asiento delimita el área de evaluación:
mover el pilar fuera de ella termina la ronda con penalización.

### Patrón repetido de asientos

El plano suministrado define un mismo patrón para las cuatro secciones rectas:
seis asientos por sección, organizados en una matriz de **3 × 2**. La anchura
de cada sección es 1000 mm, distribuida como **400 + 200 + 400 mm**: los dos
asientos quedan en las posiciones transversales de 400 y 600 mm respecto al
borde de la sección. A lo largo de cada tramo de 1000 mm, las tres posiciones
son 0, 500 y 1000 mm respecto al inicio del tramo. El simulador usa las 24
posiciones resultantes (seis por cada lado del bloque central), todas con
asiento de 50 × 50 mm y círculo de evaluación de 85 mm.

El plano también confirma estas cotas visuales: panel central de 800 mm,
líneas azul/naranja de 20 mm a 30°, guía amarilla de 3 mm y líneas punteadas
de los asientos de 1 mm. Los picos de las diagonales azul y naranja son las
esquinas del muro interno: (1000, 1000), (2000, 1000), (2000, 2000) y
(1000, 2000) mm, medidas desde el borde externo de la pista. Por tanto, se
encuentra a 1000 mm de cada borde externo, queda centrado y su contorno
geométrico es de 1000 × 1000 mm; el panel de 800 mm queda dentro y no se usa
para las colisiones. No se asigna un radio redondeado sin cota.

En una misma sección recta, cada posición longitudinal de 500 mm admite **un
solo pilar**: no se pueden ocupar a la vez los dos asientos transversales del
mismo tramo. La escena fija del simulador coloca dos pilares por lado, en los
tramos 0 y 1000 mm de cada sección, dejando libre el tramo central de 500 mm.

## Dirección de circulación

La primera línea de referencia detectada define el sentido de la ronda:

| Primera línea | Sentido de circulación |
|---|---|
| Naranja | Horario |
| Azul | Antihorario |

El simulador permite marcar esta detección con `O` (naranja) o `B` (azul), la
muestra en el panel y la guarda en el historial de la trayectoria.

## Zona de salida fija del simulador

El simulador inicia en la zona inferior izquierda de salida de **500 × 400
mm**, con límites `(1000, 2600)` a `(1500, 3000)` mm. El centro del carro se
coloca en `(1250, 2800)` mm y apunta hacia la esquina inferior izquierda. Estos
son los valores vigentes de `simulator/track_config.py` y deben usarse tanto en
Pygame como en el runner headless. En modo automático avanza recto desde allí
hasta detectar con el frente la primera línea: naranja selecciona horario y
azul selecciona antihorario. `O` y `B` siguen disponibles como anulación manual
para pruebas.

## 3. Señales de tránsito

Las señales son paralelepípedos rectangulares verticales con:

```text
ancho:      50 mm
profundidad: 50 mm
altura:    100 mm
```

En cada ronda puede haber aleatoriamente:

- hasta 7 señales rojas;
- hasta 7 señales verdes.

| Señal | Color RGB de referencia |
|---|---|
| Roja | (238, 39, 55) |
| Verde | (68, 214, 44) |

El material y el peso de las señales no están definidos.

Para el procesamiento con OpenCV, estos colores deben considerarse valores de
referencia y no valores exactos garantizados por la cámara. La iluminación,
el balance de blancos y las sombras pueden alterar los valores capturados.

## 4. Cajón de estacionamiento

En cada ronda del Reto con Obstáculos se coloca un cajón de estacionamiento
con dos delimitadores.

Cada delimitador es un paralelepípedo rectangular de:

```text
longitud: 200 mm
espesor:   20 mm
altura:   100 mm
```

| Elemento | Especificación |
|---|---|
| Cantidad de delimitadores | 2 por ronda del Reto con Obstáculos |
| Color | Magenta |
| Color RGB de referencia | (255, 0, 255) |
| Material | No definido |
| Peso | No definido |

El robot debe localizar el cajón y realizar un estacionamiento paralelo sin
salirse del área ni tocar sus delimitadores.

## 5. Implicaciones para la visión artificial

El módulo `software/raspberry_pi/src/camera.py` debe poder producir, como
mínimo, las siguientes detecciones:

- líneas azules;
- líneas naranjas;
- regiones negras correspondientes a muros;
- señales rojas;
- señales verdes;
- delimitadores magenta del estacionamiento;
- zonas de arranque y marcas auxiliares cuando sean visibles.

La detección de color por sí sola no confirma la distancia física ni si una
señal fue movida. Para estimar distancias se necesitará calibrar la cámara
con la altura, el ángulo vertical, la resolución y la perspectiva reales.

## 6. Datos pendientes de confirmar

- Espesor real de los muros exteriores e interiores.
- Color exterior de los muros exteriores.
- Distancia exacta entre muros para cada alternativa de ronda.
- Iluminación y condiciones de captura.
- Altura y ángulo vertical definitivos de la cámara.
- Relación entre píxeles y milímetros para cada zona de la imagen.
- Tolerancia práctica de los colores RGB/CMYK bajo la cámara utilizada.
- Geometría exacta del cajón de estacionamiento en cada configuración.
