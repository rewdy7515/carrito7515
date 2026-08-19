# Calibración manual de visión

La cámara y los controles de OpenCV se muestran en la misma página:

`http://pirobot.local:8000`

Debajo de las vistas original y procesada aparece **Ajuste manual de visión**.
Los cambios se aplican al siguiente análisis y se guardan en
`config/vision_tuning.json`, por lo que permanecen después de reiniciar el
programa.

Controles iniciales:

- `Distancia segura`: solo cambia la etiqueta y la referencia visual de la
  franja; debe calibrarse con la geometría real de la cámara.
- `Área mínima de señal`: aumenta este valor para eliminar cuadros pequeños.
- `Umbral muro negro`: controla qué tan oscuro debe ser un muro para detectarse.
- Para rojo y verde se pueden ajustar la tolerancia de tono, la saturación
  mínima y el brillo mínimo.

Primero se recomienda subir `Área mínima de señal` hasta que desaparezcan los
falsos positivos y después bajar gradualmente el `brillo mínimo` del verde si
el pilar no se detecta bajo sombra.

## Distancia estimada de las señales

El autónomo usa el alto conocido de la señal, `100 mm`, y su alto aparente en
píxeles:

```text
distancia_mm = foco_equivalente_px × 100 / alto_de_la_señal_px
```

El foco equivalente inicial es `240 px` y debe calibrarse. Por ejemplo, si una
señal medida a `400 mm` ocupa `60 px` de alto, el valor será:

```text
foco_equivalente_px = 60 × 400 / 100 = 240
```

La distancia es una estimación visual, no una lectura de encoder. Si la señal
sale del campo de visión durante un giro, el autónomo no la da por superada
inmediatamente: continúa recto usando una distancia de despeje estimada por
tiempo y velocidad, y solo después busca la siguiente señal.
