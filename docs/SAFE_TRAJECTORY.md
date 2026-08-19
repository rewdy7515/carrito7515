# Trayectoria segura alrededor de señales

`software/raspberry_pi/src/trajectory_planner.py` planifica un esquive con
forma de S. Recibe la posición relativa de una señal y devuelve seis fases:

1. avanzar hasta el punto de inicio;
2. girar hacia el lado de rebase;
3. girar en sentido contrario para recuperar el rumbo;
4. avanzar hasta que el cuerpo completo del carrito pase la señal;
5. girar para volver a la trayectoria;
6. enderezar las ruedas.

El planificador usa estas medidas registradas del carrito:

- largo: 211.5 mm;
- distancia entre ejes: 148 mm;
- ancho de vía delantero: 146 mm, usado provisionalmente como ancho mínimo;
- señal: 50 × 50 × 100 mm.

Por defecto deja 80 mm de separación lateral adicional y 100 mm longitudinal.
Si la señal está demasiado cerca o el radio de giro no permite el
desplazamiento, responde que la maniobra no es segura. El controlador deberá
detenerse en ese caso.

## Calibraciones obligatorias antes de mover el robot con este plan

1. Medir el **ángulo real de las ruedas** para cada ángulo lógico del servo.
   El ángulo del servo no equivale automáticamente al ángulo de las ruedas.
2. Calibrar el foco equivalente de la cámara usando una señal de 100 mm a una
   distancia conocida.
3. Medir velocidad real o integrar el encoder. Sin encoder, las fases en mm
   solo pueden convertirse a tiempo mediante una estimación provisional.

La conversión visual es:

```text
distancia_mm = foco_equivalente_px × 100 / alto_de_la_señal_px
```
