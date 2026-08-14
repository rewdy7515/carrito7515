import cv2
import numpy as np

def nada(x):
    pass

# Inicializar cámara con la resolución de tu código (320x240)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Crear ventana de controles
cv2.namedWindow('Controles', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Controles', 450, 400)

# Crear Trackbars
cv2.createTrackbar('MODO (0=Piso,1=Verde,2=Rojo,3=Magenta)', 'Controles', 0, 3, nada)

# Trackbars para HSV
cv2.createTrackbar('H Min', 'Controles', 35, 179, nada)
cv2.createTrackbar('H Max', 'Controles', 85, 179, nada)
cv2.createTrackbar('S Min', 'Controles', 60, 255, nada)
cv2.createTrackbar('S Max', 'Controles', 255, 255, nada)
cv2.createTrackbar('V Min', 'Controles', 50, 255, nada)
cv2.createTrackbar('V Max', 'Controles', 255, 255, nada)

# Trackbar para la Pista (Blanco/Negro)
cv2.createTrackbar('Umbral Piso', 'Controles', 95, 255, nada)

modo_anterior = 0

print("\n--- INICIANDO CALIBRADOR (CON RECORTES DEL CÓDIGO 1) ---")
print("Cambia el 'MODO' y los deslizadores saltaran a tus valores por defecto.")
print("Presiona 'p' para imprimir los valores en la consola.")
print("Presiona 'q' para salir.")
print("--------------------------------------------------------\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    modo_actual = cv2.getTrackbarPos('MODO (0=Piso,1=Verde,2=Rojo,3=Magenta)', 'Controles')
    
    # Auto-ajuste a valores por defecto al cambiar de modo
    if modo_actual != modo_anterior:
        if modo_actual == 0:
            cv2.setTrackbarPos('Umbral Piso', 'Controles', 95)
        elif modo_actual == 1: # VERDE
            cv2.setTrackbarPos('H Min', 'Controles', 35)
            cv2.setTrackbarPos('H Max', 'Controles', 85)
            cv2.setTrackbarPos('S Min', 'Controles', 60)
            cv2.setTrackbarPos('S Max', 'Controles', 255)
            cv2.setTrackbarPos('V Min', 'Controles', 50)
            cv2.setTrackbarPos('V Max', 'Controles', 255)
        elif modo_actual == 2: # ROJO
            cv2.setTrackbarPos('H Min', 'Controles', 0)
            cv2.setTrackbarPos('H Max', 'Controles', 5)
            cv2.setTrackbarPos('S Min', 'Controles', 100)
            cv2.setTrackbarPos('S Max', 'Controles', 255)
            cv2.setTrackbarPos('V Min', 'Controles', 80)
            cv2.setTrackbarPos('V Max', 'Controles', 255)
        elif modo_actual == 3: # MAGENTA
            cv2.setTrackbarPos('H Min', 'Controles', 130)
            cv2.setTrackbarPos('H Max', 'Controles', 175)
            cv2.setTrackbarPos('S Min', 'Controles', 80)
            cv2.setTrackbarPos('S Max', 'Controles', 255)
            cv2.setTrackbarPos('V Min', 'Controles', 80)
            cv2.setTrackbarPos('V Max', 'Controles', 255)
            
        modo_anterior = modo_actual

    # Leer valores actuales de los trackbars
    h_min = cv2.getTrackbarPos('H Min', 'Controles')
    h_max = cv2.getTrackbarPos('H Max', 'Controles')
    s_min = cv2.getTrackbarPos('S Min', 'Controles')
    s_max = cv2.getTrackbarPos('S Max', 'Controles')
    v_min = cv2.getTrackbarPos('V Min', 'Controles')
    v_max = cv2.getTrackbarPos('V Max', 'Controles')
    thresh_val = cv2.getTrackbarPos('Umbral Piso', 'Controles')

    # --- APLICAR LOS RECORTES EXACTOS DEL CÓDIGO 1 ---
    if modo_actual == 0 or modo_actual == 3:
        # Modo Piso y Magenta utilizan la franja estrecha (Y: 80 a 140)
        roi = frame[80:140, 0:320]
    else:
        # Modo Obstáculos Verde y Rojo utilizan la franja amplia (Y: 60 a 240)
        roi = frame[60:240, 0:320]

    if modo_actual == 0:
        # MODO 0: PISO
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (7, 7), 0)
        _, mascara = cv2.threshold(blur, thresh_val, 255, cv2.THRESH_BINARY)
        resultado = cv2.cvtColor(mascara, cv2.COLOR_GRAY2BGR)
        info = f"MODO: PISO | Umbral: {thresh_val}"
    
    else:
        # MODO 1, 2, 3: COLORES
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mascara = cv2.inRange(hsv, lower, upper)
        resultado = cv2.bitwise_and(roi, roi, mask=mascara)
        
        if modo_actual == 1:
            color_str = "VERDE"
        elif modo_actual == 2:
            color_str = "ROJO"
        else:
            color_str = "MAGENTA"
            
        info = f"MODO: {color_str} | L:[{h_min},{s_min},{v_min}] U:[{h_max},{s_max},{v_max}]"

    # Mostrar info en la ventana visual (sobre el ROI recortado)
    cv2.putText(roi, info, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    # Mostrar ventanas (Ahora solo muestran el trozo de imagen pertinente)
    cv2.imshow('Vision del Robot (ROI)', roi)
    cv2.imshow('Mascara (Blanco = Detectado)', mascara)
    cv2.imshow('Resultado', resultado)

    tecla = cv2.waitKey(1) & 0xFF
    
    # Imprimir en terminal
    if tecla == ord('p'):
        print(f"\n[CALIBRADO] {info}")
        
    # Salir
    elif tecla == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()