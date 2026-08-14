# Instrucciones para agentes

Este repositorio corresponde al robot de **WRO 2026 Future Engineers** de Ingenieros Paralelos.

## Alcance actual

- La nueva lógica de navegación se desarrollará desde cero.
- No reutilizar ni tomar como diseño de referencia la lógica de los archivos `.py` existentes, salvo solicitud expresa del usuario.
- Esta documentación describe el contexto físico, eléctrico y de software base; no define todavía el algoritmo de navegación.

## Reglas de trabajo

- Antes de modificar hardware o código de Arduino, leer `docs/HARDWARE.md`, `docs/CONNECTIONS.md`, `docs/ARCHITECTURE.md`, `docs/ENVIRONMENT.md` y `docs/OPEN_QUESTIONS.md`.
- No cambiar pines, protocolo serial, límites del servo ni alimentación sin actualizar primero la documentación correspondiente.
- No inventar conexiones, valores eléctricos, capacidades, medidas ni comportamiento del hardware.
- Identificar siempre la fuente de cada dato y diferenciar entre:
  - confirmado por el código;
  - documentado en el README o datasheets;
  - observado en fotografías;
  - pendiente de confirmar físicamente.
- Priorizar pruebas individuales, con el robot elevado o asegurado, y condiciones de seguridad antes de pruebas de movimiento.
- Ante comandos inválidos, pérdida de comunicación o errores críticos, el sistema debe detener el motor.
- No incluir credenciales, contraseñas, IP privadas permanentes ni otros secretos.

## Restricciones del contexto heredado

Los archivos Python existentes pertenecen a una implementación anterior que será reemplazada. No documentar ni reutilizar sus algoritmos, parámetros o decisiones de navegación como diseño del sistema nuevo.

