# Entorno

## Información confirmable

| Área | Dato | Fuente/estado |
|---|---|---|
| Computador | Raspberry Pi 4 Model B de 4 GB | README; documentado |
| Software de alto nivel | Python | README y `robot.service`; entorno previsto |
| Firmware | Arduino/C++ | `src/Ino Code/Arduino_Code.ino`; confirmado |
| Cámara | USB, Logitech C922 | README y fotografías; documentado/observado |
| Comunicación | Serial a 115200 baudios | `.ino`; confirmado |
| Usuario antiguo | `samuelg` | `src/robot.service`; confirmado como configuración histórica |
| Servicio antiguo | `WorkingDirectory=/home/samuelg/Desktop` y `ExecStart=/usr/bin/python3 /home/samuelg/Desktop/Main0.1` | `src/robot.service`; probablemente desactualizado |

El servicio antiguo no debe copiarse como configuración recomendada. Solo registra un entorno anterior y una ruta que no debe asumirse válida para la nueva instalación.

## Pendiente de comprobar directamente en la Raspberry Pi

- Puerto serial real y dispositivo (`/dev/ttyACM*`, `/dev/ttyUSB*` u otro).
- Dispositivo de cámara y permisos de acceso.
- Sistema operativo e imagen instalada.
- Versión exacta de Python.
- Dependencias instaladas.
- Ruta final del repositorio.
- Usuario de ejecución y configuración actual de `systemd`.

