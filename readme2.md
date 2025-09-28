# Documentación del proyecto actual de investigación para el sistema de control del robot con GPS + PID

## Índice
1. [Descripción General](#descripción-general)
2. [Estructura del Proyecto](#estructura-del-proyecto)
3. [Archivos de Cabecera (include/)](#archivos-de-cabecera-include)
4. [Archivos de Implementación (src/)](#archivos-de-implementación-src)
5. [Interrelaciones entre Archivos](#interrelaciones-entre-archivos)
6. [Flujo de Ejecución](#flujo-de-ejecución)
7. [Configuración y Parámetros](#configuración-y-parámetros)

---

## Descripción General

Este proyecto implementa un sistema de control autónomo para un robot VEX V5 con capacidades de navegación GPS. El sistema está diseñado para participar en competencias de robótica VEX, incorporando control PID, navegación GPS, y múltiples estrategias autónomas.

### Características Principales:
- **Control PID avanzado** para movimientos precisos
- **Navegación GPS** para posicionamiento absoluto
- **Sistema de sensores integrado** (IMU, encoders, GPS)
- **Múltiples estrategias autónomas** seleccionables
- **Control manual con joystick** para operación teleoperada
- **Sistema de calibración y ajuste** para optimización de parámetros

---

## Estructura del Proyecto

```
AutonomoConGPS/
├── include/         # Archivos de utilidades (.h)
├── src/             # Implementaciones (.cpp)
├── data/            # Datos y scripts auxiliares
├── vex/             # Archivos del sistema VEX
├── makefile         # Script de compilación
└── README.md        # Documentación básica
```

---

## Archivos de utilidades (include/)

### 1. `adjusment.h` - Sistema de Calibración y Ajuste
**Propósito:** Define la clase y funciones para calibración automática de parámetros PID y pruebas de sistemas.

**Funcionalidad:**
- Automatiza el proceso de calibración de controladores PID
- Genera archivos de datos para análisis posterior
- Permite pruebas sistemáticas del comportamiento del robot

### 2. `autonomous.h` - Estrategias Autónomas
**Propósito:** Define todas las posibles estrategias autónomas disponibles para seleccionar en competencia según lo requerido.

### 3. `base.h` - Sistema de Movimiento Base
**Propósito:** Define la clase Base que controla todos los movimientos del chasis del robot.

**Clase Base - Métodos principales:**
- **Movimientos temporales:** `timerForward()`, `timerRotate()`
- **Movimientos por posición:** `posForwardRel()`, `posForwardAbs()`
- **Movimientos PID:** `PIDPosForwardRel()`, `PIDAngleRotateAbs()`
- **Movimientos parabólicos:** `PIDPosCurveRel()`, `PIDPosCurveAbs()`

### 4. `basic-functions.h` - Funciones básicas de movimiento y sensores
**Propósito:** Funciones fundamentales para control directo de motores y lectura de sensores.

**Funciones de movimiento (output):**
- `moveLeft()`, `moveRight()`: Control individual de lados
- `moveForward()`, `moveClockwise()`: Movimientos coordinados
- `lockBase()`, `unlockBase()`: Control de frenos

**Funciones de lectura (input):**
- `getLeftPos()`, `getRightPos()`: Posiciones de encoders
- `getForwardPos()`: Posición promedio hacia adelante
- `getHeading()`: Orientación del robot

### 5. `controller.h` - Control remoto
**Propósito:** Manejo de entradas del controlador manual y definición de variables de botones.

**Características:**
- Variables globales para todos los botones y joysticks
- Sistema de detección de cambios de estado (`last_X`, `last_Y`, etc.)
- Función `defineController()` para actualización continua de estados

### 6. `GPS.h` - Sistema de Navegación GPS
**Propósito:** Define la clase GPS para navegación absoluta y control de posición.

**Clase GPS - Métodos principales:**
- `initGPS()`: Inicialización del sistema GPS
- `updateGpsPos()`: Actualización continua de posición
- `gpsPIDMove()`: Movimiento PID basado en coordenadas GPS
- `gpsAim()`: Apuntado hacia coordenadas específicas
- **Métodos de acceso:** `getGpsX()`, `getGpsY()`, `getGpsHeading()`

### 7. `math-tools.h` - Herramientas Matemáticas
**Propósito:** Funciones matemáticas auxiliares para cálculos de navegación.

**Funciones:**
- `deg2rad()`, `rad2deg()`: Conversiones angulares
- `calAbsDeltaAng()`: Cálculo de diferencias angulares normalizadas

### 8. `my-timer.h` - Sistema de Temporización
**Propósito:** Clase para manejo de temporizadores personalizados.

**Clase MyTimer:**
- Constructor con tiempo inicial opcional
- `reset()`: Reinicia el temporizador
- `getTime()`: Obtiene tiempo transcurrido en milisegundos

### 9. `parameters.h` - Configuración de Parámetros
**Propósito:** Constantes y parámetros configurables del sistema.

**Parámetros principales:**
- Límites de movimiento y zona muerta de joystick
- Constantes físicas (diámetro de rueda, relación de engranajes)
- Parámetros PID específicos por robot
- Configuraciones condicionales para diferentes robots

### 10. `PID.h` - Controlador PID
**Propósito:** Implementación completa de controlador PID con características avanzadas.

**Clase PID - Características:**
- Coeficientes configurables (Kp, Ki, Kd)
- Límites de integral y rango de activación
- Tolerancias de error y derivada
- Detección automática de llegada al objetivo

### 11. `robot-config.h` - Configuración de Hardware
**Propósito:** Declaraciones de todos los componentes de hardware del robot.

**Componentes declarados:**
- **Chasis:** Motores izquierdo y derecho (4 motores cada lado)
- **Sensores:** IMU y sensor GPS
- **Mecanismos:** Recolector, rampa, pinza, neumáticos
- Función `vexcodeInit()` para inicialización

### 12. `sensors.h` - Sistema de Sensores
**Propósito:** Define la clase Sensors para manejo unificado de todos los sensores.

**Clase Sensors - Métodos:**
- `updateSensors()`: Actualización continua de lecturas
- Getters para posiciones y velocidades de base
- Control de orientación del robot
- Funciones de reset para encoders e IMU

### 13. `vex.h` - Inclusiones del Sistema VEX
**Propósito:** Archivo de inclusión principal para el sistema VEX V5.

---

## Archivos de Implementación (src/)

### 1. `adjustment.cpp` - Implementación de Calibración
**Funcionalidad principal:**
- Funciones de calibración automática que varían parámetros PID sistemáticamente
- Genera archivos de texto con datos de entrada/salida para análisis

**Ejemplo de uso:**
```cpp
// Calibración automática del parámetro P para movimiento hacia adelante
tuning_forward_p(); // Varía Kp de 0.2 a 0.4 en incrementos de 0.02
```

### 1. `GPS.cpp` - Implementación del Sistema GPS
**Funcionalidades principales:**

**Cálculos matemáticos:**
- `calTheta()`: Calcula ángulo hacia objetivo
- `calTargetPara()`: Calcula parámetros de navegación

**Actualización de posición:**
- `updateGpsPos()`: Integración numérica de posición usando odometría
- Combinación de datos de encoders e IMU

**Navegación:**
- `gpsPIDMove()`: Movimiento PID hacia coordenadas específicas
- `gpsAim()`: Rotación hacia punto de interés
- Diferentes modos: "continue", "heading", "PID"

### 2. `main.cpp` - Programa Principal
**Estructura principal:**
- `pre_auton()`: Inicialización antes de competencia
- `autonomous()`: Selector de estrategias autónomas
- `usercontrol()`: Control manual con joystick

**Sistema de selección de estrategias:**
- Variable `auton_strategy` para seleccionar rutina
- Interfaz de selección con botones del controlador

**Control manual:**
- Control de chasis con zona muerta configurable
- Sistema de bloqueo de base con botón B

### 3. `PID.cpp` - Implementación del Controlador PID
**Algoritmo PID completo:**
- Cálculo de error, derivada e integral
- Condiciones de finalización configurables
- Manejo de ciclos de ejecución

### 4. `robot-config.cpp` - Configuración de Hardware
**Configuración específica:**
- Definición de puertos para todos los motores
- Configuración de grupos de motores (Left, Right, Recolector)
- Inicialización de sensores (IMU, GPS)

---

## Interrelaciones entre Archivos

### Jerarquía de Dependencias

```
main.cpp (Programa principal)
├── autonomous.h/cpp (Estrategias autónomas)
│   ├── GPS.h/cpp (Navegación GPS)
│   ├── base.h/cpp (Control de movimiento)
│   └── basic-functions.h/cpp (Funciones básicas)
├── controller.h (Interfaz de control)
├── sensors.h/cpp (Sistema de sensores)
└── robot-config.h/cpp (Configuración hardware)

Módulos de soporte:
├── PID.h/cpp (Controlador PID)
├── my-timer.h/cpp (Temporización)
├── math-tools.h/cpp (Matemáticas)
├── parameters.h (Configuración)
└── adjusment.h/cpp (Calibración)
```

### Flujo de Datos Principal

1. **Inicialización (main.cpp → robot-config.cpp)**
   - Configuración de hardware
   - Calibración de sensores
   - Inicialización de clases globales

2. **Operación Autónoma (autonomous.cpp)**
   - GPS.cpp actualiza posición continuamente
   - base.cpp ejecuta movimientos PID
   - sensors.cpp proporciona retroalimentación

3. **Control Manual (main.cpp)**
   - controller.h lee entradas
   - basic-functions.cpp ejecuta comandos directos
   - sensors.cpp monitorea estado

---

## Flujo de Ejecución

### 1. Inicio del Sistema
```
vexcodeInit() → Calibración IMU → Inicialización GPS → Configuración sensores
```

### 2. Modo Autónomo
```
auton_init() → Selección de estrategia → Ejecución de rutina → auton_pre_usercontrol()
```

### 3. Modo Manual
```
Bucle infinito: defineController() → Procesamiento entradas → Control motores → Espera 20ms
```

### 4. Hilos de Fondo
```
autonSensors(): Actualización continua de sensores cada 5ms
autonGPS(): Actualización de posición GPS cada ciclo
```

---

## Configuración y Parámetros

### Parámetros Físicos del Robot
- **Diámetro de rueda:**
- **Relación de engranajes:**
- **Configuración:** Chasis de X motores por cada lado

### Parámetros de Control
- **Zona muerta joystick:** 5%
- **Límite mínimo de movimiento:** 5%
- **Límites PID configurables** por tipo de robot

### Configuración GPS
- **Puerto:**
- **Offset inicial:**
- **Unidades:** Milímetros para distancia

### Configuración de Competencia
- **Estrategias autónomas:** 4 seleccionables