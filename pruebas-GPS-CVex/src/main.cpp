#include "vex.h"
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

// DECLARACIONES GLOBALES

// Una instancia global de competition
vex::competition Competition;

// Declaración del controlador
vex::controller Controller = vex::controller(vex::controllerType::primary);

// Declaración del Brain
vex::brain Brain; 

// Declaración del GPS: Puerto 20, Offset X=750mm, Y=400mm (ejemplo).
vex::gps GPS = vex::gps(vex::PORT20, 750.0, 400.0, vex::distanceUnits::mm, 0.0, vex::turnType::right);

// Declaraciones de motores
vex::motor LeftMotor1 = vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor LeftMotor2 = vex::motor(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor LeftMotor3 = vex::motor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor LeftMotor4 = vex::motor(vex::PORT4, vex::gearSetting::ratio18_1, false);

vex::motor RightMotor1 = vex::motor(vex::PORT6, vex::gearSetting::ratio18_1, true);
vex::motor RightMotor2 = vex::motor(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::motor RightMotor3 = vex::motor(vex::PORT9, vex::gearSetting::ratio18_1, true);
vex::motor RightMotor4 = vex::motor(vex::PORT10, vex::gearSetting::ratio18_1, false);

// Grupos de motores para la base
vex::motor_group LeftDrive = vex::motor_group(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);
vex::motor_group RightDrive = vex::motor_group(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

// CONSTANTES DE NAVEGACIÓN
constexpr double IN_TO_MM = 25.4;
constexpr double TARGET_TOLERANCE_MM = 25.0; // Tolerancia de error de 25 mm (~1 pulgada)

// Constantes de control Proporcional (P). AJUSTAR PARA TU ROBOT.
constexpr double HEADING_KP = 0.6;
constexpr double DRIVE_KP = 1.2;

// --- ESTRUCTURA PARA PUNTOS DE REFERENCIA ---
struct Waypoint {
    double x;
    double y;
};

// Imprime los datos del GPS en la consola y en la pantalla del Brain
void print_gps_data() {
    printf("Procesado (Codigo): X=%.2f in, Y=%.2f in, H=%.2f deg\n", 
           GPS.xPosition(vex::distanceUnits::in), 
           GPS.yPosition(vex::distanceUnits::in), 
           GPS.heading());
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Codigo:");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("X: %.2f in", GPS.xPosition(vex::distanceUnits::in));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Y: %.2f in", GPS.yPosition(vex::distanceUnits::in));
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("H: %.2f deg", GPS.heading());
}

// Detiene ambos grupos de motores usando el frenado de tipo 'brake'
void stop_drivetrain() {
    LeftDrive.stop(vex::brakeType::brake);
    RightDrive.stop(vex::brakeType::brake);
}

// Función para navegar a una coordenada (X, Y) específica mezclando avance y corrección de rumbo
void driveToPoint(double targetX, double targetY, double driveSpeed = 50.0) {
    double targetX_mm = targetX * IN_TO_MM;
    double targetY_mm = targetY * IN_TO_MM;

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Movimiento a: (%.1f, %.1f) in", targetX, targetY);
    printf("--- Movimiento a: (%.1f, %.1f) in ---\n", targetX, targetY);

    double currentX, currentY;
    double distance, requiredHeading;
    double errorHeading;
    double driveVelocity, turnVelocity;

    do {
        // Obtener las coordenadas en X y Y con respecto al campo
        currentX = GPS.xPosition(vex::distanceUnits::mm);
        currentY = GPS.yPosition(vex::distanceUnits::mm);

        // Calcular la distancia al objetivo y el rumbo requerido
        double deltaX = targetX_mm - currentX;
        double deltaY = targetY_mm - currentY;
        distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

        // Calcular el rumbo requerido
        requiredHeading = std::atan2(deltaY, deltaX) * 180.0 / M_PI;
        requiredHeading = 90.0 - requiredHeading;
        if (requiredHeading < 0) requiredHeading += 360.0;
        if (requiredHeading > 360) requiredHeading -= 360.0;
        
        // Calcular el error de rumbo y aplicar el control de rumbo
        errorHeading = requiredHeading - GPS.heading();
        while (errorHeading > 180)
            errorHeading -= 360;
        while (errorHeading < -180)
            errorHeading += 360;

        // "Control Proporcional" para velocidad de avance y corrección de rumbo
        driveVelocity = std::min(driveSpeed, distance * DRIVE_KP);
        driveVelocity = std::max(10.0, driveVelocity);

        // Control Proporcional para rumbo
        turnVelocity = errorHeading * HEADING_KP;

        // Calcular la velocidad de avance y corrección de rumbo
        double leftVelocity = driveVelocity - turnVelocity;
        double rightVelocity = driveVelocity + turnVelocity;
        
        // Limitar la velocidad de avance a 100% para evitar que el robot se mueva demasiado rápido
        double max_abs_vel = std::max(std::abs(leftVelocity), std::abs(rightVelocity));
        if (max_abs_vel > 100.0) {
            leftVelocity = (leftVelocity / max_abs_vel) * 100.0;
            rightVelocity = (rightVelocity / max_abs_vel) * 100.0;
        }


        LeftDrive.setVelocity(leftVelocity, vex::velocityUnits::pct);
        RightDrive.setVelocity(rightVelocity, vex::velocityUnits::pct);

        LeftDrive.spin(vex::directionType::fwd);
        RightDrive.spin(vex::directionType::fwd);

        print_gps_data();
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Dist: %.2f in", distance / IN_TO_MM);
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("ReqH: %.2f deg", requiredHeading);
        printf("Dist: %.2f in, ReqH: %.2f deg\n", distance / IN_TO_MM, requiredHeading);

        vex::task::sleep(20);

    } while (distance > TARGET_TOLERANCE_MM);

    stop_drivetrain();
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("Punto alcanzado!");
    printf("--- Punto alcanzado! ---\n");
    vex::task::sleep(500);
}

// Inicializa el gps y espera su calibración
bool initialize_gps() {
    Brain.Screen.print("Inicializando GPS...");
    
    if (!GPS.installed()) {
        Brain.Screen.clearScreen();
        Brain.Screen.print("ERROR: Sensor GPS no instalado en PORT20.");
        return false;
    }

    printf("Iniciando calibracion del GPS...\n");
    GPS.calibrate();
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Calibrando... NO MOVER");

    int waitTime = 0;
    // Espera a que el gps este listo o timeout de 5 segundos
    while (GPS.isCalibrating() && waitTime < 5000) {
        vex::task::sleep(100);
        waitTime += 100;
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Tiempo: %d ms", waitTime);
    }
    
    // Si el gps no está listo, muestra un mensaje de error y retorna false
    if (GPS.isCalibrating()) {
        Brain.Screen.clearScreen();
        Brain.Screen.print("ERROR: Fallo en la calibracion del GPS (Timeout).");
        return false;
    }
    
    Brain.Screen.clearScreen();
    Brain.Screen.print("GPS Listo.");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Usando coordenadas del campo");
    vex::task::sleep(1000);
    return true;
}

// AUTONOMO
void autonomous(void) {
    // Inicializa el gps y mantiene la base detenida
    if (!initialize_gps()) {
        stop_drivetrain();
        return;
    }

    // Coordenadas iniciales del robot (en pulgadas) y rumbo (en grados)
    double startX = -35.57;
    double startY = -41.73;
    double startHeading = 162.01;

    // Lados del cuadrado de 24 pulgadas
    double side = 24.0;
    
    // SECUENCIA DE MOVIMIENTOS PARA FORMAR UN CUADRADO
    
    double h_rad; // Rumbo en radianes

    // Esquina 1 (mueve hacia adelante 24 pulgadas)
    h_rad = startHeading * M_PI / 180.0;
    double corner1X = startX + side * std::sin(h_rad);
    double corner1Y = startY + side * std::cos(h_rad);

    // Esquina 2 (gira 90 grados y mueve 24 pulgadas)
    h_rad = (startHeading + 90.0);
    if(h_rad > 360) h_rad -= 360;
    h_rad = h_rad * M_PI / 180.0;
    double corner2X = corner1X + side * std::sin(h_rad);
    double corner2Y = corner1Y + side * std::cos(h_rad);

    // Esquina 3 (gira 90 grados y mueve 24 pulgadas)
    h_rad = (startHeading + 180.0);
    if(h_rad > 360) h_rad -= 360;
    h_rad = h_rad * M_PI / 180.0;
    double corner3X = corner2X + side * std::sin(h_rad);
    double corner3Y = corner2Y + side * std::cos(h_rad);

    
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("-> Moviendo al Lado 1 del cuadrado...");
    driveToPoint(corner1X, corner1Y);

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("-> Moviendo al Lado 2 del cuadrado...");
    driveToPoint(corner2X, corner2Y);

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("-> Moviendo al Lado 3 del cuadrado...");
    driveToPoint(corner3X, corner3Y);

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("-> Regresando al inicio...");
    driveToPoint(startX, startY);
    
    stop_drivetrain();
    Brain.Screen.clearScreen();
    Brain.Screen.print("Ruta autonomo completada!");
}

// Función para el modo Driver en movimmiento de tank
void usercontrol(void) {
    while (true) {
        LeftDrive.setVelocity(Controller.Axis3.position(), vex::percentUnits::pct);
        RightDrive.setVelocity(Controller.Axis2.position(), vex::percentUnits::pct);
        LeftDrive.spin(vex::directionType::fwd);
        RightDrive.spin(vex::directionType::fwd);
        
        print_gps_data();
        
        vex::task::sleep(20);
    }
}

// Configuración del main
int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    vex::task::sleep(100); 
    return 0;
}