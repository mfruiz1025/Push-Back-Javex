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
vex::gps GPS = vex::gps(vex::PORT15, 750.0, 400.0, vex::distanceUnits::mm, 0.0, vex::turnType::right);

// Declaraciones de motores
vex::motor LeftMotor1 = vex::motor(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::motor LeftMotor2 = vex::motor(vex::PORT9, vex::gearSetting::ratio18_1, true);
vex::motor LeftMotor3 = vex::motor(vex::PORT10, vex::gearSetting::ratio18_1, false);

vex::motor RightMotor1 = vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor RightMotor2 = vex::motor(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor RightMotor3 = vex::motor(vex::PORT3, vex::gearSetting::ratio18_1, true);

vex::motor recoleccion1 = vex::motor(vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor recoleccion2 = vex::motor(vex::PORT12, vex::gearSetting::ratio18_1, false);


// Grupos de motores para la base
vex::motor_group LeftDrive = vex::motor_group(LeftMotor1, LeftMotor2, LeftMotor3);
vex::motor_group RightDrive = vex::motor_group(RightMotor1, RightMotor2, RightMotor3);
vex::motor_group Recoleccion = vex::motor_group(recoleccion1, recoleccion2);

// Constantes de control Proporcional (P). AJUSTAR PARA TU ROBOT.
constexpr double HEADING_KP = 0.25;
constexpr double DRIVE_KP = 0.8;

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

    // Mando
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("X: %.1f", GPS.xPosition(vex::distanceUnits::in));
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Y: %.1f", GPS.yPosition(vex::distanceUnits::in));
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("H: %.1f", GPS.heading());
}

// Detiene ambos grupos de motores usando el frenado de tipo 'brake'
void stop_drivetrain() {
    LeftDrive.stop(vex::brakeType::brake);
    RightDrive.stop(vex::brakeType::brake);
}

void driveToPoint(double targetX_mm, double targetY_mm, double driveSpeed = 30.0, double finalHeading = -1) {
    // Mostrar destino en pantalla
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Moviendo a: %.0f, %.0f mm", targetX_mm, targetY_mm);
    printf("-> Moviendo a: %.0f, %.0f mm\n", targetX_mm, targetY_mm);

    // Variables de posición y control
    double currentX, currentY;
    double distance, requiredHeading, errorHeading;
    double driveVelocity, turnVelocity;
    double lastDistance = 99999;

    int stuckCounter = 0;

    do {
        // Posición actual del GPS
        currentX = GPS.xPosition(vex::distanceUnits::mm);
        currentY = GPS.yPosition(vex::distanceUnits::mm);

        // Diferencias
        double deltaX = targetX_mm - currentX;
        double deltaY = targetY_mm - currentY;

        // Distancia al objetivo
        distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

        // Cálculo de rumbo deseado
        requiredHeading = std::atan2(deltaY, deltaX) * 180.0 / M_PI;
        requiredHeading = 90.0 - requiredHeading;
        if (requiredHeading < 0) requiredHeading += 360.0;
        if (requiredHeading > 360) requiredHeading -= 360.0;

        // Error de rumbo (diferencia entre rumbo actual y deseado)
        errorHeading = requiredHeading - GPS.heading();
        while (errorHeading > 180) errorHeading -= 360;
        while (errorHeading < -180) errorHeading += 360;

        // Control proporcional para avanzar
        driveVelocity = std::min(driveSpeed, distance * DRIVE_KP);
        driveVelocity = std::max(15.0, driveVelocity); // velocidad mínima segura

        // Control proporcional para girar (limitar el giro para evitar movimientos bruscos)
        turnVelocity = std::max(-20.0, std::min(errorHeading * HEADING_KP, 20.0));

        // Ajuste progresivo si está muy cerca
        if (distance < 150) {
            driveVelocity *= 0.6;  // desacelera suavemente al llegar
        }

        // Cálculo de velocidades para motores izquierdo y derecho
        double leftVelocity = driveVelocity - turnVelocity;
        double rightVelocity = driveVelocity + turnVelocity;

        // Limita velocidad total a 100%
        double maxVel = std::max(std::abs(leftVelocity), std::abs(rightVelocity));
        if (maxVel > 100.0) {
            leftVelocity = (leftVelocity / maxVel) * 100.0;
            rightVelocity = (rightVelocity / maxVel) * 100.0;
        }

        // Aplicar velocidades
        LeftDrive.setVelocity(leftVelocity, vex::percentUnits::pct);
        RightDrive.setVelocity(rightVelocity, vex::percentUnits::pct);
        LeftDrive.spin(vex::directionType::fwd);
        RightDrive.spin(vex::directionType::fwd);

        // Pantalla de depuración
        print_gps_data();
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Dist: %.0f mm", distance);
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("ReqH: %.1f deg", requiredHeading);

        // Verificación de progreso (prevención de estancamiento)
        if (std::abs(distance - lastDistance) < 2.0) {
            stuckCounter++;
        } else {
            stuckCounter = 0;
        }
        lastDistance = distance;

        vex::task::sleep(20);

        // Sale si se acerca lo suficiente o se queda estancado
    } while (distance > 20.0 && stuckCounter < 75);  // 20 mm de tolerancia

    stop_drivetrain();  // Frenado suave

    // Mensaje de éxito
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("Punto alcanzado!");
    printf("-> Punto alcanzado!\n");

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
    // Inicializa el GPS y detiene la base si falla
    if (!initialize_gps()) {
        stop_drivetrain();
        return;
    }

    // Lado del cuadrado (5 pies a mm)
    double side = 5.0 * 5.0 * 304.8; // 609.6 mm

    // Obtener posición y rumbo inicial desde el GPS
    double startX = GPS.xPosition(vex::distanceUnits::mm);
    double startY = GPS.yPosition(vex::distanceUnits::mm);
    double startHeading = GPS.heading();  // en grados

    // Mostrar en pantalla
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Inicio: X=%.1f Y=%.1f H=%.1f", startX, startY, startHeading);

    // Calcular posiciones de cada esquina del cuadrado
    double h_rad;

    // Esquina 1
    h_rad = startHeading * M_PI / 180.0;
    double corner1X = startX + side * std::sin(h_rad);
    double corner1Y = startY + side * std::cos(h_rad);

    // Esquina 2 (gira 90 grados)
    h_rad = (startHeading + 90.0) * M_PI / 180.0;
    double corner2X = corner1X + side * std::sin(h_rad);
    double corner2Y = corner1Y + side * std::cos(h_rad);

    // Esquina 3 (gira 180 grados)
    h_rad = (startHeading + 180.0) * M_PI / 180.0;
    double corner3X = corner2X + side * std::sin(h_rad);
    double corner3Y = corner2Y + side * std::cos(h_rad);

    // Moverse a las esquinas
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("-> Lado 1");
    driveToPoint(corner1X, corner1Y, 40.0);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("-> Lado 2");
    driveToPoint(corner2X, corner2Y, 40.0);

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("-> Lado 3");
    driveToPoint(corner3X, corner3Y, 40.0);

    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("-> Regresando...");
    driveToPoint(startX, startY, 40.0);

    stop_drivetrain();
    Brain.Screen.setCursor(8, 1);
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