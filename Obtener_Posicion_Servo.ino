/*
 * Este código demuestra cómo obtener la posición exacta (en grados) de un servo
 * utilizando tanto la librería estándar Servo.h como ServoEasing.hpp.
 */

#include <ServoEasing.hpp> // Recomendada para movimientos suaves
#include <ESP32Servo.h>    // Necesaria si usas ESP32 con la librería Servo estándar

// --- Ejemplo con ServoEasing ---
ServoEasing servoEasing;
const int pinEasing = 25;

// --- Ejemplo con Servo estándar ---
Servo servoEstandar;
const int pinEstandar = 26;

void setup() {
    Serial.begin(115200);

    // Configuración ServoEasing
    servoEasing.attach(pinEasing, 90); // Inicia en 90 grados

    // Configuración Servo estándar
    servoEstandar.attach(pinEstandar);
    servoEstandar.write(90); // Inicia en 90 grados

    Serial.println("Sistema iniciado. Moviendo servos...");
}

void loop() {
    // 1. Mover con ServoEasing y leer posición en tiempo real
    Serial.println("\nMoviendo ServoEasing a 180 grados...");
    servoEasing.startEaseTo(180, 50); // Mover a 180 con velocidad 50

    while (servoEasing.isMoving()) {
        // getCurrentAngle() devuelve la posición exacta durante el movimiento
        float anguloActual = servoEasing.getCurrentAngle();
        Serial.print("ServoEasing pos: ");
        Serial.print(anguloActual);
        Serial.println("°");
        delay(100);
    }
    Serial.println("ServoEasing llegó a su destino.");

    delay(2000);

    // 2. Mover con Servo estándar y leer posición
    Serial.println("\nMoviendo Servo estándar a 0 grados...");
    servoEstandar.write(0);

    // La librería estándar no tiene isMoving(), pero podemos leer el último valor escrito
    int ultimoGradoEscrito = servoEstandar.read();
    Serial.print("El último grado enviado al servo estándar fue: ");
    Serial.println(ultimoGradoEscrito);

    delay(2000);

    // 3. Regresar ServoEasing a 90 grados
    Serial.println("\nRegresando ServoEasing a 90 grados...");
    servoEasing.startEaseTo(90, 50);
    while (servoEasing.isMoving()) {
        Serial.print("ServoEasing pos: ");
        Serial.println(servoEasing.getCurrentAngle());
        delay(100);
    }

    delay(5000); // Esperar antes de repetir el ciclo
}
