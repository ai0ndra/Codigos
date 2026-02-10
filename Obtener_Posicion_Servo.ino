/*
 * Este código permite controlar un servo manualmente desde el Monitor Serial
 * y muestra en tiempo real en qué grado se encuentra.
 */

#include <ServoEasing.hpp>

ServoEasing servo;
const int pinServo = 25; // Cambia este pin según tu conexión (p.ej. 25 en ESP32)

void setup() {
    Serial.begin(115200);

    // Configuración inicial: empezamos en 90 grados
    servo.attach(pinServo, 90);

    Serial.println("--- Control Manual de Servo ---");
    Serial.println("Escribe un número entre 0 y 180 en el Monitor Serial y pulsa Enter.");
    Serial.println("El sistema te dirá en qué grado se encuentra el servo en cada momento.");
    Serial.print("Posición actual: ");
    Serial.println(servo.getCurrentAngle());
}

void loop() {
    // Verificar si hay datos en el puerto serial
    if (Serial.available() > 0) {
        // Leer el número ingresado
        int objetivo = Serial.parseInt();

        // Limpiar el buffer del Serial
        while(Serial.available() > 0) Serial.read();

        // Validar el rango (0 a 180 grados)
        if (objetivo >= 0 && objetivo <= 180) {
            Serial.print("\nMoviendo a: ");
            Serial.print(objetivo);
            Serial.println(" grados...");

            // Mover el servo suavemente
            // Usamos una velocidad de 45 grados por segundo (ideal según requerimientos previos)
            servo.startEaseTo(objetivo, 45);

            // Mientras se mueve, reportamos la posición exacta
            while (servo.isMoving()) {
                float anguloActual = servo.getCurrentAngle();
                Serial.print("Grado actual: ");
                Serial.print(anguloActual);
                Serial.println("°");

                delay(100); // Pequeña pausa para no saturar el Monitor Serial
            }

            Serial.println("¡Movimiento completado!");
            Serial.print("Posición final confirmada: ");
            Serial.print(servo.getCurrentAngle());
            Serial.println("°");
            Serial.println("\nEsperando nuevo comando...");
        } else {
            if (objetivo != 0 || (objetivo == 0 && Serial.read() != -1)) { // Evitar el 0 que devuelve parseInt si no hay número
                 Serial.println("Error: Por favor ingresa un ángulo válido entre 0 y 180.");
            }
        }
    }
}
