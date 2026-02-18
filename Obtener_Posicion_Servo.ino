/*
 * Este código permite controlar un servo manualmente desde el Monitor Serial.
 * Responde a la duda: ¿Por qué siempre empieza en 90 grados?
 */

#include <ServoEasing.hpp>

ServoEasing servo;
const int pinServo = 25;

// --- POSICIÓN INICIAL (HOME) ---
// Los servos estándar no saben su posición física al encenderse.
// Por eso, definimos un "Home" en el código para que el servo tenga un punto de partida conocido.
const int ANGULO_INICIAL = 90;

void setup() {
    Serial.begin(115200);

    // Al hacer attach(pin, grado), el servo se moverá inmediatamente a ese grado.
    // Esto evita movimientos erráticos, pero causará un "salto" si el servo
    // estaba en una posición muy diferente cuando se apagó.
    servo.attach(pinServo, ANGULO_INICIAL);

    Serial.println("--- Control Manual de Servo ---");
    Serial.print("Iniciado en posición Home: ");
    Serial.print(ANGULO_INICIAL);
    Serial.println(" grados.");
    Serial.println("Escribe un número (0-180) para moverlo:");
}

void loop() {
    if (Serial.available() > 0) {
        int objetivo = Serial.parseInt();

        // Limpiar buffer
        while(Serial.available() > 0) Serial.read();

        if (objetivo >= 0 && objetivo <= 180) {
            Serial.print("\nMoviendo de ");
            Serial.print(servo.getCurrentAngle());
            Serial.print("° a ");
            Serial.print(objetivo);
            Serial.println("°...");

            servo.startEaseTo(objetivo, 45);

            while (servo.isMoving()) {
                Serial.print("Grado actual: ");
                Serial.print(servo.getCurrentAngle());
                Serial.println("°");
                delay(100);
            }

            Serial.println("¡Llegamos!");
            Serial.println("\nEsperando nuevo ángulo...");
        } else {
            // Manejar caso de timeout o entrada inválida
            if (objetivo != 0 || (objetivo == 0 && Serial.read() != -1)) {
                 Serial.println("Error: Ingresa un ángulo entre 0 y 180.");
            }
        }
    }
}
