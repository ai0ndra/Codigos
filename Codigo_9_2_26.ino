#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// VELOCIDAD para el loop (Grados por segundo)
int velocidad = 20;
int espera = 1500;

// Mueve usando duración (milisegundos) y espera a que termine
void moverYEsperarD(ServoEasing &servo, int angulo, int milis) {
  servo.easeToD(angulo, milis);
  while (servo.isMovingAndEasing()) {
    delay(1);
  }
}

// Mueve usando velocidad (grados por segundo) y espera a que termine
void moverYEsperar(ServoEasing &servo, int angulo, int vel) {
  servo.easeTo(angulo, vel);
  while (servo.isMovingAndEasing()) {
    delay(1);
  }
}

void setup() {
  // Pines ESP32
  // Usamos attach simple para que el primer movimiento (a 90) sea controlado
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  // ----- AL INICIO: Cada servo se mueve a 90 durante 1 segundo (1000ms) -----
  // Esto asegura que si el brazo no estaba en 90, llegue allí de forma controlada.
  moverYEsperarD(SERVO1, 90, 1000);
  moverYEsperarD(SERVO2, 90, 1000);
  moverYEsperarD(SERVO3, 90, 1000);
  moverYEsperarD(SERVO4, 90, 1000);
  moverYEsperarD(SERVO5, 90, 1000);

  delay(espera);
}

void loop() {
  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  moverYEsperar(SERVO1, 90, velocidad);
  moverYEsperar(SERVO2, 90, velocidad);
  moverYEsperar(SERVO3, 90, velocidad);
  moverYEsperar(SERVO4, 90, velocidad);
  moverYEsperar(SERVO5, 90, velocidad);
  delay(espera);

  // -------- SECUENCIA 2: IR A TRABAJO UNO POR UNO --------
  moverYEsperar(SERVO1, 40, velocidad);
  moverYEsperar(SERVO2, 40, velocidad);
  moverYEsperar(SERVO3, 95, velocidad);
  moverYEsperar(SERVO4, 54, velocidad);
  moverYEsperar(SERVO5, 5, velocidad);
  delay(espera);
}
