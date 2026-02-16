#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// VELOCIDAD (Grados por segundo)
// 20 es una velocidad lenta y fluida.
int velocidad = 20;
int espera = 1500;

void setup() {
  // Pines ESP32 e inicialización en 90 grados.
  // El salto inicial es inevitable por hardware si el brazo no está ya en 90.
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado (Easing)
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  // Calibración inicial lenta: un servo a la vez.
  moverYEsperar(SERVO1, 90, velocidad);
  moverYEsperar(SERVO2, 90, velocidad);
  moverYEsperar(SERVO3, 90, velocidad);
  moverYEsperar(SERVO4, 90, velocidad);
  moverYEsperar(SERVO5, 90, velocidad);

  delay(espera);
}

// Función que garantiza movimiento uno por uno y es compatible con todas las versiones.
void moverYEsperar(ServoEasing &servo, int angulo, int vel) {
  servo.easeTo(angulo, vel);
  while (servo.isMovingAndEasing()) {
    delay(1);
  }
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
