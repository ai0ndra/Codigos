#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// VELOCIDAD (Grados por segundo)
// 15 es una velocidad extremadamente lenta y suave.
int velocidad = 15;
int espera = 1500;

void setup() {
  // ----- PASO 1: LLEGAR A 90 GRADOS UNO POR UNO -----
  // Al hacer attach, el servo saltará a 90°. Es un comportamiento físico inevitable.
  // Pero al hacerlo de uno en uno con un delay, logramos que se posicionen secuencialmente.

  SERVO1.attach(23, 90); delay(800);
  SERVO2.attach(33, 90); delay(800);
  SERVO3.attach(32, 90); delay(800);
  SERVO4.attach(25, 90); delay(800);
  SERVO5.attach(22, 90); delay(800);

  // Configuración de suavizado
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  delay(espera);
}

// Función auxiliar para mover un servo y esperar a que termine (Garantiza secuencia)
void moverYEsperar(ServoEasing &servo, int angulo, int vel) {
  servo.easeTo(angulo, vel);
  // Este bucle fuerza al programa a esperar hasta que el servo llegue a su destino
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
