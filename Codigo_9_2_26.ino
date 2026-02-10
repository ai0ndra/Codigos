#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// DURACIÓN del movimiento (en milisegundos)
// 3000 = 3 segundos por cada movimiento de servo.
// Usamos easeToD para asegurar que la librería use el tiempo y no la velocidad.
int duracionSosegada = 3000;
int espera           = 2000; // Pausa entre posiciones

void setup() {
  // IMPORTANTE: En la versión más reciente para ESP32,
  // esta línea es necesaria para gestionar los servos correctamente.
  ServoEasing::setServoEasingCount(5);

  // Pines ESP32 e inicialización.
  // El primer "salto" ocurrirá al hacer attach. Es inevitable por hardware.
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado (Easing)
  // EASE_CUBIC_IN_OUT proporciona el arranque y parada más fluidos.
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  // ----- CALIBRACIÓN INICIAL -----
  // easeToD(angulo, milisegundos) es bloqueante: se mueve uno por uno lentamente.
  SERVO1.easeToD(90, duracionSosegada);
  SERVO2.easeToD(90, duracionSosegada);
  SERVO3.easeToD(90, duracionSosegada);
  SERVO4.easeToD(90, duracionSosegada);
  SERVO5.easeToD(90, duracionSosegada);

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  SERVO1.easeToD(90, duracionSosegada);
  SERVO2.easeToD(90, duracionSosegada);
  SERVO3.easeToD(90, duracionSosegada);
  SERVO4.easeToD(90, duracionSosegada);
  SERVO5.easeToD(90, duracionSosegada);

  delay(espera);

  // -------- SECUENCIA 2: IR A TRABAJO UNO POR UNO --------
  SERVO1.easeToD(40, duracionSosegada);
  SERVO2.easeToD(40, duracionSosegada);
  SERVO3.easeToD(95, duracionSosegada);
  SERVO4.easeToD(54, duracionSosegada);
  SERVO5.easeToD(5, duracionSosegada);

  delay(espera);
}
