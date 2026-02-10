#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// VELOCIDAD de movimiento (en grados por segundo)
// Un valor de 20 o 30 es lento y permite ver el efecto de suavizado (Easing).
// Si quieres que sea más lento, baja este número (ej. 10).
// Si quieres que sea más rápido, súbelo (ej. 60).
int velocidad = 30;
int espera    = 1500; // Pausa entre secuencias

void setup() {

  // Pines ESP32
  // Al inicializar en 90, intentamos que el brazo no salte bruscamente al encender.
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado (Easing)
  // EASE_CUBIC_IN_OUT proporciona un arranque y parada muy suaves.
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  // ----- MOVIMIENTO INICIAL: ASEGURAR POSICIÓN HOME (90) -----
  // easeTo es una función que espera a que el servo termine antes de seguir.
  SERVO1.easeTo(90, velocidad);
  SERVO2.easeTo(90, velocidad);
  SERVO3.easeTo(90, velocidad);
  SERVO4.easeTo(90, velocidad);
  SERVO5.easeTo(90, velocidad);

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  // Esto asegura que el robot siempre empiece desde 90 en cada ciclo.
  SERVO1.easeTo(90, velocidad);
  SERVO2.easeTo(90, velocidad);
  SERVO3.easeTo(90, velocidad);
  SERVO4.easeTo(90, velocidad);
  SERVO5.easeTo(90, velocidad);

  delay(espera);

  // -------- SECUENCIA 2: MOVER UNO POR UNO A POSICIÓN DE TRABAJO --------
  SERVO1.easeTo(40, velocidad);
  SERVO2.easeTo(40, velocidad);
  SERVO3.easeTo(95, velocidad);
  SERVO4.easeTo(54, velocidad);
  SERVO5.easeTo(5, velocidad);

  delay(espera);
}
