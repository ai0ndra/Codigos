#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// VELOCIDAD calculada (83 grados por segundo)
// Basada en un retraso de 12ms por grado (1000/12 = 83).
// Esto mantiene una velocidad constante sin importar la distancia,
// pero conserva el arranque y parada suaves (Easing).
int velocidadIdeal = 83;
int espera         = 1500; // Pausa entre posiciones

void setup() {
  // Pines ESP32 e inicialización en 90 grados.
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

  // ----- INICIO: ASEGURAR POSICIÓN 90 -----
  // easeTo usa la velocidad (grados por segundo) en lugar de tiempo fijo.
  SERVO1.easeTo(90, velocidadIdeal);
  SERVO2.easeTo(90, velocidadIdeal);
  SERVO3.easeTo(90, velocidadIdeal);
  SERVO4.easeTo(90, velocidadIdeal);
  SERVO5.easeTo(90, velocidadIdeal);

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  SERVO1.easeTo(90, velocidadIdeal);
  SERVO2.easeTo(90, velocidadIdeal);
  SERVO3.easeTo(90, velocidadIdeal);
  SERVO4.easeTo(90, velocidadIdeal);
  SERVO5.easeTo(90, velocidadIdeal);

  delay(espera);

  // -------- SECUENCIA 2: IR A TRABAJO UNO POR UNO --------
  SERVO1.easeTo(40, velocidadIdeal);
  SERVO2.easeTo(40, velocidadIdeal);
  SERVO3.easeTo(95, velocidadIdeal);
  SERVO4.easeTo(54, velocidadIdeal);
  SERVO5.easeTo(5, velocidadIdeal);

  delay(espera);
}
