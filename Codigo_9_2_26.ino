#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// VELOCIDAD (Grados por segundo)
// Un valor pequeño como 20 hace que el brazo se mueva de forma lenta y elegante.
// A esta velocidad, verás claramente el arranque y parada suaves.
int velocidad = 20;
int espera = 1500; // Pausa entre posiciones

void setup() {
  // Pines ESP32
  // El "salto" inicial al encender es un comportamiento de hardware inevitable,
  // pero intentamos minimizarlo iniciando la señal en 90 grados.
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado (Easing)
  // CUBIC ofrece el movimiento más fluido y profesional.
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  // Calibración inicial lenta
  // easeTo es bloqueante: mueve un servo a la vez de forma secuencial.
  SERVO1.easeTo(90, velocidad);
  SERVO2.easeTo(90, velocidad);
  SERVO3.easeTo(90, velocidad);
  SERVO4.easeTo(90, velocidad);
  SERVO5.easeTo(90, velocidad);

  delay(espera);
}

void loop() {
  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  SERVO1.easeTo(90, velocidad);
  SERVO2.easeTo(90, velocidad);
  SERVO3.easeTo(90, velocidad);
  SERVO4.easeTo(90, velocidad);
  SERVO5.easeTo(90, velocidad);
  delay(espera);

  // -------- SECUENCIA 2: IR A TRABAJO UNO POR UNO --------
  SERVO1.easeTo(40, velocidad);
  SERVO2.easeTo(40, velocidad);
  SERVO3.easeTo(95, velocidad);
  SERVO4.easeTo(54, velocidad);
  SERVO5.easeTo(5, velocidad);
  delay(espera);
}
