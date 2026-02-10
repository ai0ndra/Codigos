#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// VELOCIDAD (Grados por segundo)
// 15 es una velocidad extremadamente suave y lenta.
// Ideal para que se note el arranque y parada suaves (Easing).
// ¡NO uses números como 2000 aquí, o irá a máxima velocidad!
int velocidadSuave = 15;
int espera         = 1500; // Pausa entre posiciones

void setup() {
  // IMPORTANTE: Para la versión más reciente de la librería en ESP32,
  // esta línea es necesaria para gestionar los 5 servos.
  ServoEasing::setServoEasingCount(5);

  // Pines ESP32 e inicialización en 90 grados.
  // El "salto" que ves al encender ocurre justo en estas líneas.
  // Para evitarlo, deja el brazo en posición de 90° antes de apagarlo.
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado (Easing)
  // CUBIC ofrece el arranque y parada más elegantes y fluidos.
  setEasingTypeForAllServos(EASE_CUBIC_IN_OUT);

  // ----- CALIBRACIÓN INICIAL -----
  // easeTo(angulo, velocidad) es bloqueante:
  // Espera a que cada servo termine antes de pasar al siguiente.
  SERVO1.easeTo(90, velocidadSuave);
  SERVO2.easeTo(90, velocidadSuave);
  SERVO3.easeTo(90, velocidadSuave);
  SERVO4.easeTo(90, velocidadSuave);
  SERVO5.easeTo(90, velocidadSuave);

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  SERVO1.easeTo(90, velocidadSuave);
  SERVO2.easeTo(90, velocidadSuave);
  SERVO3.easeTo(90, velocidadSuave);
  SERVO4.easeTo(90, velocidadSuave);
  SERVO5.easeTo(90, velocidadSuave);

  delay(espera);

  // -------- SECUENCIA 2: IR A TRABAJO UNO POR UNO --------
  SERVO1.easeTo(40, velocidadSuave);
  SERVO2.easeTo(40, velocidadSuave);
  SERVO3.easeTo(95, velocidadSuave);
  SERVO4.easeTo(54, velocidadSuave);
  SERVO5.easeTo(5, velocidadSuave);

  delay(espera);
}
