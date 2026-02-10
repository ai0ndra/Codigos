#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// VELOCIDAD (Grados por segundo)
// 15 es una velocidad extremadamente suave y lenta.
// Ideal para ver el efecto de arranque y parada suaves (Easing).
int velocidadSuave = 15;
int espera         = 2000; // Pausa entre posiciones (2 segundos)

void setup() {
  // Pines ESP32 e inicialización en 90 grados.
  // IMPORTANTE: Para evitar el salto inicial, asegúrate de dejar el brazo
  // en esta posición (90°) antes de apagarlo la última vez.
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado (Easing)
  // EASE_CUBIC_IN_OUT proporciona la aceleración y deceleración más elegantes.
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  // ----- CALIBRACIÓN INICIAL -----
  // Como ya hicimos attach en 90, esto solo asegura que el sistema esté sincronizado.
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

  // -------- SECUENCIA 2: MOVER UNO POR UNO A POSICIÓN DE TRABAJO --------
  SERVO1.easeTo(40, velocidadSuave);
  SERVO2.easeTo(40, velocidadSuave);
  SERVO3.easeTo(95, velocidadSuave);
  SERVO4.easeTo(54, velocidadSuave);
  SERVO5.easeTo(5, velocidadSuave);

  delay(espera);
}
