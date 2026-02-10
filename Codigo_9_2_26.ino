#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// Tiempo para cada movimiento (2000 ms = 2 segundos)
int tiempoEse = 2000;
int espera    = 1500; // Pausa entre posiciones

void setup() {
  // Pines ESP32 e inicialización en 90 grados.
  // Al usar attach(pin, inicial), el servo se posiciona antes de activarse.
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado (Easing)
  // EASE_CUBIC_IN_OUT proporciona el arranque y parada más suaves.
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  // ----- INICIO: ASEGURAR POSICIÓN 90 -----
  // easeToD es una función de bloqueo (bloquea hasta que el servo termina).
  // Esto asegura que se muevan uno por uno sin necesidad de funciones externas.
  SERVO1.easeToD(90, tiempoEse);
  SERVO2.easeToD(90, tiempoEse);
  SERVO3.easeToD(90, tiempoEse);
  SERVO4.easeToD(90, tiempoEse);
  SERVO5.easeToD(90, tiempoEse);

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  SERVO1.easeToD(90, tiempoEse);
  SERVO2.easeToD(90, tiempoEse);
  SERVO3.easeToD(90, tiempoEse);
  SERVO4.easeToD(90, tiempoEse);
  SERVO5.easeToD(90, tiempoEse);

  delay(espera);

  // -------- SECUENCIA 2: IR A TRABAJO UNO POR UNO --------
  SERVO1.easeToD(40, tiempoEse);
  SERVO2.easeToD(40, tiempoEse);
  SERVO3.easeToD(95, tiempoEse);
  SERVO4.easeToD(54, tiempoEse);
  SERVO5.easeToD(5, tiempoEse);

  delay(espera);
}
