#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// Tiempos de movimiento (en milisegundos)
// 2000 = 2 segundos por cada movimiento de servo.
int tiempo2 = 2000;
int tiempo3 = 2000;
int espera  = 1500; // Pausa entre secuencias

void setup() {

  // Pines ESP32
  // Inicializamos en 90 grados para minimizar el salto inicial.
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado (Easing)
  SERVO1.setEasingType(EASE_SINE_IN_OUT);
  SERVO2.setEasingType(EASE_SINE_IN_OUT);
  SERVO3.setEasingType(EASE_SINE_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  // ----- MOVIMIENTO INICIAL: ASEGURAR POSICIÓN HOME (90) -----
  SERVO1.startEaseToD(90, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO2.startEaseToD(90, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO3.startEaseToD(90, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO4.startEaseToD(90, tiempo3);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO5.startEaseToD(90, tiempo3);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  // Aunque ya esté en 90 al empezar, esto asegura que el robot siempre empiece desde aquí cada ciclo.
  SERVO1.startEaseToD(90, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO2.startEaseToD(90, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO3.startEaseToD(90, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO4.startEaseToD(90, tiempo3);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO5.startEaseToD(90, tiempo3);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  delay(espera);

  // -------- SECUENCIA 2: MOVER UNO POR UNO A POSICIÓN DE TRABAJO --------
  SERVO1.startEaseToD(40, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO2.startEaseToD(40, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO3.startEaseToD(95, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO4.startEaseToD(54, tiempo3);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO5.startEaseToD(5, tiempo3);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  delay(espera);
}
