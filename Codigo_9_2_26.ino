#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// Tiempos de movimiento (en milisegundos)
// 10000 = 10 segundos. 15000 = 15 segundos.
// He aumentado estos valores para que el movimiento sea MUCHO más lento.
int tiempo2 = 10000;
int tiempo3 = 10000;
int espera  = 2000;

void setup() {

  // Pines ESP32
  // Al hacer attach con 90, el servo intentará mantenerse en esa posición desde el inicio.
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

  // ----- MOVIMIENTO INICIAL: UNO POR UNO -----
  // Aseguramos que todos empiecen en 90 grados de forma lenta y secuencial.
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

  // -------- SECUENCIA 1: MOVER UNO POR UNO A POSICIÓN DE TRABAJO --------
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

  // -------- SECUENCIA 2: VOLVER UNO POR UNO A INICIAL (90) --------
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
