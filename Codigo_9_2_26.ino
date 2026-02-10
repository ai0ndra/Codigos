#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// Tiempos de movimiento (en milisegundos)
// 3000 = 3 segundos. 10000 = 10 segundos.
int tiempo2 = 5000;
int tiempo3 = 5000;
int espera  = 1000;

void setup() {

  // Pines ESP32
  // Al hacer attach, el servo saltará a la posición inicial (90 por defecto).
  // Es recomendable que el brazo esté físicamente cerca de esta posición al encenderlo.
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

  // ----- MOVIMIENTO INICIAL SIMULTÁNEO -----
  // Movemos todos a la posición de inicio (90) de forma sincronizada y lenta.
  // Nota: Si ya saltaron a 90 en el attach(), este movimiento será instantáneo.
  SERVO1.setEaseToD(90, tiempo2);
  SERVO2.setEaseToD(90, tiempo2);
  SERVO3.setEaseToD(90, tiempo2);
  SERVO4.setEaseToD(90, tiempo3);
  SERVO5.setEaseToD(90, tiempo3);

  synchronizeAllServosStartAndWaitForAllServosToStop();

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: MOVER A POSICIÓN DE TRABAJO --------
  // Movemos todos los servos al mismo tiempo para que sea más fluido
  SERVO1.setEaseToD(40, tiempo2);
  SERVO2.setEaseToD(40, tiempo2);
  SERVO3.setEaseToD(95, tiempo2);
  SERVO4.setEaseToD(54, tiempo3);
  SERVO5.setEaseToD(5, tiempo3);

  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  // -------- SECUENCIA 2: VOLVER A INICIAL --------
  SERVO1.setEaseToD(90, tiempo2);
  SERVO2.setEaseToD(90, tiempo2);
  SERVO3.setEaseToD(90, tiempo2);
  SERVO4.setEaseToD(90, tiempo3);
  SERVO5.setEaseToD(90, tiempo3);

  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);
}
