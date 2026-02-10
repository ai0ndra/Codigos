#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// Tiempo total para cada movimiento (2000 ms = 2 segundos)
int tiempoMov = 2000;
int espera    = 1500; // Pausa entre posiciones

void setup() {
  // Pines ESP32 e inicialización en 90 grados.
  // attach(pin, inicial) ayuda a evitar saltos bruscos.
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
  // Usamos synchronizeAllServosStartAndWaitForAllServosToStop() después de cada movimiento
  // para asegurar que se muevan de uno en uno (secuencialmente).

  SERVO1.startEaseToD(90, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO2.startEaseToD(90, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO3.startEaseToD(90, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO4.startEaseToD(90, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO5.startEaseToD(90, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  SERVO1.startEaseToD(90, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO2.startEaseToD(90, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO3.startEaseToD(90, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO4.startEaseToD(90, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO5.startEaseToD(90, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  delay(espera);

  // -------- SECUENCIA 2: IR A TRABAJO UNO POR UNO --------
  SERVO1.startEaseToD(40, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO2.startEaseToD(40, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO3.startEaseToD(95, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO4.startEaseToD(54, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  SERVO5.startEaseToD(5, tiempoMov);
  synchronizeAllServosStartAndWaitForAllServosToStop();

  delay(espera);
}
