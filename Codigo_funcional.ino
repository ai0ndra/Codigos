#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

int tiempo2 = 1500;  // 1.5 segundos por movimiento
int tiempo3 = 1500;  // 1.5 segundos por movimiento
int espera = 1500;   // 1.5 segundos de espera

void setup() {
  // NOTA: Se ha eliminado toda comunicación Serial para permitir el uso de los pines 1 y 3.
  delay(1000);

  // Pines ESP32 - Incluyendo pines 1 y 3 (TX/RX0)
  // Inicializamos en 0 para que el movimiento a 90 sea visible
  SERVO1.attach(23, 0);
  SERVO2.attach(3, 0);
  SERVO3.attach(32, 0);
  SERVO4.attach(1, 0);
  SERVO5.attach(22, 0);

  // ----- POSICIÓN INICIAL -----
  SERVO1.startEaseToD(90, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO2.startEaseToD(90, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO3.startEaseToD(90, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO4.startEaseToD(90, tiempo3);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO5.startEaseToD(90, tiempo3);
  updateAndWaitForAllServosToStop();

  delay(espera);
}

void loop() {
  // -------- SECUENCIA 1 --------
  SERVO1.startEaseToD(40, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO2.startEaseToD(30, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO3.startEaseToD(95, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO4.startEaseToD(54, tiempo3);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO5.startEaseToD(5, tiempo3);
  updateAndWaitForAllServosToStop();
  delay(espera);

  // -------- VOLVER A INICIAL --------
  SERVO1.startEaseToD(90, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO2.startEaseToD(90, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO3.startEaseToD(90, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO4.startEaseToD(90, tiempo3);
  updateAndWaitForAllServosToStop();
  delay(espera);

  SERVO5.startEaseToD(90, tiempo3);
  updateAndWaitForAllServosToStop();
  delay(espera);
}
