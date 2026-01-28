#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

int tiempo2 = 3000;  // 3 segundos por movimiento (antes 2)
int tiempo3 = 4000;  // 4 segundos por movimiento (antes 3)
int espera = 5000;   // 5 segundos en cada posición

void setup() {
  // Pines ESP32
  SERVO1.attach(25);
  SERVO2.attach(23);
  SERVO3.attach(26);
  SERVO4.attach(18);
  SERVO5.attach(19);

  // ----- POSICIÓN INICIAL -----
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

  delay(espera);   // espera 5 segundos en posición inicial
}

void loop() {

  // -------- SECUENCIA 1 --------
  SERVO1.startEaseToD(40, tiempo2);        // cintura
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO2.startEaseToD(30, tiempo2);        // brazo 1
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO3.startEaseToD(95, tiempo2);        // brazo 2
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO4.startEaseToD(54, tiempo3);        // base pinza
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO5.startEaseToD(5, tiempo3);         // pinza
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  // -------- VOLVER A INICIAL --------
  SERVO1.startEaseToD(90, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO2.startEaseToD(90, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO3.startEaseToD(90, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO4.startEaseToD(90, tiempo3);
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO5.startEaseToD(90, tiempo3);
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);
}
