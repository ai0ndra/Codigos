#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

int tiempo2 = 10000;
int tiempo3 = 10000;
int espera  = 1000;

void setup() {

  // Pines ESP32
  SERVO1.attach(23);
  SERVO2.attach(33);
  SERVO3.attach(32);
  SERVO4.attach(25);
  SERVO5.attach(22);

  // ----- OPCIÓN 2: ARRANQUE ULTRA SUAVE -----
  SERVO1.setEasingType(EASE_SINE_IN_OUT);
  SERVO2.setEasingType(EASE_SINE_IN_OUT);
  SERVO3.setEasingType(EASE_SINE_IN_OUT);

  // Los otros dos quedan normales
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

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

  delay(espera);
}

void loop() {

  SERVO1.startEaseToD(40, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO2.startEaseToD(40, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO3.startEaseToD(95, tiempo2);
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO4.startEaseToD(54, tiempo3);
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  SERVO5.startEaseToD(5, tiempo3);
  synchronizeAllServosStartAndWaitForAllServosToStop();
  delay(espera);

  // ----- VOLVER A INICIAL -----
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
