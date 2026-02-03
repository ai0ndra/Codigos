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
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Sistema de Brazo Robot Iniciado (Pines Corregidos) ---");

  // Pines ESP32 - Evitamos Pines 1 y 3 (TX/RX) para no interferir con el Serial
  // Inicializamos en 0 para que el movimiento a 90 sea visible
  SERVO1.attach(23, 0);
  SERVO2.attach(32, 0);
  SERVO3.attach(27, 0);
  SERVO4.attach(26, 0);
  SERVO5.attach(22, 0);

  Serial.println("Definiendo posición inicial...");

  // ----- POSICIÓN INICIAL -----
  Serial.println("Servo 1 a 90...");
  SERVO1.startEaseToD(90, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  Serial.println("Servo 2 a 90...");
  SERVO2.startEaseToD(90, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  Serial.println("Servo 3 a 90...");
  SERVO3.startEaseToD(90, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  Serial.println("Servo 4 a 90...");
  SERVO4.startEaseToD(90, tiempo3);
  updateAndWaitForAllServosToStop();
  delay(espera);

  Serial.println("Servo 5 a 90...");
  SERVO5.startEaseToD(90, tiempo3);
  updateAndWaitForAllServosToStop();

  Serial.println("Posición inicial alcanzada. Esperando...");
  delay(espera);
}

void loop() {
  Serial.println("--- Iniciando Secuencia 1 ---");

  // -------- SECUENCIA 1 --------
  Serial.println("Cintura...");
  SERVO1.startEaseToD(40, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  Serial.println("Brazo 1...");
  SERVO2.startEaseToD(30, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  Serial.println("Brazo 2...");
  SERVO3.startEaseToD(95, tiempo2);
  updateAndWaitForAllServosToStop();
  delay(espera);

  Serial.println("Base Pinza...");
  SERVO4.startEaseToD(54, tiempo3);
  updateAndWaitForAllServosToStop();
  delay(espera);

  Serial.println("Pinza...");
  SERVO5.startEaseToD(5, tiempo3);
  updateAndWaitForAllServosToStop();
  delay(espera);

  // -------- VOLVER A INICIAL --------
  Serial.println("--- Volviendo a Posición Inicial ---");
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
