// Definición de pines para los motores
const int motorRightForward = 4;
const int motorRightBackward = 5;
const int motorLeftForward = 6;
const int motorLeftBackward = 7;

void setup() {
  // Inicializa los pines de motor como salidas
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);

  // Inicia los motores en estado detenido
  stopMotors();
}

void loop() {
  // Aquí puedes llamar a las funciones de movimiento según sea necesario
  advance();
  delay(3000);
  back();
  delay(3000);
  turnLeftOneMotor();
  delay(3000);
  turnRightOneMotor();
  delay(3000);
  turnLeftBothMotors();
  delay(3000);
  turnRightBothMotors();
  delay(3000);
  stopMotors();
  delay(3000);
}

void advance() {
  analogWrite(motorRightForward, 255);
  analogWrite(motorLeftForward, 255);
  stopBackward();
}

void back() {
  analogWrite(motorRightBackward, 255);
  analogWrite(motorLeftBackward, 255);
  stopForward();
}

void turnLeftOneMotor() {
  analogWrite(motorRightForward, 255);
  stopMotorsLeft();
}

void turnRightOneMotor() {
  analogWrite(motorLeftForward, 255);
  stopMotorsRight();
}

void turnLeftBothMotors() {
  analogWrite(motorRightForward, 255);
  analogWrite(motorLeftBackward, 255);
  stopBackwardRight();
}

void turnRightBothMotors() {
  analogWrite(motorRightBackward, 255);
  analogWrite(motorLeftForward, 255);
  stopForwardRight();
}

void stopMotors() {
  stopForward();
  stopBackward();
}

void stopForward() {
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftForward, 0);
}

void stopBackward() {
  analogWrite(motorRightBackward, 0);
  analogWrite(motorLeftBackward, 0);
}

void stopMotorsRight() {
  analogWrite(motorRightForward, 0);
  analogWrite(motorRightBackward, 0);
}

void stopMotorsLeft() {
  analogWrite(motorLeftForward, 0);
  analogWrite(motorLeftBackward, 0);
}

void stopBackwardRight() {
  analogWrite(motorRightBackward, 0);
  analogWrite(motorLeftForward, 0);
}

void stopForwardRight() {
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftBackward, 0);
}