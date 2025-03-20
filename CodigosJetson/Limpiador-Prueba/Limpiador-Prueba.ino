// Definición de los pines del Arduino conectados al VNH2SP30
const int inAPin = 10;  // INA pin
const int inBPin = 9;  // INB pin
const int pwMPin = 8;  // PWM pin

// Definición de pines para los motores
const int motorRightForward = 4;
const int motorRightBackward = 5;
const int motorLeftForward = 6;
const int motorLeftBackward = 7;

void setup() {
  pinMode(inAPin, OUTPUT);
  pinMode(inBPin, OUTPUT);
  pinMode(pwMPin, OUTPUT);
 // Inicializa los pines de motor como salidas
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);

  // Inicia los motores en estado detenido
  stopMotors();

}

void loop() {
  turnLeftOneMotor();
  delay(3000);
  advance();
  activaBarredora();
  delay(4000);
  back();
  //desactivaBarredora();
  delay(2000);
  desactivaBarredora();
  //turnRightOneMotor();
  //delay(3000);
  turnLeftBothMotors();
  delay(3000);
  //turnRightBothMotors();
  //delay(3000);
  advance();
  activaBarredora();
  delay(4000);
  back();
  //desactivaBarredora();
  delay(2000);
  desactivaBarredora();
  stopMotors();
  delay(5000);

}

void activaBarredora(){
  digitalWrite(inAPin, LOW);
  digitalWrite(inBPin, HIGH);
  analogWrite(pwMPin, 255);
}
void desactivaBarredora(){
  digitalWrite(inAPin, LOW);
  digitalWrite(inBPin, LOW);
  analogWrite(pwMPin, 0); 
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
