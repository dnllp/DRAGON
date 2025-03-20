#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Definir los canales del PCA9685 para cada servo
#define PAN_FRONT_SERVO 0
#define TILT_FRONT_SERVO 1
#define PAN_REVERSE_SERVO 2
#define TILT_REVERSE_SERVO 3
#define TRAY_LEFT_SERVO 4
#define TRAY_RIGHT_SERVO 5

// Definir las conexiones para los sensores ultrasónicos 
#define ECHO_PIN_1 23
#define TRIG_PIN_1 22
#define ECHO_PIN_2 25
#define TRIG_PIN_2 24
#define ECHO_PIN_3 27
#define TRIG_PIN_3 26
#define ECHO_PIN_4 29
#define TRIG_PIN_4 28
#define ECHO_PIN_5 31
#define TRIG_PIN_5 30

unsigned long previousMillis = 0;
const long interval = 100; // Intervalo de tiempo en milisegundos

// Inicializar el controlador PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  Serial.println("Controlador de servo PCA9685");

  pwm.begin();
  pwm.setPWMFreq(60);  // Frecuencia típica para servos

  // Coloca los servos en la posición inicial
  resetServos();
  // Configurar los pines para los sensores ultrasónicos
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);
  pinMode(TRIG_PIN_4, OUTPUT);
  pinMode(ECHO_PIN_4, INPUT);
  pinMode(TRIG_PIN_5, OUTPUT);
  pinMode(ECHO_PIN_5, INPUT);
}

void loop() {
  /*
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    readUltrasonicSensors();
  }
  */
  sweepCameraFront();
  //sweepCameraReverse();
  //unloadTray();
  delay(1);  // Espera un segundo antes de reiniciar el ciclo
}

// Función para colocar los servos en la posición inicial
void resetServos() {
  setServo(PAN_FRONT_SERVO, 180);
  setServo(TILT_FRONT_SERVO, 130);

  /*
  setServo(PAN_REVERSE_SERVO, 90);
  setServo(TILT_REVERSE_SERVO, 90);

  
  setServo(TRAY_LEFT_SERVO, 90);
  setServo(TRAY_RIGHT_SERVO, 90);
  */
}

// Función para barrer la cámara hacia adelante
void sweepCameraFront() {
  for (int angle = 20; angle <= 120; angle += 1) {
    setServo(PAN_FRONT_SERVO, angle);
    delay(200);
  }
  for (int angle = 120; angle >= 20; angle -= 1) {
    setServo(PAN_FRONT_SERVO, angle);
    delay(200);
  }
}


void sweepCameraReverse() {
  for (int angle = 0; angle <= 180; angle += 10) {
    setServo(PAN_REVERSE_SERVO, angle);
    delay(100);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    setServo(PAN_REVERSE_SERVO, angle);
    delay(100);
  }
}


// Función para descargar la bandeja
void unloadTray() {
  // Suponiendo que los servos están invertidos para la bandeja
  for (int angle = 90; angle <= 180; angle += 10) {
    setServo(TRAY_LEFT_SERVO, 180 - angle);  // Invertido
    setServo(TRAY_RIGHT_SERVO, angle);
    delay(100);
  }

  // Regresar la bandeja a su posición inicial
  setServo(TRAY_LEFT_SERVO, 90);
  setServo(TRAY_RIGHT_SERVO, 90);
}

// Función para establecer la posición del servo
void setServo(int channel, int angle) {
  int pulse = map(angle, 0, 180, 125, 625);  // Mapea el ángulo a pulsos (ajuste según tu servo)
  pwm.setPWM(channel, 0, pulse);
}

// Función para leer los sensores ultrasónicos
void readUltrasonicSensors() {
  Serial.println("Distancias de los sensores ultrasónicos:");

  Serial.print("Sensor 1: ");
  Serial.print(getDistance(TRIG_PIN_1, ECHO_PIN_1));
  Serial.println(" cm");

  Serial.print("Sensor 2: ");
  Serial.print(getDistance(TRIG_PIN_2, ECHO_PIN_2));
  Serial.println(" cm");

  Serial.print("Sensor 3: ");
  Serial.print(getDistance(TRIG_PIN_3, ECHO_PIN_3));
  Serial.println(" cm");

  Serial.print("Sensor 4: ");
  Serial.print(getDistance(TRIG_PIN_4, ECHO_PIN_4));
  Serial.println(" cm");

  Serial.print("Sensor 5: ");
  Serial.print(getDistance(TRIG_PIN_5, ECHO_PIN_5));
  Serial.println(" cm");
}

// Función para calcular la distancia medida por el sensor ultrasónico
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2; // Velocidad del sonido en cm/ms

  return distance;
}
