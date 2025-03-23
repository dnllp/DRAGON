#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_PWMServoDriver.h>

// Definición de pines de sensores ultrasónicos
#define ECHO_PIN_1 22   // Trasero derecho
#define TRIG_PIN_1 23
#define ECHO_PIN_2 24   // Trasero izquierdo
#define TRIG_PIN_2 25
#define ECHO_PIN_3 26   // Frontal derecho
#define TRIG_PIN_3 27
#define ECHO_PIN_4 28   // Frontal izquierdo
#define TRIG_PIN_4 29

// Definición de canales del PCA9685 para servos
#define PAN_FRONT_SERVO 0
#define TILT_FRONT_SERVO 1
#define PAN_REVERSE_SERVO 2
#define TILT_REVERSE_SERVO 3
#define TRAY_LEFT_SERVO 4
#define TRAY_RIGHT_SERVO 5

// Definición de pines para el control de motores
const int inAPin = 10;  // INA pin
const int inBPin = 9;   // INB pin
const int pwMPin = 8;   // PWM pin

const int motorRightForward = 4;
const int motorRightBackward = 5;
const int motorLeftForward = 6;
const int motorLeftBackward = 7;

// Inicializar el controlador PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Inicializar el IMU (BNO055)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Variables compartidas entre tareas (si es necesario)
volatile float sensorData[4]; // Datos de los sensores ultrasónicos
volatile float imuData;       // Datos del IMU

// Prototipos de funciones
void taskSensores(void *parameter);
void taskServosMotores(void *parameter);
void readUltrasonicSensors();
long getDistance(int trigPin, int echoPin);
void resetServos();
void setServo(int channel, int angle);
void activaBarredora();
void desactivaBarredora();
void advance();
void back();
void stopMotors();

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);  // Espera a que el puerto serial se abra

  // Inicializar el controlador PCA9685
  pwm.begin();
  pwm.setPWMFreq(60);  // Frecuencia típica para servos

  // Configuración de pines de sensores ultrasónicos
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);
  pinMode(TRIG_PIN_4, OUTPUT);
  pinMode(ECHO_PIN_4, INPUT);

  // Configuración de pines de motores
  pinMode(inAPin, OUTPUT);
  pinMode(inBPin, OUTPUT);
  pinMode(pwMPin, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);

  // Inicializar el BNO055
  if (!bno.begin()) {
    Serial.println("Error al iniciar BNO055. Verifica la conexión.");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // Crear tareas (hilos)
  xTaskCreatePinnedToCore(
    taskSensores,      // Función de la tarea
    "TaskSensores",    // Nombre de la tarea
    10000,             // Tamaño de la pila
    NULL,              // Parámetros de la tarea
    1,                 // Prioridad de la tarea
    NULL,              // Handle de la tarea
    0                  // Núcleo donde se ejecutará (0 o 1)
  );

  xTaskCreatePinnedToCore(
    taskServosMotores, // Función de la tarea
    "TaskServosMotores", // Nombre de la tarea
    10000,             // Tamaño de la pila
    NULL,              // Parámetros de la tarea
    1,                 // Prioridad de la tarea
    NULL,              // Handle de la tarea
    1                  // Núcleo donde se ejecutará (0 o 1)
  );
}

void loop() {
  // El loop principal no hace nada porque las tareas se ejecutan en paralelo
  delay(1000);
}

// Tarea para leer los sensores
void taskSensores(void *parameter) {
  while (1) {
    // Leer sensores ultrasónicos
    readUltrasonicSensors();

    // Leer datos del IMU
    sensors_event_t event;
    bno.getEvent(&event);
    imuData = event.orientation.x;

    // Imprimir datos (opcional)
    Serial.print("Distancias: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(sensorData[i]);
      Serial.print(" ");
    }
    Serial.print(" | IMU: ");
    Serial.println(imuData);

    delay(100);  // Intervalo de lectura
  }
}

// Tarea para controlar servos y motores
void taskServosMotores(void *parameter) {
  while (1) {
    // Ejemplo de movimiento
    resetServos();
    advance();
    activaBarredora();
    delay(5000);
    back();
    desactivaBarredora();
    delay(2000);
    stopMotors();
    delay(1000);
  }
}

// Funciones para sensores ultrasónicos
void readUltrasonicSensors() {
  sensorData[0] = getDistance(TRIG_PIN_1, ECHO_PIN_1);
  sensorData[1] = getDistance(TRIG_PIN_2, ECHO_PIN_2);
  sensorData[2] = getDistance(TRIG_PIN_3, ECHO_PIN_3);
  sensorData[3] = getDistance(TRIG_PIN_4, ECHO_PIN_4);
}

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

// Funciones para servos
void resetServos() {
  setServo(PAN_FRONT_SERVO, 80);
  setServo(TILT_FRONT_SERVO, 115);
  setServo(PAN_REVERSE_SERVO, 85);
  setServo(TILT_REVERSE_SERVO, 70);
  setServo(TRAY_LEFT_SERVO, 0);
  setServo(TRAY_RIGHT_SERVO, 176);
}

void setServo(int channel, int angle) {
  int pulse = map(angle, 0, 180, 125, 625);  // Mapea el ángulo a pulsos
  pwm.setPWM(channel, 0, pulse);
}

// Funciones para motores
void activaBarredora() {
  digitalWrite(inAPin, LOW);
  digitalWrite(inBPin, HIGH);
  analogWrite(pwMPin, 255);
}

void desactivaBarredora() {
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