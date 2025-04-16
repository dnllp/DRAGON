#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// Pines de control de motores
#define MOTOR_LEFT_PWM1 4
#define MOTOR_LEFT_PWM2 5
#define MOTOR_RIGHT_PWM1 6
#define MOTOR_RIGHT_PWM2 7

// Pines de encoder
#define ENCODER_LEFT_A 18
#define ENCODER_LEFT_B 19
#define ENCODER_RIGHT_A 20
#define ENCODER_RIGHT_B 21

// Parámetros del robot
const float WHEEL_RADIUS = 0.065;    // Radio de la rueda en metros
const float WHEEL_BASE = 0.30;       // Distancia entre ruedas en metros
const int ENCODER_TICKS = 980;       // Ticks por revolución del encoder

// Variables de encoder
volatile long left_ticks = 0;
volatile long right_ticks = 0;
long prev_left_ticks = 0;
long prev_right_ticks = 0;

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Variables para el Filtro de Kalman
typedef struct {
  float x;        // Posición x (m)
  float y;        // Posición y (m)
  float theta;    // Orientación (rad)
  float P[3][3];  // Matriz de covarianza
} KalmanState;

KalmanState kstate;

// Variables de control
int left_speed = 0;
int right_speed = 0;
unsigned long last_send_time = 0;
const unsigned long send_interval = 20; // 50Hz (20ms)
unsigned long last_update_time = 0;

void setup() {
  Serial.begin(57600);
  
  // Configurar pines de motor
  pinMode(MOTOR_LEFT_PWM1, OUTPUT);
  pinMode(MOTOR_LEFT_PWM2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM1, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM2, OUTPUT);
  
  // Inicializar encoders
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), handleLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), handleRightEncoder, CHANGE);
  
  // Inicializar IMU
  if(!bno.begin()) {
    Serial.println("Error al iniciar BNO055");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // Inicializar estado del Filtro de Kalman
  initKalmanFilter();
  
  Serial.println("Arduino Mega - Filtro de Kalman para Odometría");
}

void loop() {
  unsigned long current_time = millis();
  float dt = (current_time - last_update_time) / 1000.0;
  
  if(dt >= 0.02) {  // Actualizar a 50Hz
    // Paso 1: Obtener mediciones de los sensores
    float encoder_theta = updateOdometryFromEncoders(dt);
    float imu_theta = getIMUHeading();
    float imu_ang_vel = getIMUAngularVelocity();
    
    // Paso 2: Actualizar el Filtro de Kalman
    predictKalman(dt, imu_ang_vel);
    updateKalman(encoder_theta, imu_theta);
    
    last_update_time = current_time;
  }
  
  // Enviar datos periódicamente
  if(current_time - last_send_time >= send_interval) {
    sendSensorData();
    last_send_time = current_time;
  }
  
  // Procesar comandos de ROS
  if(Serial.available()) {
    processROSCommand();
  }
}

// Inicialización del Filtro de Kalman
void initKalmanFilter() {
  kstate.x = 0.0;
  kstate.y = 0.0;
  kstate.theta = 0.0;
  
  // Inicializar matriz de covarianza
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      kstate.P[i][j] = 0.0;
    }
    kstate.P[i][i] = 1.0;  // Varianzas iniciales
  }
}

// Predicción del estado (modelo de movimiento)
void predictKalman(float dt, float angular_velocity) {
  // Modelo de movimiento: x = x + v*cos(theta)*dt
  //                      y = y + v*sin(theta)*dt
  //                      theta = theta + w*dt
  
  // Calcular velocidad lineal promedio de los encoders
  float v_left = (2 * PI * WHEEL_RADIUS * (left_ticks - prev_left_ticks)) / (ENCODER_TICKS * dt);
  float v_right = (2 * PI * WHEEL_RADIUS * (right_ticks - prev_right_ticks)) / (ENCODER_TICKS * dt);
  float linear_velocity = (v_left + v_right) / 2.0;
  
  // Actualizar estado predicho
  kstate.x += linear_velocity * cos(kstate.theta) * dt;
  kstate.y += linear_velocity * sin(kstate.theta) * dt;
  kstate.theta += angular_velocity * dt;
  
  // Normalizar ángulo entre -PI y PI
  while(kstate.theta > PI) kstate.theta -= 2*PI;
  while(kstate.theta < -PI) kstate.theta += 2*PI;
  
  // Matriz de transición de estado (A)
  float A[3][3] = {
    {1, 0, -linear_velocity * sin(kstate.theta) * dt},
    {0, 1, linear_velocity * cos(kstate.theta) * dt},
    {0, 0, 1}
  };
  
  // Matriz de covarianza del proceso (Q)
  float Q[3][3] = {
    {0.01, 0, 0},
    {0, 0.01, 0},
    {0, 0, 0.01}
  };
  
  // Actualizar matriz de covarianza: P = A*P*A' + Q
  float AP[3][3], APAT[3][3];
  
  // Multiplicar A*P
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      AP[i][j] = 0;
      for(int k=0; k<3; k++) {
        AP[i][j] += A[i][k] * kstate.P[k][j];
      }
    }
  }
  
  // Multiplicar (A*P)*A'
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      APAT[i][j] = 0;
      for(int k=0; k<3; k++) {
        APAT[i][j] += AP[i][k] * A[j][k];  // A' es A transpuesta
      }
      kstate.P[i][j] = APAT[i][j] + Q[i][j];
    }
  }
}

// Actualización del estado con mediciones
void updateKalman(float encoder_theta, float imu_theta) {
  // Matriz de observación (H) - solo observamos theta
  float H[2][3] = {
    {0, 0, 1},  // Para encoder
    {0, 0, 1}   // Para IMU
  };
  
  // Covarianza de mediciones (R)
  float R[2][2] = {
    {0.05, 0},    // Varianza encoder
    {0, 0.01}     // Varianza IMU (más precisa)
  };
  
  // Vector de innovación y = z - H*x
  float y[2] = {
    encoder_theta - kstate.theta,
    imu_theta - kstate.theta
  };
  
  // Calcular S = H*P*H' + R
  float HP[2][3], S[2][2];
  
  // Multiplicar H*P
  for(int i=0; i<2; i++) {
    for(int j=0; j<3; j++) {
      HP[i][j] = 0;
      for(int k=0; k<3; k++) {
        HP[i][j] += H[i][k] * kstate.P[k][j];
      }
    }
  }
  
  // Multiplicar (H*P)*H' y sumar R
  for(int i=0; i<2; i++) {
    for(int j=0; j<2; j++) {
      S[i][j] = R[i][j];
      for(int k=0; k<3; k++) {
        S[i][j] += HP[i][k] * H[j][k];
      }
    }
  }
  
  // Calcular ganancia de Kalman K = P*H'*inv(S)
  float K[3][2];
  float detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
  float invS[2][2] = {
    {S[1][1]/detS, -S[0][1]/detS},
    {-S[1][0]/detS, S[0][0]/detS}
  };
  
  // Multiplicar P*H'
  float PHT[3][2];
  for(int i=0; i<3; i++) {
    for(int j=0; j<2; j++) {
      PHT[i][j] = 0;
      for(int k=0; k<3; k++) {
        PHT[i][j] += kstate.P[i][k] * H[j][k];
      }
    }
  }
  
  // Multiplicar (P*H')*inv(S) para obtener K
  for(int i=0; i<3; i++) {
    for(int j=0; j<2; j++) {
      K[i][j] = 0;
      for(int k=0; k<2; k++) {
        K[i][j] += PHT[i][k] * invS[k][j];
      }
    }
  }
  
  // Actualizar estado: x = x + K*y
  kstate.x += K[0][0]*y[0] + K[0][1]*y[1];
  kstate.y += K[1][0]*y[0] + K[1][1]*y[1];
  kstate.theta += K[2][0]*y[0] + K[2][1]*y[1];
  
  // Normalizar ángulo
  while(kstate.theta > PI) kstate.theta -= 2*PI;
  while(kstate.theta < -PI) kstate.theta += 2*PI;
  
  // Actualizar covarianza: P = (I - K*H)*P
  float KH[3][3], IKH[3][3];
  
  // Multiplicar K*H
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      KH[i][j] = 0;
      for(int k=0; k<2; k++) {
        KH[i][j] += K[i][k] * H[k][j];
      }
    }
  }
  
  // Calcular I - K*H
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      IKH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
    }
  }
  
  // Multiplicar (I-KH)*P
  float newP[3][3];
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      newP[i][j] = 0;
      for(int k=0; k<3; k++) {
        newP[i][j] += IKH[i][k] * kstate.P[k][j];
      }
    }
  }
  
  // Copiar nueva matriz de covarianza
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      kstate.P[i][j] = newP[i][j];
    }
  }
}

// Obtener orientación de los encoders
float updateOdometryFromEncoders(float dt) {
  // Calcular distancia recorrida por cada rueda
  float left_dist = (2 * PI * WHEEL_RADIUS * (left_ticks - prev_left_ticks)) / ENCODER_TICKS;
  float right_dist = (2 * PI * WHEEL_RADIUS * (right_ticks - prev_right_ticks)) / ENCODER_TICKS;
  
  // Guardar ticks actuales para la próxima iteración
  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;
  
  // Calcular cambio de orientación (modelo diferencial)
  float delta_theta = (right_dist - left_dist) / WHEEL_BASE;
  
  return delta_theta;
}

// Obtener orientación de la IMU
float getIMUHeading() {
  imu::Quaternion quat = bno.getQuat();
  
  // Convertir cuaternión a ángulo de Euler (solo nos interesa el yaw)
  float q0 = quat.w();
  float q1 = quat.x();
  float q2 = quat.y();
  float q3 = quat.z();
  
  float yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
  return yaw;
}

// Obtener velocidad angular de la IMU
float getIMUAngularVelocity() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  return gyro.z();  // Velocidad angular en el eje Z (rad/s)
}

// Enviar datos por serial
void sendSensorData() {
  Serial.print("KF ");
  Serial.print(kstate.x, 3);
  Serial.print(" ");
  Serial.print(kstate.y, 3);
  Serial.print(" ");
  Serial.print(kstate.theta, 3);
  Serial.println();
}

// Control de motores (sin cambios)
void setMotorSpeed(int pwm_pin1, int pwm_pin2, int speed) {
  speed = constrain(speed, -255, 255);
  
  if(speed > 0) {
    analogWrite(pwm_pin1, speed);
    analogWrite(pwm_pin2, 0);
  }
  else if(speed < 0) {
    analogWrite(pwm_pin1, 0);
    analogWrite(pwm_pin2, -speed);
  }
  else {
    analogWrite(pwm_pin1, 0);
    analogWrite(pwm_pin2, 0);
  }
}

// Interrupciones para encoders (sin cambios)
void handleLeftEncoder() {
  if(digitalRead(ENCODER_LEFT_A)) {
    left_ticks += (digitalRead(ENCODER_LEFT_B)) ? -1 : 1;
  } else {
    left_ticks += (digitalRead(ENCODER_LEFT_B)) ? 1 : -1;
  }
}

void handleRightEncoder() {
  if(digitalRead(ENCODER_RIGHT_A)) {
    right_ticks += (digitalRead(ENCODER_RIGHT_B)) ? 1 : -1;
  } else {
    right_ticks += (digitalRead(ENCODER_RIGHT_B)) ? -1 : 1;
  }
}

void processROSCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  if(command.startsWith("L") && command.indexOf("R") > 0) {
    int l_pos = command.indexOf('L');
    int r_pos = command.indexOf('R');
    int space_pos = command.indexOf(' ');
    
    if(l_pos >= 0 && r_pos > l_pos && space_pos > l_pos) {
      left_speed = command.substring(l_pos+1, space_pos).toInt();
      right_speed = command.substring(r_pos+1).toInt();
      
      setMotorSpeed(MOTOR_LEFT_PWM1, MOTOR_LEFT_PWM2, left_speed);
      setMotorSpeed(MOTOR_RIGHT_PWM1, MOTOR_RIGHT_PWM2, right_speed);
    }
  }
}