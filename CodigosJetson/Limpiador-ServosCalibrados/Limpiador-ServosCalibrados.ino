#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Definir los canales del PCA9685 para cada servo
#define PAN_FRONT_SERVO 0
#define TILT_FRONT_SERVO 1
#define PAN_REVERSE_SERVO 2
#define TILT_REVERSE_SERVO 3
#define TRAY_LEFT_SERVO 4
#define TRAY_RIGHT_SERVO 5

// Inicializar el controlador PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  Serial.println("Controlador de servo PCA9685");

  pwm.begin();
  pwm.setPWMFreq(60);  // Frecuencia típica para servos

  // Coloca los servos en la posición inicial
  resetServos();
}

void loop() {
  //sweepCameraFront();
  //sweepCameraReverse();
  //unloadTray();
  resetServos();
  delay(1000);  // Espera un segundo antes de reiniciar el ciclo
}

void resetServos() {
  setServo(PAN_FRONT_SERVO, 80);//der 10 -  80   -  150  izq mirando hacia el frente
  setServo(TILT_FRONT_SERVO, 150);//arriba 160  -  115 -  40  abajo
  setServo(PAN_REVERSE_SERVO, 85); //izq 160 - 85 -  10 der mirando hacia atras
  setServo(TILT_REVERSE_SERVO, 70); // arriba 10 - 70 -  130 abajo
  setServo(TRAY_LEFT_SERVO, 90);  // servo izquierdo 0 - 110 
  setServo(TRAY_RIGHT_SERVO, 90); // servo derecho 0 - 110 
}

void sweepCameraFront() {
  for (int angle = 0; angle <= 180; angle += 10) {
    setServo(PAN_FRONT_SERVO, angle);
    delay(300);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    setServo(PAN_FRONT_SERVO, angle);
    delay(300);
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

void unloadTray() {
  /*// Suponiendo que los servos están invertidos para la bandeja
  for (int angle = 90; angle <= 110; angle += 1) {
    setServo(TRAY_LEFT_SERVO, 110 - angle);  // Invertido
    setServo(TRAY_RIGHT_SERVO, angle);
    delay(500);
  }
*/
  // Regresar la bandeja a su posición inicial
  setServo(TRAY_LEFT_SERVO, 90);
  setServo(TRAY_RIGHT_SERVO, 90);
}

void setServo(int channel, int angle) {
  int pulse = map(angle, 0, 180, 125, 625);  // Mapea el ángulo a pulsos (ajuste según tu servo)
  pwm.setPWM(channel, 0, pulse);
}
