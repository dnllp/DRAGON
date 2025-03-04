// D.R.A.G.O.N. – Dispositivo Robótico Autónomo para la Gestión y Organización de Neodesechos

#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16.h>

// Pines de los encoders
#define ENCODER_LEFT_A 2  // Interrupción 0
#define ENCODER_LEFT_B 4  
#define ENCODER_RIGHT_A 3 // Interrupción 1
#define ENCODER_RIGHT_B 5  

// Parámetros del robot
#define WHEEL_RADIUS 0.05  // Radio de la rueda en metros
#define WHEEL_BASE 0.3     // Distancia entre ruedas en metros
#define TICKS_PER_REV 500  // Pulsos por revolución del encoder
#define GEAR_RATIO 1.0     // Relación de engranajes

volatile long left_ticks = 0, right_ticks = 0; // Contadores de pulsos
float x = 0.0, y = 0.0, theta = 0.0; // Odometría

ros::NodeHandle nh;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

// Funciones de interrupción para encoders
void encoderLeftA() { 
  if (digitalRead(ENCODER_LEFT_B) == HIGH) left_ticks++;
  else left_ticks--;
}

void encoderRightA() { 
  if (digitalRead(ENCODER_RIGHT_B) == HIGH) right_ticks++;
  else right_ticks--;
}

// Función para calcular odometría
void calculateOdometry() {
    static long prev_left_ticks = 0, prev_right_ticks = 0;
    
    long delta_left = left_ticks - prev_left_ticks;
    long delta_right = right_ticks - prev_right_ticks;
    
    float d_left = (delta_left / (float)TICKS_PER_REV) * (2 * PI * WHEEL_RADIUS) / GEAR_RATIO;
    float d_right = (delta_right / (float)TICKS_PER_REV) * (2 * PI * WHEEL_RADIUS) / GEAR_RATIO;

    float d_center = (d_left + d_right) / 2.0;
    float d_theta = (d_right - d_left) / WHEEL_BASE;

    x += d_center * cos(theta);
    y += d_center * sin(theta);
    theta += d_theta;

    // Publicar odometría en ROS
    odom_msg.header.stamp = nh.now();
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

    odom_pub.publish(&odom_msg);

    prev_left_ticks = left_ticks;
    prev_right_ticks = right_ticks;
}

void setup() {
    nh.initNode();
    nh.advertise(odom_pub);

    pinMode(ENCODER_LEFT_A, INPUT);
    pinMode(ENCODER_LEFT_B, INPUT);
    pinMode(ENCODER_RIGHT_A, INPUT);
    pinMode(ENCODER_RIGHT_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderLeftA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderRightA, CHANGE);
}

void loop() {
    calculateOdometry();
    nh.spinOnce();
    delay(100);
}
