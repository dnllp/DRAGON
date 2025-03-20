#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import serial

# Configuración del puerto serial (ajusta el puerto y la velocidad según tu configuración)
ser = serial.Serial('/dev/ttyUSB0', 115200)

def serial_to_ros():
    # Inicializar el nodo de ROS
    rospy.init_node('esp32_receiver', anonymous=True)
    
    # Crear un publicador para el tópico "camara_detectada"
    pub = rospy.Publisher('camara_detectada', Int32, queue_size=10)
    
    rate = rospy.Rate(10)  # Frecuencia de publicación (10 Hz)
    
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            # Leer una línea del puerto serial
            line = ser.readline().decode('utf-8').strip()
            
            # Procesar la línea para extraer el ID de la cámara
            if line.startswith("Cámara ID:"):
                try:
                    parts = line.split(",")
                    camara_id = int(parts[0].split(":")[1].strip())
                    azul_detectado = parts[1].split(":")[1].strip()
                    
                    # Publicar el ID de la cámara si se detectó azul
                    if azul_detectado == "Sí":
                        rospy.loginfo(f"Cámara {camara_id} detectó azul")
                        pub.publish(camara_id)
                except Exception as e:
                    rospy.logerr(f"Error al procesar los datos: {e}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        serial_to_ros()
    except rospy.ROSInterruptException:
        pass
