'''
import cv2
import numpy as np
import jetson.inference
import jetson.utils
import threading
import serial

# Inicializar la comunicación serial con el Arduino
ser = serial.Serial('/dev/ttyACM0', 9600)  # Ajusta el puerto serial y la velocidad según corresponda

class CSI_Camera:

    def __init__(self):
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            # Grab the first frame to start the video capturing
            self.grabbed, self.frame = self.video_capture.read()

        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)


    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop(self):
        self.running = False
        # Kill the thread
        self.read_thread.join()
        self.read_thread = None

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()

def gstreamer_pipeline(
    sensor_id=1,
    capture_width=320,
    capture_height=240,
    display_width=320,
    display_height=240,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

''' 
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
'''

def mandarArduino(angulo_x, angulo_y):
    mensaje = f"{angulo_x},{angulo_y}\n"# Convertir los ángulos a cadenas y agregar el carácter de terminación
    ser.write(mensaje.encode())# Enviar el mensaje al Arduino a través del puerto serial

# AQUI HUBO CAMBIOS (TODA LA FUNCION)
def pixeles_a_angulos(x_pixel, y_pixel, resolucion_x, resolucion_y):
    # Rango de movimiento en grados en el eje X
    angulo_inicial = 20  # Angulo inicial
    angulo_final = 120   # Angulo final
    angulo_central = 75  # Angulo central

    # Convertir las coordenadas de píxeles a porcentajes
    porcentaje_x = x_pixel / resolucion_x

    # Convertir porcentajes a ángulos
    angulo_x = angulo_inicial + porcentaje_x * (angulo_final - angulo_inicial)

    # Mantener el ángulo dentro del rango permitido
    angulo_x = min(angulo_final, max(angulo_inicial, angulo_x))

    return angulo_x, 0  # Solo se ajusta el ángulo horizontal, el ángulo vertical se deja en 0


''' 
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
'''

def detect_and_display(camera, net):
    while True:
        grabbed, frame = camera.read()

        if not grabbed:
            print("Error: Cannot read frame from camera")
            break

        cuda_frame = jetson.utils.cudaFromNumpy(frame)

        detections = net.Detect(cuda_frame)

        max_distance = float('inf')
        closest_detection = None

        # Calculamos el centro de la imagen de la cámara
        cam_center_x = frame.shape[1] // 2
        cam_center_y = frame.shape[0] // 2

        # Dibujamos las líneas vertical y horizontal en el centro de la imagen de la cámara
        cv2.line(frame, (cam_center_x, 0), (cam_center_x, frame.shape[0]), (0, 0, 255), 2)
        cv2.line(frame, (0, cam_center_y), (frame.shape[1], cam_center_y), (0, 0, 255), 2)

        for detection in detections:
            item = net.GetClassDesc(detection.ClassID)

            if item == "lata":
                confidence = round(detection.Confidence * 100, 2)
                color = (0, 255, 0) if confidence > 90 else (0, 255, 255)

                # Obtener las coordenadas del punto rojo
                center_x = int(detection.Left + (detection.Width / 2))
                center_y = int(detection.Top + (detection.Height / 2))
                distance = np.sqrt((center_x - cam_center_x)**2 + (center_y - cam_center_y)**2) # Calcular la distancia euclidiana desde el centro de la cámara hasta el punto rojo de la lata
                cv2.circle(frame, (center_x, center_y), 10, (0, 0, 255), -1) # Dibujar un punto en el centro del bounding box (color rojo)

                # Verificar si esta lata está más cerca del centro que la anteriormente más cercana
                if distance < max_distance:
                    max_distance = distance
                    closest_detection = detection

                # Dibujar el área del bounding box
                left = int(detection.Left)
                top = int(detection.Top)
                right = int(detection.Right)
                bottom = int(detection.Bottom)
                cv2.rectangle(frame, (left, top), (right, bottom), color, 10)

                text = f"{item}: {confidence}%"
                cv2.putText(frame, text, (left, top - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                text_coords = f"Centro: ({center_x}, {center_y})"
                cv2.putText(frame, text_coords, (left, bottom + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if closest_detection is not None:
            # Obtener las coordenadas del bounding box del objeto más cercano al centro de la cámara
            left = int(closest_detection.Left)
            top = int(closest_detection.Top)
            right = int(closest_detection.Right)
            bottom = int(closest_detection.Bottom)
            cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 5)

            # Calcular las coordenadas del centro y enviarlas a arduino
            center_x = int((closest_detection.Left + closest_detection.Right) / 2)
            center_y = int((closest_detection.Top + closest_detection.Bottom) / 2)
            angulo_x, angulo_y = pixeles_a_angulos(center_x, center_y, frame.shape[1], frame.shape[0])
            mandarArduino(angulo_x, angulo_y)


        cv2.imshow("Detection", frame)

        if cv2.waitKey(1) == ord("q"):
            break

''' 
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
'''

if __name__ == "__main__":
    left_camera = CSI_Camera()
    left_camera.open(
        gstreamer_pipeline(
            sensor_id=1,
            capture_width=320,
            capture_height=240,
            flip_method=2,
            display_width=640,
            display_height=480,
        )
    )
    left_camera.start()

    if left_camera.video_capture.isOpened():
        net = jetson.inference.detectNet(argv=['--model=Latas.onnx', '--labels=labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'])

        try:
            detect_and_display(left_camera, net)
        except Exception as e:
            print("Error:", str(e))

        left_camera.stop()
        left_camera.release()
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to open the camera")
'''






import cv2
import numpy as np
import jetson.inference
import jetson.utils
import threading
import serial

# Inicializar la comunicación serial con el Arduino
ser = serial.Serial('/dev/ttyACM0', 9600)  # Ajusta el puerto serial y la velocidad según corresponda

class CSI_Camera:

    def __init__(self):
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            # Grab the first frame to start the video capturing
            self.grabbed, self.frame = self.video_capture.read()

        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)


    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop(self):
        self.running = False
        # Kill the thread
        self.read_thread.join()
        self.read_thread = None

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()

def gstreamer_pipeline(
    sensor_id=1,
    capture_width=320,
    capture_height=240,
    display_width=320,
    display_height=240,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def mandarArduino(angulo_x, angulo_y):
    mensaje = f"{angulo_x},{angulo_y}\n"  # Convertir los ángulos a cadenas y agregar el carácter de terminación
    ser.write(mensaje.encode())  # Enviar el mensaje al Arduino a través del puerto serial

def pixeles_a_angulos(x_pixel, y_pixel, resolucion_x, resolucion_y):
    # Rango de movimiento en grados en el eje X
    angulo_inicial = 20  # Angulo inicial
    angulo_final = 120   # Angulo final
    angulo_central = 75  # Angulo central

    # Convertir las coordenadas de píxeles a porcentajes
    porcentaje_x = x_pixel / resolucion_x

    # Convertir porcentajes a ángulos
    angulo_x = angulo_inicial + porcentaje_x * (angulo_final - angulo_inicial)

    # Mantener el ángulo dentro del rango permitido
    angulo_x = min(angulo_final, max(angulo_inicial, angulo_x))

    return angulo_x, 0  # Solo se ajusta el ángulo horizontal, el ángulo vertical se deja en 0

def detect_and_display(camera, net):
    while True:
        grabbed, frame = camera.read()

        if not grabbed:
            print("Error: Cannot read frame from camera")
            break

        cuda_frame = jetson.utils.cudaFromNumpy(frame)

        detections = net.Detect(cuda_frame)

        max_distance = float('inf')
        closest_detection = None

        # Calculamos el centro de la imagen de la cámara
        cam_center_x = frame.shape[1] // 2
        cam_center_y = frame.shape[0] // 2

        # Dibujamos las líneas vertical y horizontal en el centro de la imagen de la cámara
        cv2.line(frame, (cam_center_x, 0), (cam_center_x, frame.shape[0]), (0, 0, 255), 2)
        cv2.line(frame, (0, cam_center_y), (frame.shape[1], cam_center_y), (0, 0, 255), 2)

        for detection in detections:
            item = net.GetClassDesc(detection.ClassID)

            if item == "lata":
                confidence = round(detection.Confidence * 100, 2)
                color = (0, 255, 0) if confidence > 90 else (0, 255, 255)

                # Obtener las coordenadas del punto rojo
                center_x = int(detection.Left + (detection.Width / 2))
                center_y = int(detection.Top + (detection.Height / 2))
                distance = np.sqrt((center_x - cam_center_x)**2 + (center_y - cam_center_y)**2) # Calcular la distancia euclidiana desde el centro de la cámara hasta el punto rojo de la lata
                cv2.circle(frame, (center_x, center_y), 10, (0, 0, 255), -1) # Dibujar un punto en el centro del bounding box (color rojo)

                # Verificar si esta lata está más cerca del centro que la anteriormente más cercana
                if distance < max_distance:
                    max_distance = distance
                    closest_detection = detection

                # Dibujar el área del bounding box
                left = int(detection.Left)
                top = int(detection.Top)
                right = int(detection.Right)
                bottom = int(detection.Bottom)
                cv2.rectangle(frame, (left, top), (right, bottom), color, 10)

                text = f"{item}: {confidence}%"
                cv2.putText(frame, text, (left, top - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                text_coords = f"Centro: ({center_x}, {center_y})"
                cv2.putText(frame, text_coords, (left, bottom + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if closest_detection is not None:
            # Obtener las coordenadas del bounding box del objeto más cercano al centro de la cámara
            center_x = int((closest_detection.Left + closest_detection.Right) / 2)
            angulo_x = pixeles_a_angulos(center_x, cam_center_y, frame.shape[1], frame.shape[0])[0]
            mandarArduino(angulo_x, 0)

        cv2.imshow("Detection", frame)

        if cv2.waitKey(1) == ord("q"):
            break

if __name__ == "__main__":
    left_camera = CSI_Camera()
    left_camera.open(
        gstreamer_pipeline(
            sensor_id=1,
            capture_width=320,
            capture_height=240,
            flip_method=2,
            display_width=640,
            display_height=480,
        )
    )
    left_camera.start()

    if left_camera.video_capture.isOpened():
        net = jetson.inference.detectNet(argv=['--model=Latas.onnx', '--labels=labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'])

        try:
            detect_and_display(left_camera, net)
        except Exception as e:
            print("Error:", str(e))

        left_camera.stop()
        left_camera.release()
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to open the camera")

