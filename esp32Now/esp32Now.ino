#include <esp_now.h>
#include <WiFi.h>

// Estructura para recibir datos
typedef struct struct_message {
    int camaraID; // ID de la cámara (1: frontal derecha, 2: frontal izquierda, 3: posterior derecha, 4: posterior izquierda)
    bool azulDetectado; // Indica si se detectó el color azul
} struct_message;

struct_message myData;

// Función de callback para recibir datos
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("Cámara ID: ");
    Serial.print(myData.camaraID);
    Serial.print(", Azul detectado: ");
    Serial.println(myData.azulDetectado ? "Sí" : "No");

    // Aquí puedes agregar lógica para enviar los datos a ROS
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error al inicializar ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    // No es necesario hacer nada en el loop
}