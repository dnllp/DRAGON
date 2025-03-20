#include <esp_now.h>
#include <WiFi.h>
#include "esp_camera.h"

// Configuración de la cámara
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// Dirección MAC del ESP32 receptor
uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56}; // Cambia esto por la dirección MAC del receptor

// Estructura para enviar datos
typedef struct struct_message {
    bool azulDetectado;
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nEstado del último paquete enviado:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Envío exitoso" : "Envío fallido");
}

void setup() {
    Serial.begin(115200);

    // Inicialización de la cámara
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if(psramFound()){
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    // Iniciar la cámara
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error al iniciar la cámara: 0x%x", err);
        return;
    }

    // Inicializar ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error al inicializar ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    // Registrar peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Error al registrar peer");
        return;
    }
}

void loop() {
    // Capturar una imagen
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Error al capturar la imagen");
        return;
    }

    // Procesar la imagen para detectar el color azul
    bool azulDetectado = detectarAzul(fb->buf, fb->width, fb->height);

    // Enviar el resultado por ESP-NOW
    myData.azulDetectado = azulDetectado;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    if (result == ESP_OK) {
        Serial.println("Envío exitoso");
    } else {
        Serial.println("Error al enviar");
    }

    // Liberar el buffer de la imagen
    esp_camera_fb_return(fb);

    delay(1000); // Esperar 1 segundo antes de la siguiente captura
}

bool detectarAzul(uint8_t *img, int width, int height) {
    // Implementar la lógica de detección de color azul
    // Este es un ejemplo simple que cuenta píxeles azules
    int azulCount = 0;
    for (int i = 0; i < width * height * 3; i += 3) {
        uint8_t r = img[i];
        uint8_t g = img[i + 1];
        uint8_t b = img[i + 2];

        if (b > r && b > g) { // Si el azul es dominante
            azulCount++;
        }
    }

    // Si se detecta suficiente azul, devolver true
    return azulCount > (width * height * 0.1); // 10% de píxeles azules
}