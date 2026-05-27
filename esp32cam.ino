// ════════════════════════════════════════════════
//  ESP32-CAM — Código para CAM1 y CAM2
//  Solo cambia CAM_ID: 1 para la primera, 2 para la segunda
//  Pines UART: RX=13, TX=2 (iguales en ambas)
// ════════════════════════════════════════════════
#include "esp_camera.h"
#include "img_converters.h"

#define CAM_ID  1     // ← Cambia a 2 en la segunda cámara

#define RX_PIN  13
#define TX_PIN  2

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Configuración de la cámara
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = 5;
  config.pin_d1       = 18;
  config.pin_d2       = 19;
  config.pin_d3       = 21;
  config.pin_d4       = 36;
  config.pin_d5       = 39;
  config.pin_d6       = 34;
  config.pin_d7       = 35;
  config.pin_xclk     = 0;
  config.pin_pclk     = 22;
  config.pin_vsync    = 25;
  config.pin_href     = 23;
  config.pin_sccb_sda = 26;
  config.pin_sccb_scl = 27;
  config.pin_pwdn     = 32;
  config.pin_reset    = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565; // RGB565 → convertimos a JPEG
  config.frame_size   = FRAMESIZE_QQVGA;  // 160x120 — mínimo tamaño
  config.jpeg_quality = 20;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error camara: 0x%x\n", err);
    while (true) delay(1000);
  }

  // Descartar primeros frames para estabilizar el sensor
  for (int i = 0; i < 5; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(100);
  }

  Serial.printf("CAM%d lista\n", CAM_ID);
}

void loop() {
  if (!Serial1.available()) return;

  String cmd = Serial1.readStringUntil('\n');
  cmd.trim();
  Serial.println("Recibido: [" + cmd + "]");

  if (cmd == "TAKE") {

    // Vaciar buffer para capturar el frame del instante actual
    camera_fb_t *fb_viejo = esp_camera_fb_get();
    if (fb_viejo) esp_camera_fb_return(fb_viejo);
    delay(150);

    // Capturar frame fresco
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Error capturando frame");
      Serial1.write(0xFF);
      Serial1.write(0xBB); // Header de error
      return;
    }

    // Convertir RGB565 → JPEG con compresión alta para minimizar bytes
    uint8_t *jpg_buf = NULL;
    size_t   jpg_len = 0;
    bool ok = frame2jpg(fb, 20, &jpg_buf, &jpg_len); // 20 = muy comprimido
    esp_camera_fb_return(fb);

    if (!ok || jpg_buf == NULL) {
      Serial.println("Error convirtiendo a JPEG");
      Serial1.write(0xFF);
      Serial1.write(0xBB);
      return;
    }

    Serial.printf("Enviando %u bytes\n", jpg_len);

    // Enviar: header (2B) + tamaño (4B) + datos
    Serial1.write(0xFF);
    Serial1.write(0xAA);
    Serial1.write((uint8_t*)&jpg_len, 4);

    const uint8_t *ptr = jpg_buf;
    uint32_t remaining = jpg_len;
    while (remaining > 0) {
      size_t chunk = min((uint32_t)256, remaining);
      Serial1.write(ptr, chunk);
      ptr       += chunk;
      remaining -= chunk;
    }

    free(jpg_buf);
    Serial.println("Enviado OK");
  }
}
