// ════════════════════════════════════════════════
//  ESP32-C3 Super Mini — CanSat completo
//  Telemetría + Cámaras + nRF24 con ACK por paquete
// ════════════════════════════════════════════════
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// ════════════════════════════════════════════════
//  PINES
// ════════════════════════════════════════════════
#define LED_PIN   8
#define NRF_CE    1
#define NRF_CSN   3
#define SDA_PIN   7
#define SCL_PIN   10
#define CAM1_RX   20
#define CAM1_TX   21
#define CAM2_RX   2
#define CAM2_TX   9

// ════════════════════════════════════════════════
//  CONFIGURACIÓN
// ════════════════════════════════════════════════
#define ALTITUD_FOTO      2.0     // metros — cambia a 100.0 en vuelo
#define SEALEVELPRESSURE  1013.25
#define FOTO_TIMEOUT      25000   // ms esperando respuesta de cámara
#define FRAG_SIZE         24      // bytes de foto por paquete (reducido para dar espacio al header)
#define MAX_REINTENTOS    3       // veces que reintenta un frag perdido
#define ACK_TIMEOUT       50      // ms esperando ACK del receptor

// ════════════════════════════════════════════════
//  PROTOCOLO
// ════════════════════════════════════════════════
#define PKT_TELEMETRY   0x01
#define PKT_FOTO_INICIO 0x02
#define PKT_FOTO_FRAG   0x03
#define PKT_FOTO_FIN    0x04
#define PKT_ACK         0x05

// Pipes — TX y RX necesitan pipes distintos para ACK bidireccional
const byte PIPE_TX[6] = "CSAT1";  // C3 transmite por aquí
const byte PIPE_RX[6] = "CSAT2";  // C3 recibe ACKs por aquí

// ════════════════════════════════════════════════
//  STRUCTS
// ════════════════════════════════════════════════
struct __attribute__((packed)) Telemetry {
  uint8_t tipo = PKT_TELEMETRY;
  float   temp;
  float   pressure;
  float   altitude;
  float   yaw;
  float   pitch;
  float   roll;
  uint8_t sys;
  uint8_t gyro;
  uint8_t accel;
  uint8_t mag;
};

struct __attribute__((packed)) InicioPkt {
  uint8_t  tipo       = PKT_FOTO_INICIO;
  uint8_t  cam_id;
  uint32_t size_total;
  uint16_t n_frags;
};

struct __attribute__((packed)) FragPkt {
  uint8_t  tipo       = PKT_FOTO_FRAG;
  uint8_t  cam_id;
  uint16_t frag_idx;
  uint16_t frag_total;
  uint8_t  datos[FRAG_SIZE];
};  // 6 + 24 = 30 bytes, cabe en 32 del nRF

struct __attribute__((packed)) FinPkt {
  uint8_t tipo   = PKT_FOTO_FIN;
  uint8_t cam_id;
};

struct __attribute__((packed)) AckPkt {
  uint8_t  tipo = PKT_ACK;
  uint8_t  cam_id;
  uint16_t frag_idx;
  uint8_t  ok;  // 1 = recibido OK, 0 = pedir reenvío
};

// ════════════════════════════════════════════════
//  OBJETOS
// ════════════════════════════════════════════════
RF24            radio(NRF_CE, NRF_CSN);
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
HardwareSerial  CAM(1);

Telemetry telData;
bool      fotoTomada = false;

// ════════════════════════════════════════════════
//  HELPERS nRF — alternar TX/RX
// ════════════════════════════════════════════════
void modoTX() {
  radio.stopListening();
  radio.openWritingPipe(PIPE_TX);
  radio.openReadingPipe(1, PIPE_RX);
}

void modoRX() {
  radio.openWritingPipe(PIPE_TX);
  radio.openReadingPipe(1, PIPE_RX);
  radio.startListening();
}

// ════════════════════════════════════════════════
//  LEER SENSORES
// ════════════════════════════════════════════════
bool leerSensores() {
  if (!bmp.performReading()) return false;
  sensors_event_t ev;
  bno.getEvent(&ev, Adafruit_BNO055::VECTOR_EULER);
  bno.getCalibration(&telData.sys, &telData.gyro, &telData.accel, &telData.mag);
  telData.tipo     = PKT_TELEMETRY;
  telData.temp     = bmp.temperature;
  telData.pressure = bmp.pressure / 100.0;
  telData.altitude = bmp.readAltitude(SEALEVELPRESSURE);
  telData.yaw      = ev.orientation.x;
  telData.pitch    = ev.orientation.y;
  telData.roll     = ev.orientation.z;
  return true;
}

// ════════════════════════════════════════════════
//  ENVIAR TELEMETRÍA
// ════════════════════════════════════════════════
void enviarTelemetria() {
  modoTX();
  digitalWrite(LED_PIN, HIGH);
  radio.write(&telData, sizeof(telData));
  digitalWrite(LED_PIN, LOW);

  Serial.print("{\"temp\":");      Serial.print(telData.temp, 2);
  Serial.print(",\"pressure\":"); Serial.print(telData.pressure, 2);
  Serial.print(",\"altitude\":"); Serial.print(telData.altitude, 2);
  Serial.print(",\"yaw\":");      Serial.print(telData.yaw, 2);
  Serial.print(",\"pitch\":");    Serial.print(telData.pitch, 2);
  Serial.print(",\"roll\":");     Serial.print(telData.roll, 2);
  Serial.print(",\"calib\":{\"sys\":"); Serial.print(telData.sys);
  Serial.print(",\"gyro\":");     Serial.print(telData.gyro);
  Serial.print(",\"accel\":");    Serial.print(telData.accel);
  Serial.print(",\"mag\":");      Serial.print(telData.mag);
  Serial.println("}}");
}

// ════════════════════════════════════════════════
//  DISPARAR CÁMARA
// ════════════════════════════════════════════════
bool dispararCamara(int cam_id, uint8_t** buf_out, uint32_t* len_out) {
  int rx = (cam_id == 1) ? CAM1_RX : CAM2_RX;
  int tx = (cam_id == 1) ? CAM1_TX : CAM2_TX;

  CAM.begin(115200, SERIAL_8N1, rx, tx);
  delay(300);
  while (CAM.available()) CAM.read();
  delay(50);

  CAM.println("TAKE");
  CAM.flush();
  Serial.printf("{\"cam\":%d,\"cmd\":\"TAKE\"}\n", cam_id);

  unsigned long t = millis();
  byte prev = 0;

  while (millis() - t < FOTO_TIMEOUT) {
    // Telemetría intercalada cada 2s mientras espera
    static unsigned long ultTel = 0;
    if (millis() - ultTel > 2000) {
      ultTel = millis();
      if (leerSensores()) enviarTelemetria();
    }

    if (!CAM.available()) continue;
    byte b = CAM.read();

    if (prev == 0xFF && b == 0xBB) {
      Serial.printf("{\"error\":\"CAM%d error interno\"}\n", cam_id);
      CAM.end();
      return false;
    }

    if (prev == 0xFF && b == 0xAA) {
      // Esperar tamaño
      unsigned long tt = millis();
      while (CAM.available() < 4 && millis() - tt < 3000);
      if (CAM.available() < 4) {
        Serial.printf("{\"error\":\"CAM%d timeout tamaño\"}\n", cam_id);
        CAM.end();
        return false;
      }

      uint32_t size = 0;
      CAM.readBytes((char*)&size, 4);

      if (size == 0 || size > 80000) {
        Serial.printf("{\"error\":\"CAM%d tamaño inválido %u\"}\n", cam_id, size);
        CAM.end();
        return false;
      }

      Serial.printf("{\"cam\":%d,\"bytes\":%u}\n", cam_id, size);

      uint8_t* buf = (uint8_t*)malloc(size);
      if (!buf) {
        Serial.printf("{\"error\":\"CAM%d sin memoria\"}\n", cam_id);
        CAM.end();
        return false;
      }

      uint32_t recibidos = 0;
      unsigned long t2 = millis();
      while (recibidos < size && millis() - t2 < FOTO_TIMEOUT) {
        int disp = CAM.available();
        if (disp <= 0) continue;
        uint32_t leer = min((uint32_t)disp, min((uint32_t)128, size - recibidos));
        int n = CAM.readBytes((char*)(buf + recibidos), leer);
        if (n > 0) { recibidos += n; t2 = millis(); }
      }

      CAM.end();

      if (recibidos != size) {
        Serial.printf("{\"error\":\"CAM%d incompleto %u/%u\"}\n", cam_id, recibidos, size);
        free(buf);
        return false;
      }

      *buf_out = buf;
      *len_out = size;
      Serial.printf("{\"cam\":%d,\"status\":\"OK\"}\n", cam_id);
      return true;
    }

    prev = b;
  }

  Serial.printf("{\"error\":\"CAM%d timeout header\"}\n", cam_id);
  CAM.end();
  return false;
}

// ════════════════════════════════════════════════
//  ESPERAR ACK DEL RECEPTOR
// ════════════════════════════════════════════════
bool esperarAck(uint8_t cam_id, uint16_t frag_idx) {
  modoRX();
  unsigned long t = millis();

  while (millis() - t < ACK_TIMEOUT) {
    if (!radio.available()) continue;

    AckPkt ack;
    radio.read(&ack, sizeof(ack));

    if (ack.tipo == PKT_ACK && ack.cam_id == cam_id && ack.frag_idx == frag_idx) {
      modoTX();
      return ack.ok == 1;
    }
  }

  modoTX();
  return false;  // timeout = paquete perdido
}

// ════════════════════════════════════════════════
//  ENVIAR FOTO POR nRF CON ACK Y REINTENTO
// ════════════════════════════════════════════════
void enviarFotoNRF(uint8_t cam_id, uint8_t* buf, uint32_t size) {
  uint16_t n_frags = (size + FRAG_SIZE - 1) / FRAG_SIZE;

  Serial.printf("{\"nrf\":\"inicio\",\"cam\":%d,\"frags\":%u}\n", cam_id, n_frags);

  // Paquete de inicio — sin ACK, se reenvía si el receptor lo pide implícito
  modoTX();
  InicioPkt inicio;
  inicio.cam_id     = cam_id;
  inicio.size_total = size;
  inicio.n_frags    = n_frags;
  radio.write(&inicio, sizeof(inicio));
  delay(20);

  uint16_t perdidos = 0;

  for (uint16_t i = 0; i < n_frags; i++) {
    // Telemetría cada 5 fragmentos
    if (i % 5 == 0) {
      if (leerSensores()) enviarTelemetria();
    }

    // Armar fragmento
    FragPkt frag;
    frag.cam_id     = cam_id;
    frag.frag_idx   = i;
    frag.frag_total = n_frags;
    uint32_t offset = (uint32_t)i * FRAG_SIZE;
    uint32_t chunk  = min((uint32_t)FRAG_SIZE, size - offset);
    memset(frag.datos, 0, FRAG_SIZE);
    memcpy(frag.datos, buf + offset, chunk);

    // Intentar enviar con reintentos
    bool confirmado = false;
    for (int intento = 0; intento < MAX_REINTENTOS && !confirmado; intento++) {
      modoTX();
      radio.write(&frag, sizeof(frag));
      confirmado = esperarAck(cam_id, i);

      if (!confirmado) {
        Serial.printf("{\"nrf\":\"reintento\",\"cam\":%d,\"frag\":%u,\"intento\":%d}\n",
                      cam_id, i, intento + 1);
        delay(10);
      }
    }

    if (!confirmado) {
      perdidos++;
      Serial.printf("{\"nrf\":\"perdido\",\"cam\":%d,\"frag\":%u}\n", cam_id, i);
    } else {
      Serial.printf("{\"nrf\":\"ok\",\"cam\":%d,\"frag\":%u,\"total\":%u}\n",
                    cam_id, i, n_frags);
    }

    digitalWrite(LED_PIN, i % 2);
  }

  // Paquete de fin
  modoTX();
  FinPkt fin;
  fin.cam_id = cam_id;
  radio.write(&fin, sizeof(fin));
  digitalWrite(LED_PIN, LOW);

  Serial.printf("{\"nrf\":\"fin\",\"cam\":%d,\"enviados\":%u,\"perdidos\":%u}\n",
                cam_id, n_frags - perdidos, perdidos);
}

// ════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(2000);
  Serial.println("{\"status\":\"BOOT\"}");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bmp.begin_I2C(0x77)) {
    Serial.println("{\"error\":\"BMP390\"}");
    while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  if (!bno.begin()) {
    Serial.println("{\"error\":\"BNO055\"}");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  if (!radio.begin()) {
    Serial.println("{\"error\":\"NRF24\"}");
    while (1);
  }
  radio.setChannel(2);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  modoTX();

  Serial.println("{\"status\":\"READY\"}");
}

// ════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════
void loop() {
  if (!leerSensores()) { delay(100); return; }
  enviarTelemetria();

  if (!fotoTomada && telData.altitude >= ALTITUD_FOTO) {
    fotoTomada = true;
    Serial.println("{\"evento\":\"DISPARO_FOTO\"}");

    uint8_t  *buf1 = nullptr, *buf2 = nullptr;
    uint32_t  len1 = 0,        len2 = 0;

    bool ok1 = dispararCamara(1, &buf1, &len1);
    bool ok2 = dispararCamara(2, &buf2, &len2);

    if (ok1) { enviarFotoNRF(1, buf1, len1); free(buf1); }
    else      { Serial.println("{\"error\":\"CAM1 fallo\"}"); }

    if (ok2) { enviarFotoNRF(2, buf2, len2); free(buf2); }
    else      { Serial.println("{\"error\":\"CAM2 fallo\"}"); }

    Serial.println("{\"evento\":\"FOTOS_LISTAS\"}");
  }

  delay(200);
}
