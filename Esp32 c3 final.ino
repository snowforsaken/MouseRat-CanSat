// ════════════════════════════════════════════════
//  ESP32-C3 Super Mini — CanSat (1 cámara)
//  Telemetría + Cámara (RX=2, TX=9) + nRF24 con ACK por paquete
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
#define CAM_RX    2
#define CAM_TX    9

// ════════════════════════════════════════════════
//  CONFIGURACIÓN — ajusta antes de cada vuelo
// ════════════════════════════════════════════════
#define ALTITUD_FOTO      2.0   // metros — pon 2.0 para pruebas en banco
#define SEALEVELPRESSURE  1013.25
#define FOTO_TIMEOUT      25000   // ms esperando respuesta de cámara
#define FRAG_SIZE         24      // bytes de foto por paquete nRF
#define MAX_REINTENTOS    3
#define ACK_TIMEOUT       100     // ms esperando ACK del receptor

// ════════════════════════════════════════════════
//  PROTOCOLO
// ════════════════════════════════════════════════
#define PKT_TELEMETRY   0x01
#define PKT_FOTO_INICIO 0x02
#define PKT_FOTO_FRAG   0x03
#define PKT_FOTO_FIN    0x04
#define PKT_ACK         0x05

const byte PIPE_TX[6] = "CSAT1";
const byte PIPE_RX[6] = "CSAT2";

// ════════════════════════════════════════════════
//  STRUCTS — todos exactamente 32 bytes para nRF
// ════════════════════════════════════════════════

// Telemetría: 1+4+4+4+4+4+4+1+1+1+1 = 29 bytes → padding a 32
struct __attribute__((packed)) TelPkt {
  uint8_t tipo;
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
  uint8_t pad[3];
};

// Inicio de foto: 8 bytes útiles → padding a 32
struct __attribute__((packed)) InicioPkt {
  uint8_t tipo;
  uint8_t cam_id;
  uint8_t size_b0;
  uint8_t size_b1;
  uint8_t size_b2;
  uint8_t size_b3;
  uint8_t nf_b0;
  uint8_t nf_b1;
  uint8_t pad[24];
};

// Fragmento: 6 bytes header + 24 datos + 2 pad = 32 bytes
struct __attribute__((packed)) FragPkt {
  uint8_t tipo;
  uint8_t cam_id;
  uint8_t idx_b0;
  uint8_t idx_b1;
  uint8_t tot_b0;
  uint8_t tot_b1;
  uint8_t datos[FRAG_SIZE];
  uint8_t pad[2];
};

// Fin: 2 bytes útiles → padding a 32
struct __attribute__((packed)) FinPkt {
  uint8_t tipo;
  uint8_t cam_id;
  uint8_t pad[30];
};

// ACK: 5 bytes útiles → padding a 32
struct __attribute__((packed)) AckPkt {
  uint8_t tipo;
  uint8_t cam_id;
  uint8_t idx_b0;
  uint8_t idx_b1;
  uint8_t ok;
  uint8_t pad[27];
};

// ════════════════════════════════════════════════
//  HELPERS — empacar/desempacar valores multi-byte
// ════════════════════════════════════════════════
inline void pack16(uint8_t* dst, uint16_t val) {
  dst[0] = val & 0xFF;
  dst[1] = (val >> 8) & 0xFF;
}
inline void pack32(uint8_t* dst, uint32_t val) {
  dst[0] = val & 0xFF;
  dst[1] = (val >> 8) & 0xFF;
  dst[2] = (val >> 16) & 0xFF;
  dst[3] = (val >> 24) & 0xFF;
}
inline uint16_t unpack16(const uint8_t* src) {
  return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}
inline uint32_t unpack32(const uint8_t* src) {
  return (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
         ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

// ════════════════════════════════════════════════
//  OBJETOS
// ════════════════════════════════════════════════
RF24            radio(NRF_CE, NRF_CSN);
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
HardwareSerial  CAM(1);

TelPkt   telPkt;
bool     fotoTomada    = false;
uint32_t ultTelemetria = 0;

// ════════════════════════════════════════════════
//  nRF — alternar modos TX/RX
// ════════════════════════════════════════════════
void modoTX() {
  radio.stopListening();
  radio.openWritingPipe(PIPE_TX);
  radio.openReadingPipe(1, PIPE_RX);
}

void modoRX() {
  radio.stopListening();
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
  bno.getCalibration(&telPkt.sys, &telPkt.gyro, &telPkt.accel, &telPkt.mag);

  telPkt.tipo     = PKT_TELEMETRY;
  telPkt.temp     = bmp.temperature;
  telPkt.pressure = bmp.pressure / 100.0f;
  telPkt.altitude = bmp.readAltitude(SEALEVELPRESSURE);
  telPkt.yaw      = ev.orientation.x;
  telPkt.pitch    = ev.orientation.y;
  telPkt.roll     = ev.orientation.z;
  memset(telPkt.pad, 0, sizeof(telPkt.pad));
  return true;
}

// ════════════════════════════════════════════════
//  ENVIAR TELEMETRÍA
// ════════════════════════════════════════════════
void enviarTelemetria() {
  modoTX();
  uint8_t buf[32];
  memcpy(buf, &telPkt, 32);
  digitalWrite(LED_PIN, HIGH);
  radio.write(buf, 32);
  digitalWrite(LED_PIN, LOW);

  Serial.print(F("{\"temp\":"));       Serial.print(telPkt.temp, 2);
  Serial.print(F(",\"pressure\":"));   Serial.print(telPkt.pressure, 2);
  Serial.print(F(",\"altitude\":"));   Serial.print(telPkt.altitude, 2);
  Serial.print(F(",\"yaw\":"));        Serial.print(telPkt.yaw, 2);
  Serial.print(F(",\"pitch\":"));      Serial.print(telPkt.pitch, 2);
  Serial.print(F(",\"roll\":"));       Serial.print(telPkt.roll, 2);
  Serial.print(F(",\"calib\":{\"sys\":")); Serial.print(telPkt.sys);
  Serial.print(F(",\"gyro\":"));       Serial.print(telPkt.gyro);
  Serial.print(F(",\"accel\":"));      Serial.print(telPkt.accel);
  Serial.print(F(",\"mag\":"));        Serial.print(telPkt.mag);
  Serial.println(F("}}"));

  ultTelemetria = millis();
}

// ════════════════════════════════════════════════
//  TELEMETRÍA INTERCALADA (durante captura/envío)
// ════════════════════════════════════════════════
void telemetriaIntercalada() {
  if (millis() - ultTelemetria >= 2000) {
    if (leerSensores()) enviarTelemetria();
  }
}

// ════════════════════════════════════════════════
//  DISPARAR CÁMARA Y RECIBIR FOTO EN RAM
// ════════════════════════════════════════════════
bool dispararCamara(uint8_t** buf_out, uint32_t* len_out) {
  CAM.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);
  delay(300);
  while (CAM.available()) CAM.read();
  delay(50);

  CAM.println("TAKE");
  CAM.flush();
  Serial.println(F("{\"cam\":1,\"cmd\":\"TAKE\"}"));

  unsigned long t = millis();
  byte prev = 0;

  while (millis() - t < FOTO_TIMEOUT) {
    telemetriaIntercalada();

    if (!CAM.available()) continue;
    byte b = CAM.read();

    if (prev == 0xFF && b == 0xBB) {
      Serial.println(F("{\"error\":\"CAM error interno\"}"));
      CAM.end();
      return false;
    }

    if (prev == 0xFF && b == 0xAA) {
      // Esperar 4 bytes de tamaño
      unsigned long tt = millis();
      while (CAM.available() < 4 && millis() - tt < 3000);
      if (CAM.available() < 4) {
        Serial.println(F("{\"error\":\"CAM timeout tamanio\"}"));
        CAM.end();
        return false;
      }

      // Leer tamaño byte a byte (seguro contra endian)
      uint8_t sb[4];
      CAM.readBytes((char*)sb, 4);
      uint32_t size = unpack32(sb);

      if (size == 0 || size > 80000) {
        Serial.printf("{\"error\":\"CAM tamanio invalido %u\"}\n", size);
        CAM.end();
        return false;
      }

      Serial.printf("{\"cam\":1,\"bytes\":%u}\n", size);

      uint8_t* buf = (uint8_t*)malloc(size);
      if (!buf) {
        Serial.println(F("{\"error\":\"CAM sin memoria\"}"));
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
        Serial.printf("{\"error\":\"CAM incompleto %u/%u\"}\n", recibidos, size);
        free(buf);
        return false;
      }

      *buf_out = buf;
      *len_out = size;
      Serial.println(F("{\"cam\":1,\"status\":\"OK\"}"));
      return true;
    }

    prev = b;
  }

  Serial.println(F("{\"error\":\"CAM timeout header\"}"));
  CAM.end();
  return false;
}

// ════════════════════════════════════════════════
//  ESPERAR ACK DEL RECEPTOR
// ════════════════════════════════════════════════
bool esperarAck(uint16_t frag_idx) {
  modoRX();
  unsigned long t = millis();
  uint8_t rxbuf[32];

  while (millis() - t < ACK_TIMEOUT) {
    if (!radio.available()) continue;

    radio.read(rxbuf, 32);

    if (rxbuf[0] == PKT_ACK && rxbuf[1] == 1) {
      uint16_t idx = unpack16(&rxbuf[2]);
      if (idx == frag_idx) {
        modoTX();
        return rxbuf[4] == 1;
      }
    }
  }

  modoTX();
  return false;
}

// ════════════════════════════════════════════════
//  ENVIAR FOTO POR nRF CON ACK Y REINTENTO
// ════════════════════════════════════════════════
void enviarFotoNRF(uint8_t* buf, uint32_t size) {
  uint16_t n_frags = (size + FRAG_SIZE - 1) / FRAG_SIZE;
  Serial.printf("{\"nrf\":\"inicio\",\"cam\":1,\"frags\":%u}\n", n_frags);

  // Paquete de inicio
  uint8_t inicioBuf[32];
  memset(inicioBuf, 0, 32);
  inicioBuf[0] = PKT_FOTO_INICIO;
  inicioBuf[1] = 1;  // cam_id fijo = 1
  pack32(&inicioBuf[2], size);
  pack16(&inicioBuf[6], n_frags);

  modoTX();
  radio.write(inicioBuf, 32);
  delay(20);

  uint16_t perdidos = 0;

  for (uint16_t i = 0; i < n_frags; i++) {
    if (i % 5 == 0) telemetriaIntercalada();

    // Armar fragmento en buffer de 32 bytes
    uint8_t fragBuf[32];
    memset(fragBuf, 0, 32);
    fragBuf[0] = PKT_FOTO_FRAG;
    fragBuf[1] = 1;  // cam_id fijo = 1
    pack16(&fragBuf[2], i);
    pack16(&fragBuf[4], n_frags);

    uint32_t offset = (uint32_t)i * FRAG_SIZE;
    uint32_t chunk  = min((uint32_t)FRAG_SIZE, size - offset);
    memcpy(&fragBuf[6], buf + offset, chunk);

    // Reintentos con ACK
    bool confirmado = false;
    for (int intento = 0; intento < MAX_REINTENTOS && !confirmado; intento++) {
      modoTX();
      radio.write(fragBuf, 32);
      confirmado = esperarAck(i);

      if (!confirmado) {
        Serial.printf("{\"nrf\":\"reintento\",\"cam\":1,\"frag\":%u,\"intento\":%d}\n",
                      i, intento + 1);
        delay(10);
      }
    }

    if (!confirmado) {
      perdidos++;
      Serial.printf("{\"nrf\":\"perdido\",\"cam\":1,\"frag\":%u}\n", i);
    } else {
      Serial.printf("{\"nrf\":\"ok\",\"cam\":1,\"frag\":%u,\"total\":%u}\n", i, n_frags);
    }

    digitalWrite(LED_PIN, i % 2);
  }

  // Paquete de fin
  uint8_t finBuf[32];
  memset(finBuf, 0, 32);
  finBuf[0] = PKT_FOTO_FIN;
  finBuf[1] = 1;
  modoTX();
  radio.write(finBuf, 32);
  digitalWrite(LED_PIN, LOW);

  Serial.printf("{\"nrf\":\"fin\",\"cam\":1,\"enviados\":%u,\"perdidos\":%u}\n",
                n_frags - perdidos, perdidos);
}

// ════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(2000);
  Serial.println(F("{\"status\":\"BOOT\"}"));

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bmp.begin_I2C(0x77)) {
    Serial.println(F("{\"error\":\"BMP390\"}"));
    while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  if (!bno.begin()) {
    Serial.println(F("{\"error\":\"BNO055\"}"));
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  if (!radio.begin()) {
    Serial.println(F("{\"error\":\"NRF24\"}"));
    while (1);
  }
  radio.setChannel(2);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(5, 15);
  radio.enableDynamicPayloads();
  modoTX();

  Serial.println(F("{\"status\":\"READY\"}"));
}

// ════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════
void loop() {
  if (!leerSensores()) { delay(100); return; }
  enviarTelemetria();

  if (!fotoTomada && telPkt.altitude >= ALTITUD_FOTO) {
    fotoTomada = true;
    Serial.println(F("{\"evento\":\"DISPARO_FOTO\"}"));

    uint8_t* buf = nullptr;
    uint32_t len = 0;

    if (dispararCamara(&buf, &len)) {
      enviarFotoNRF(buf, len);
      free(buf);
    } else {
      Serial.println(F("{\"error\":\"CAM fallo\"}"));
    }

    Serial.println(F("{\"evento\":\"FOTO_LISTA\"}"));
  }

  delay(200);
}
