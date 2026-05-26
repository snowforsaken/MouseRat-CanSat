// ════════════════════════════════════════════════
//  ESP32 Dev Module — Receptor en tierra
//  Recibe telemetría e imagen por nRF24
//  Manda ACK por cada fragmento de imagen
//  Reenvía todo por Serial al dashboard Python
// ════════════════════════════════════════════════
#include <SPI.h>
#include <RF24.h>

#define NRF_CE   4
#define NRF_CSN  5
#define LED_RX   2

RF24 radio(NRF_CE, NRF_CSN);

// Pipes — inversos al C3
const byte PIPE_TX[6] = "CSAT1";  // receptor escucha aquí
const byte PIPE_RX[6] = "CSAT2";  // receptor manda ACKs aquí

// ════════════════════════════════════════════════
//  PROTOCOLO
// ════════════════════════════════════════════════
#define PKT_TELEMETRY   0x01
#define PKT_FOTO_INICIO 0x02
#define PKT_FOTO_FRAG   0x03
#define PKT_FOTO_FIN    0x04
#define PKT_ACK         0x05
#define FRAG_SIZE       24

const uint8_t MAGIC[4] = {0xDE, 0xAD, 0xBE, 0xEF};

// ════════════════════════════════════════════════
//  STRUCTS
// ════════════════════════════════════════════════
struct __attribute__((packed)) Telemetry {
  uint8_t tipo;
  float   temp, pressure, altitude, yaw, pitch, roll;
  uint8_t sys, gyro, accel, mag;
};

struct __attribute__((packed)) InicioPkt {
  uint8_t  tipo;
  uint8_t  cam_id;
  uint32_t size_total;
  uint16_t n_frags;
};

struct __attribute__((packed)) FragPkt {
  uint8_t  tipo;
  uint8_t  cam_id;
  uint16_t frag_idx;
  uint16_t frag_total;
  uint8_t  datos[FRAG_SIZE];
};

struct __attribute__((packed)) FinPkt {
  uint8_t tipo;
  uint8_t cam_id;
};

struct __attribute__((packed)) AckPkt {
  uint8_t  tipo = PKT_ACK;
  uint8_t  cam_id;
  uint16_t frag_idx;
  uint8_t  ok;
};

uint8_t buf[32];

// ════════════════════════════════════════════════
//  HELPERS
// ════════════════════════════════════════════════
void enviar_magic() {
  Serial.write(MAGIC, 4);
}

void modoRX() {
  radio.openReadingPipe(1, PIPE_TX);
  radio.startListening();
}

void modoTX() {
  radio.stopListening();
  radio.openWritingPipe(PIPE_RX);
}

void mandarAck(uint8_t cam_id, uint16_t frag_idx, uint8_t ok) {
  modoTX();
  AckPkt ack;
  ack.cam_id   = cam_id;
  ack.frag_idx = frag_idx;
  ack.ok       = ok;
  radio.write(&ack, sizeof(ack));
  modoRX();
}

// ════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════
void setup() {
  pinMode(LED_RX, OUTPUT);
  digitalWrite(LED_RX, LOW);

  Serial.begin(115200);
  delay(2000);
  Serial.println("{\"status\":\"RECEIVER_READY\"}");

  SPI.begin(18, 19, 23);

  if (!radio.begin()) {
    Serial.println("{\"error\":\"NRF24 not detected\"}");
    while (1);
  }

  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  modoRX();

  Serial.println("{\"status\":\"LISTENING\"}");
}

// ════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════
void loop() {
  if (!radio.available()) return;

  digitalWrite(LED_RX, HIGH);
  radio.read(buf, 32);
  uint8_t tipo = buf[0];

  // ── Telemetría → JSON ───────────────────────
  if (tipo == PKT_TELEMETRY) {
    Telemetry* d = (Telemetry*)buf;
    Serial.print("{\"temp\":");      Serial.print(d->temp, 2);
    Serial.print(",\"pressure\":"); Serial.print(d->pressure, 2);
    Serial.print(",\"altitude\":"); Serial.print(d->altitude, 2);
    Serial.print(",\"yaw\":");      Serial.print(d->yaw, 2);
    Serial.print(",\"pitch\":");    Serial.print(d->pitch, 2);
    Serial.print(",\"roll\":");     Serial.print(d->roll, 2);
    Serial.print(",\"calib\":{\"sys\":"); Serial.print(d->sys);
    Serial.print(",\"gyro\":");     Serial.print(d->gyro);
    Serial.print(",\"accel\":");    Serial.print(d->accel);
    Serial.print(",\"mag\":");      Serial.print(d->mag);
    Serial.println("}}");
  }

  // ── Inicio foto → binario con magic ─────────
  else if (tipo == PKT_FOTO_INICIO) {
    InicioPkt* p = (InicioPkt*)buf;
    enviar_magic();
    Serial.write(PKT_FOTO_INICIO);
    Serial.write(p->cam_id);
    Serial.write((uint8_t*)&p->size_total, 4);
    Serial.write((uint8_t*)&p->n_frags, 2);
    Serial.flush();
  }

  // ── Fragmento → ACK + binario con magic ─────
  else if (tipo == PKT_FOTO_FRAG) {
    FragPkt* p = (FragPkt*)buf;

    // Mandar ACK inmediatamente al C3
    mandarAck(p->cam_id, p->frag_idx, 1);

    // Reenviar fragmento al dashboard con magic header
    enviar_magic();
    Serial.write(PKT_FOTO_FRAG);
    Serial.write(p->cam_id);
    Serial.write((uint8_t*)&p->frag_idx, 2);
    Serial.write((uint8_t*)&p->frag_total, 2);
    Serial.write(p->datos, FRAG_SIZE);
    Serial.flush();
  }

  // ── Fin foto → binario con magic ────────────
  else if (tipo == PKT_FOTO_FIN) {
    FinPkt* p = (FinPkt*)buf;
    enviar_magic();
    Serial.write(PKT_FOTO_FIN);
    Serial.write(p->cam_id);
    Serial.flush();
  }

  delay(2);
  digitalWrite(LED_RX, LOW);
}
