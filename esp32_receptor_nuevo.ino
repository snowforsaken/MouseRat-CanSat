// ════════════════════════════════════════════════
//  ESP32 Dev Module — Receptor en tierra
//  Recibe telemetría e imagen por nRF24
//  Manda ACK por cada fragmento
//  Reenvía todo por Serial al dashboard Python
//
//  BUGS CORREGIDOS:
//  1. Cast directo de buf a struct eliminado — memcpy seguro
//  2. Pipes correctamente invertidos respecto al C3
//  3. enableDynamicPayloads() para coincidir con C3
//  4. Validación de tipo antes de procesar paquete
//  5. Tamaño de payload verificado
// ════════════════════════════════════════════════
#include <SPI.h>
#include <RF24.h>

#define NRF_CE   4
#define NRF_CSN  5
#define LED_RX   2

RF24 radio(NRF_CE, NRF_CSN);

// Pipes INVERTIDOS al C3: C3 TX → receptor RX, receptor TX → C3 RX
const byte PIPE_RX_ADDR[6] = "CSAT1";  // escuchamos donde C3 transmite
const byte PIPE_TX_ADDR[6] = "CSAT2";  // transmitimos donde C3 escucha

#define PKT_TELEMETRY   0x01
#define PKT_FOTO_INICIO 0x02
#define PKT_FOTO_FRAG   0x03
#define PKT_FOTO_FIN    0x04
#define PKT_ACK         0x05
#define FRAG_SIZE       24

const uint8_t MAGIC[4] = {0xDE, 0xAD, 0xBE, 0xEF};

// Helpers endian
inline uint16_t unpack16(const uint8_t* s) {
  return (uint16_t)s[0] | ((uint16_t)s[1] << 8);
}
inline uint32_t unpack32(const uint8_t* s) {
  return (uint32_t)s[0] | ((uint32_t)s[1]<<8) |
         ((uint32_t)s[2]<<16) | ((uint32_t)s[3]<<24);
}

uint8_t rxbuf[32];

void modoRX() {
  radio.stopListening();
  radio.openWritingPipe(PIPE_TX_ADDR);
  radio.openReadingPipe(1, PIPE_RX_ADDR);
  radio.startListening();
}

void modoTX() {
  radio.stopListening();
  radio.openWritingPipe(PIPE_TX_ADDR);
  radio.openReadingPipe(1, PIPE_RX_ADDR);
}

void mandarAck(uint8_t cam_id, uint16_t frag_idx, uint8_t ok) {
  modoTX();
  uint8_t ackBuf[32];
  memset(ackBuf, 0, 32);
  ackBuf[0] = PKT_ACK;
  ackBuf[1] = cam_id;
  ackBuf[2] = frag_idx & 0xFF;
  ackBuf[3] = (frag_idx >> 8) & 0xFF;
  ackBuf[4] = ok;
  radio.write(ackBuf, 32);
  modoRX();
}

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
  radio.setRetries(5, 15);       // FIX: coincidir con C3
  radio.enableDynamicPayloads(); // FIX: coincidir con C3
  modoRX();

  Serial.println("{\"status\":\"LISTENING\"}");
}

void loop() {
  if (!radio.available()) return;

  digitalWrite(LED_RX, HIGH);
  radio.read(rxbuf, 32);
  uint8_t tipo = rxbuf[0];

  // FIX: validar tipo antes de procesar
  if (tipo < PKT_TELEMETRY || tipo > PKT_ACK) {
    digitalWrite(LED_RX, LOW);
    return;
  }

  // ── Telemetría ──────────────────────────────
  if (tipo == PKT_TELEMETRY) {
    // FIX: extraer floats con memcpy, no cast directo
    float temp, pressure, altitude, yaw, pitch, roll;
    memcpy(&temp,     &rxbuf[1],  4);
    memcpy(&pressure, &rxbuf[5],  4);
    memcpy(&altitude, &rxbuf[9],  4);
    memcpy(&yaw,      &rxbuf[13], 4);
    memcpy(&pitch,    &rxbuf[17], 4);
    memcpy(&roll,     &rxbuf[21], 4);
    uint8_t sys   = rxbuf[25];
    uint8_t gyro  = rxbuf[26];
    uint8_t accel = rxbuf[27];
    uint8_t mag   = rxbuf[28];

    Serial.print("{\"temp\":");      Serial.print(temp, 2);
    Serial.print(",\"pressure\":"); Serial.print(pressure, 2);
    Serial.print(",\"altitude\":"); Serial.print(altitude, 2);
    Serial.print(",\"yaw\":");      Serial.print(yaw, 2);
    Serial.print(",\"pitch\":");    Serial.print(pitch, 2);
    Serial.print(",\"roll\":");     Serial.print(roll, 2);
    Serial.print(",\"calib\":{\"sys\":"); Serial.print(sys);
    Serial.print(",\"gyro\":"); Serial.print(gyro);
    Serial.print(",\"accel\":"); Serial.print(accel);
    Serial.print(",\"mag\":"); Serial.print(mag);
    Serial.println("}}");
  }

  // ── Inicio foto ──────────────────────────────
  else if (tipo == PKT_FOTO_INICIO) {
    uint8_t  cam_id    = rxbuf[1];
    uint32_t size_tot  = unpack32(&rxbuf[2]);
    uint16_t n_frags   = unpack16(&rxbuf[6]);

    // Validar valores razonables antes de reenviar
    if (size_tot == 0 || size_tot > 200000 || n_frags == 0 || n_frags > 10000) {
      digitalWrite(LED_RX, LOW);
      return;
    }

    Serial.write(MAGIC, 4);
    Serial.write(PKT_FOTO_INICIO);
    Serial.write(cam_id);
    Serial.write(&rxbuf[2], 4);  // size_total ya en little-endian
    Serial.write(&rxbuf[6], 2);  // n_frags ya en little-endian
    Serial.flush();
  }

  // ── Fragmento ────────────────────────────────
  else if (tipo == PKT_FOTO_FRAG) {
    uint8_t  cam_id   = rxbuf[1];
    uint16_t frag_idx = unpack16(&rxbuf[2]);
    uint16_t frag_tot = unpack16(&rxbuf[4]);

    // Validar
    if (frag_idx >= frag_tot || frag_tot == 0 || frag_tot > 10000) {
      digitalWrite(LED_RX, LOW);
      return;
    }

    // ACK inmediato al C3
    mandarAck(cam_id, frag_idx, 1);

    // Reenviar al dashboard
    Serial.write(MAGIC, 4);
    Serial.write(PKT_FOTO_FRAG);
    Serial.write(cam_id);
    Serial.write(&rxbuf[2], 2);  // frag_idx
    Serial.write(&rxbuf[4], 2);  // frag_total
    Serial.write(&rxbuf[6], FRAG_SIZE);
    Serial.flush();
  }

  // ── Fin foto ─────────────────────────────────
  else if (tipo == PKT_FOTO_FIN) {
    uint8_t cam_id = rxbuf[1];
    Serial.write(MAGIC, 4);
    Serial.write(PKT_FOTO_FIN);
    Serial.write(cam_id);
    Serial.flush();
  }

  delay(2);
  digitalWrite(LED_RX, LOW);
}
