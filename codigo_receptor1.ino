// Código del receptor

#include <SPI.h>
#include <RF24.h>

// =====================================
// NRF24 PINS ESP32 DEV MODULE
// =====================================
#define NRF_CE   4
#define NRF_CSN  5

RF24 radio(NRF_CE, NRF_CSN);

const byte address[6] = "00001";

// =====================================
// LED RX
// =====================================
#define LED_RX 2

// =====================================
// TELEMETRY STRUCT
// =====================================
struct Telemetry {

  float temp;
  float pressure;
  float altitude;

  float yaw;
  float pitch;
  float roll;

  uint8_t sys;
  uint8_t gyro;
  uint8_t accel;
  uint8_t mag;
};

Telemetry data;

void setup() {

  // =====================================
  // LED
  // =====================================
  pinMode(LED_RX, OUTPUT);

  digitalWrite(LED_RX, LOW);

  // =====================================
  // SERIAL
  // =====================================
  Serial.begin(115200);

  delay(2000);

  Serial.println("NRF24 RECEIVER START");

  // =====================================
  // SPI START
  // =====================================
  SPI.begin(18, 19, 23);

  // =====================================
  // NRF24 INIT
  // =====================================
  if (!radio.begin()) {

    Serial.println("{\"error\":\"NRF24 not detected\"}");

    while (1);
  }

  radio.setChannel(108);

  radio.setDataRate(RF24_250KBPS);

  radio.setPALevel(RF24_PA_MAX);

  radio.openReadingPipe(0, address);

  radio.startListening();

  Serial.println("{\"status\":\"RECEIVER_READY\"}");
}

void loop() {

  // =====================================
  // CHECK DATA
  // =====================================
  if (radio.available()) {

    // =========================
    // LED ON
    // =========================
    digitalWrite(LED_RX, HIGH);

    // =========================
    // READ DATA
    // =========================
    radio.read(&data, sizeof(data));

    // =========================
    // JSON OUTPUT
    // =========================
    Serial.print("{");

    Serial.print("\"temp\":");
    Serial.print(data.temp, 2);
    Serial.print(",");

    Serial.print("\"pressure\":");
    Serial.print(data.pressure, 2);
    Serial.print(",");

    Serial.print("\"altitude\":");
    Serial.print(data.altitude, 2);
    Serial.print(",");

    Serial.print("\"yaw\":");
    Serial.print(data.yaw, 2);
    Serial.print(",");

    Serial.print("\"pitch\":");
    Serial.print(data.pitch, 2);
    Serial.print(",");

    Serial.print("\"roll\":");
    Serial.print(data.roll, 2);
    Serial.print(",");

    Serial.print("\"calib\":{");

    Serial.print("\"sys\":");
    Serial.print(data.sys);
    Serial.print(",");

    Serial.print("\"gyro\":");
    Serial.print(data.gyro);
    Serial.print(",");

    Serial.print("\"accel\":");
    Serial.print(data.accel);
    Serial.print(",");

    Serial.print("\"mag\":");
    Serial.print(data.mag);

    Serial.print("}");

    Serial.println("}");

    // =========================
    // LED OFF
    // =========================
    delay(20);

    digitalWrite(LED_RX, LOW);
  }
}