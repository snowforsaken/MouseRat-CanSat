// Código del transmisor

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// =====================================
// LED INTEGRADO
// =====================================
#define LED_TX 8

// =====================================
// I2C
// =====================================
#define SDA_PIN 7
#define SCL_PIN 10

// =====================================
// NRF24
// =====================================
#define NRF_CE   1
#define NRF_CSN  3

RF24 radio(NRF_CE, NRF_CSN);

const byte address[6] = "00001";

// =====================================
// BMP390
// =====================================
Adafruit_BMP3XX bmp;

#define SEALEVELPRESSURE_HPA 1013.25

// =====================================
// BNO055
// =====================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// =====================================
// STRUCT TELEMETRY
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
  pinMode(LED_TX, OUTPUT);
  digitalWrite(LED_TX, LOW);

  // =====================================
  // SERIAL
  // =====================================
  Serial.begin(115200);

  delay(2000);

  Serial.println("SYSTEM BOOT");

  // =====================================
  // I2C
  // =====================================
  Wire.begin(SDA_PIN, SCL_PIN);

  // =====================================
  // BMP390 INIT
  // =====================================
  if (!bmp.begin_I2C(0x77)) {

    Serial.println("{\"error\":\"BMP390 not detected\"}");

    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);

  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);

  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  // =====================================
  // BNO055 INIT
  // =====================================
  if (!bno.begin()) {

    Serial.println("{\"error\":\"BNO055 not detected\"}");

    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  // =====================================
  // NRF24 INIT
  // =====================================
  if (!radio.begin()) {

    Serial.println("{\"error\":\"NRF24 failed\"}");

    while (1);
  }

  radio.setChannel(108);

  radio.setDataRate(RF24_250KBPS);

  radio.setPALevel(RF24_PA_MAX);

  radio.openWritingPipe(address);

  radio.stopListening();

  Serial.println("{\"status\":\"SYSTEM_READY\"}");
}

void loop() {

  // =====================================
  // BMP390 READ
  // =====================================
  if (!bmp.performReading()) {

    Serial.println("{\"error\":\"BMP390 read failed\"}");

    delay(100);

    return;
  }

  // =====================================
  // BNO055 READ
  // =====================================
  sensors_event_t orientationData;

  bno.getEvent(
    &orientationData,
    Adafruit_BNO055::VECTOR_EULER
  );

  // =====================================
  // CALIBRATION
  // =====================================
  bno.getCalibration(
    &data.sys,
    &data.gyro,
    &data.accel,
    &data.mag
  );

  // =====================================
  // SAVE DATA
  // =====================================
  data.temp = bmp.temperature;

  data.pressure = bmp.pressure / 100.0;

  data.altitude =
    bmp.readAltitude(SEALEVELPRESSURE_HPA);

  data.yaw = orientationData.orientation.x;

  data.pitch = orientationData.orientation.y;

  data.roll = orientationData.orientation.z;

  // =====================================
  // NRF TRANSMIT
  // =====================================
  digitalWrite(LED_TX, HIGH);

  bool ok = radio.write(&data, sizeof(data));

  digitalWrite(LED_TX, LOW);

  // =====================================
  // JSON SERIAL OUTPUT
  // =====================================
  Serial.print("{");

  Serial.print("\"tx\":\"");
  Serial.print(ok ? "OK" : "FAIL");
  Serial.print("\",");

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

  delay(200);
}