#include "Adafruit_BMP3XX.h"
#include "Adafruit_MMA8451.h"
#include <RFM69.h>
#include <RFM69_ATC.h>

// -----[ Network Config ]-----
#define NODEID 0
#define NETWORKID 100
#define FREQUENCY RF69_915MHZ
#define ENCRYPTKEY "TSAT-2B/Bubbles-"

#define RFM69_CS 5
#define RFM69_RST 14
#define RFM69_IRQ 13
// Auto Transmission Control
// Saves power
#define ENABLE_ATC

// -----[ Constants ]-----

// #define TEST_MODE
// #define TEST_MODE_USE_MMA

#define RX_INTERVAL 200 // Packet receive interval
#define TX_INTERVAL 500 // Packet send interval

#define PACKET_PING 1
#define PACKET_TELEMETRY 2

#define SATELLITE_ID 1

// -----[ Misc ]-----

#define LED_BUILTIN 2

// -----[ Types ]-----

typedef struct {
  unsigned int index;
  unsigned long time; // milliseconds
  double temperature; // celsius
  double pressure;    // pa
  double altitude;    // m
  float accel[3];     // m/s/s
} DataPoint;

// -----[ Statics ]-----

#ifdef ENABLE_ATC
RFM69_ATC radio(RFM69_CS, RFM69_IRQ);
#else
RFM69 radio(RFM69_CS, RFM69_IRQ);
#endif
Adafruit_BMP3XX bmp;
Adafruit_MMA8451 mma = Adafruit_MMA8451();

unsigned int datapoint_count = 0;
DataPoint previous_datapoint;

// -----[ Networking ]-----
// Packets are sent as raw bytes.
// While this makes things a little more complicated on the receiving end,
// It means we need basically no code for serialization/deserialization on air
// It also makes it very lightweight (not that it matters too much though)

// Telemetry Packet
typedef struct {
  uint32_t index;
  uint32_t time;
  // These values are fixed-point with 3 decimals.
  // Ideally they would be floating point, but floating point
  // representation is very weird (undefined?) across platorms.
  // This might not be entirely necessary, but it's good to make sure.
  // Telemetry doesn't need to be 100% accurate anyways.
  int32_t altitude;
  uint32_t pressure;
  uint32_t temperature;
  uint32_t acceleration_magnitude;
  int32_t velocity;
} PacketTelemetry;

// Ping packet
typedef struct {
  // Counter that we sent back to identify which ping it is
  uint32_t counter;
} PacketPing;

// General data for each packet
typedef struct {
  uint16_t satellite_id;
  uint16_t message_type;
} PacketHeader;

typedef struct {
  PacketHeader header;
  union {
    PacketTelemetry telemetry;
    PacketPing ping;
  } data;
} Packet;

static TaskHandle_t communication_handle;

// Receives a packet from the communications module.
// Parameters:
//   - packet: A pointer to the struct of which
//       the packet is written to.
// Returns:
//    - whether receiving the packet was successful.
bool receive_packet(Packet *packet) {
  if (!radio.receiveDone()) {
    return false;
  }
  if (radio.DATALEN < sizeof(PacketHeader)) {
    return false;
  }

  // Read in packet header
  memcpy(packet, radio.DATA, sizeof(PacketHeader));

  // Depending on which packet type we received, we
  // have to read a different amount of bytes.
  int size;
  switch (packet->header.message_type) {
    case PACKET_PING:
      size = sizeof(PacketPing);
      break;
    case PACKET_TELEMETRY:
      size = sizeof(PacketTelemetry);
      break;
    default:
      // Unknown packet.
      return false;
  }

  // Check to make sure the rest of the data is there too
  if (radio.DATALEN - sizeof(PacketHeader) < size) {
    return false;
  }

  // Read in the actual packet data into the data portion.
  memcpy(&packet->data, radio.DATA, size);

  return true;
}

// Converts a datapoint into a telemetry packet.
// Parameters:
//   - data: A pointer to the datapoint we are converting.
//   - (optional) previous: A pointer to the previous datapoint.
// Returns:
//    - A telemetry packet which can be sent.
Packet datapoint_to_telemetry(DataPoint *data, DataPoint *previous) {
  // Magnitude of 3d vector (search it up)
  double accel_mag = sqrt(sq(data->accel[0]) + sq(data->accel[1]) + sq(data->accel[2]));

  // Velocity = dx / dt
  // If we don't have a previous datapoint, default to 0
  double velocity =
      previous ? 1000 * (data->altitude - previous->altitude) / (data->time - previous->time) : 0;

  Packet packet;
  packet.header = {SATELLITE_ID, PACKET_TELEMETRY};
  packet.data.telemetry = {
      data->index,
      data->time,
      // Floating-point to fixed-point 3 decimals. See above for rationale.
      data->altitude * 1000,
      data->pressure * 1000,
      data->temperature * 1000,
      accel_mag * 1000,
      velocity * 1000,
  };

  return packet;
}

// Handles receiving a ping packet.
void handle_ping_packet(Packet *packet, uint16_t sender) {
  // Send back the same packet that received.
  radio.send(sender, packet, sizeof(PacketHeader) + sizeof(PacketPing));
}

// Reads packets every RX_INTERVAL seconds
int last_rx = 0;
void handle_rx() {
  if (millis() - last_rx < RX_INTERVAL) {
    return;
  }
  last_rx = millis();

  Packet packet;
  bool success = receive_packet(&packet);

  switch (packet.header.message_type) {
    case PACKET_PING:
      handle_ping_packet(&packet, radio.SENDERID);
      break;
      // We never should receive a telemetry packet.
      // If we do then just ignore it.
  }
}

// Sends out packets every TX_INTERVAL seconds
int last_tx = 0;
bool previous = false;
void handle_tx() {
  if (millis() - last_tx < TX_INTERVAL) {
    return;
  }
  last_tx = millis();

  // TODO: should we use a queue so that other threads can send packets?
  // Maybe for TSAT-3 :)
  DataPoint dp;
  capture_data(&dp);

  Packet packet = datapoint_to_telemetry(&dp, previous ? &previous_datapoint : NULL);
  previous = true;
  previous_datapoint = dp;
  // Broadcast to every node
  radio.send(RF69_BROADCAST_ADDR, &packet, sizeof(PacketHeader) + sizeof(PacketTelemetry));
}

// The main loop for transmitting packets.
void communication_loop(void *_) {
  bool previous = false;
  while (1) {
    handle_rx();
    handle_tx();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// -----[ Sensor Data ]-----

bool mma_available = false;
bool bmp_available = false;
float base_pressure;
void capture_data(DataPoint *dp) {
  dp->index = datapoint_count++;
  dp->time = millis();
  if (bmp_available && bmp.performReading()) {
    dp->temperature = bmp.temperature;
    dp->pressure = bmp.pressure;
    dp->altitude = bmp.readAltitude(base_pressure);
  } else {
    dp->temperature = 0;
    dp->pressure = 0;
    dp->altitude = 0;
  }
  if (mma_available) {
    mma.read();
    sensors_event_t event;
    mma.getEvent(&event);

    dp->accel[0] = event.acceleration.x;
    dp->accel[1] = event.acceleration.y;
    dp->accel[2] = event.acceleration.z;
  } else {
    dp->accel[0] = 0;
    dp->accel[1] = 0;
    dp->accel[2] = 0;
  }
}

// -----[ Initialization ]-----

// Separated out for simplicity
void test_mode_init() {
  Serial.begin(115200);

  if (mma.begin()) {
    Serial.println("MMA Initialized!");
    mma_available = true;
    mma.setRange(MMA8451_RANGE_2_G);
  } else {
    Serial.println("MMA failed to init!");
  }

  if (bmp.begin_I2C()) {
    bmp_available = true;

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("BMP Initialized!");
  } else {
    Serial.println("BMP failed to init!");
  }
}

void setup() {
  // Reset the RFM69
  // Needed for it to initialize properly!
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  pinMode(LED_BUILTIN, OUTPUT);
  if (!radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
    // Blink LED to signal error
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }
  radio.setHighPower(); // needed for RFM69HCW
  radio.encrypt(ENCRYPTKEY);

#ifdef TEST_MODE
  test_mode_init();
  return;
#endif

  if (mma.begin()) {
    mma_available = true;

    mma.setRange(MMA8451_RANGE_4_G);
  };

  if (bmp.begin_I2C()) {
    bmp_available = true;

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Calibrate base pressure
    // Note: oversampling results in pressure readings being smoothed.
    // the first couple of readings will be inaccurate, so lets discard them
    float pressure_sum = 0;
    int reading_count = 0;
    for (int i = 0; i < 10; i++) {
      delay(200);
      if (bmp.performReading() && i >= 5) {
        pressure_sum += bmp.pressure;
        reading_count++;
      }
    }

    if (reading_count != 0) {
      base_pressure = pressure_sum / reading_count / 100;
    } else {
      // Calibration failed, use a default sea-level pressure.
      base_pressure = 1013.25;
    }
  };

  // Set builtin led to on to signify we're live.
  digitalWrite(LED_BUILTIN, HIGH);

  xTaskCreatePinnedToCore(communication_loop, "communication", 3000, NULL, 5, &communication_handle,
                          0);
}

#ifdef TEST_MODE
unsigned int counter = 0;
bool previous = false;
void loop() {
  if (radio.receiveDone()) {
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print(radio.DATA[i]);
    Serial.println();

    if (radio.ACKRequested()) {
      radio.sendACK();
      delay(10);
      Serial.println(" - ACK sent");
    }
  }
  Serial.println("Creating fake datapoint,");
  DataPoint dp;
  capture_data(&dp);
  Packet packet = datapoint_to_telemetry(&dp, previous ? &previous_datapoint : NULL);

  // Broadcast to every node
  radio.send(RF69_BROADCAST_ADDR, &packet, sizeof(PacketHeader) + sizeof(PacketTelemetry), false);

  previous_datapoint = dp;
  previous = true;

  Serial.println("Sleeping.");
  delay(250);
  // put your main code here, to run repeatedly:
}
#endif

#ifndef TEST_MODE
void loop() {}
#endif
