#include "Adafruit_BMP3XX.h"
#include "Adafruit_MMA8451.h"
#include <RFM69.h>
#include <RFM69_ATC.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>
// -----[ SD Config ]-----

#define SCK 14  // Stolen from TSAT-0 might be different now
#define MISO 12
#define MOSI 13
#define SD_CS 15
// -----[ Network Config ]-----
#define NODEID 0
#define NETWORKID 100
#define FREQUENCY RF69_915MHZ
#define ENCRYPTKEY "TSAT-2B/25"

// Auto Transmission Control
// Saves power
#define ENABLE_ATC

// -----[ Sensor Config ]-----

#define SEALEVELPRESSURE_HPA (1013.25)

// -----[ Constants ]-----

#define TEST_MODE

#define PACKET_PING 1
#define PACKET_TELEMETRY 2

#define SATELLITE_ID 1


File flightLog;
bool cardFail; //if sd card doesn't activate works like tsat0
char file_name[20];
int flightNum;
//move these anywhere if they don't belong here
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
RFM69_ATC radio;
#else
RFM69 radio;
#endif
Adafruit_BMP3XX bmp;
Adafruit_MMA8451 mma = Adafruit_MMA8451();

unsigned int datapoint_count = 0;
DataPoint previous_datapoint;

// -----[ Networking ]-----
// Packets are sent as raw bytes.
// While this makes things a little more complicated on the receiving end,
// It means we need basically no code for serialization/deserialization
// It also makes it very lightweight (not that it matters)

// Telemetry Packet
typedef struct {
  uint32_t index;
  uint32_t time;
  // These values are fixed-point with 3 decimals.
  // Ideally they would be floating point, but floating point
  // representation is very weird (undefined?) across platorms.
  // This might not be entirely necessary, but it's good to make sure.
  // Telemetry doesn't need to be 100% accurate anyways.
  uint32_t altitude;
  uint32_t pressure;
  uint32_t temperature;
  uint32_t acceleration_magnitude;
  uint32_t velocity;
} PacketTelemetry;

// Ping packet
typedef struct {
  // Counter that we sent back to identify which ping it is
  uint32_t counter;
} PacketPing;

// Metadata for each packet
typedef struct {
  uint16_t satellite_id;
  uint16_t message_type;
} PacketMeta;

typedef struct {
  PacketMeta meta;
  union {
    PacketTelemetry telemetry;
    PacketPing ping;
  } data;
} Packet;

static TaskHandle_t communication_rx_handle;
static TaskHandle_t communication_tx_handle;

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
  if (radio.DATALEN < sizeof(PacketMeta)) {
    return false;
  }

  // Read in packet header
  memcpy(packet, radio.DATA, sizeof(PacketMeta));

  // Depending on which packet type we received, we
  // have to read a different amount of bytes.
  int size;
  switch (packet->meta.message_type) {
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
  if (radio.DATALEN - sizeof(PacketMeta) < size) {
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
      previous ? (data->altitude - previous->altitude) / (data->time - previous->time) : 0;

  Packet packet;
  packet.meta = {SATELLITE_ID, PACKET_TELEMETRY};
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

void datapoint_to_csv(File file, Datapoint *dp){

  if(file == NULL){
    Serial.println("File not found!");
    return;
  }
  if(dp == NULL){
    Serial.println("Null Datapoint");
    return;
  }

  file.print(dp->index);
  file.print(",");
  file.print(dp->time);
  file.print(",");
  file.print(dp->temperature);
  file.print(",");
  file.print(dp->pressure);
  file.print(",");
  file.print(dp->altitude);
  file.print(",");
  file.print(dp->accel[0]); //accel x
  file.print(",");
  file.print(dp->accel[1]); // accel y
  file.print(",");
  file.println(dp->accel[2]); //accel z
}
// Handles receiving a ping packet.
void handle_ping_packet(Packet *packet, uint16_t sender) {
  // Send back the same packet that received.
  radio.send(sender, packet, sizeof(PacketMeta) + sizeof(PacketPing));
}

// The main loop for receiving packets.
void communication_rx(void *_) {
  while (1) {
    Packet packet;
    bool success = receive_packet(&packet);

    switch (packet.meta.message_type) {
      case PACKET_PING:
        handle_ping_packet(&packet, radio.SENDERID);
        break;
        // We never should receive a telemetry packet.
        // If we do then just ignore it.
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void communication_tx(void *_) {
  bool previous = false;
  while (1) {
    DataPoint dp;
    bool success = capture_data(&dp);
    if (success) {
      Packet packet = datapoint_to_telemetry(&dp, previous ? &previous_datapoint : NULL);
      previous = true;
      previous_datapoint = dp;
      // Node 255 broadcasts to every node on network
      radio.send(255, &packet, sizeof(PacketMeta) + sizeof(PacketTelemetry));
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// -----[ Sensor Data ]-----

bool capture_data(DataPoint *dp) {
  if (!bmp.performReading()) {
    return false;
  }
  mma.read();

  dp->index = datapoint_count++;
  dp->time = millis();
  dp->temperature = bmp.temperature;
  dp->pressure = bmp.pressure;
  dp->altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  dp->accel[0] = mma.x;
  dp->accel[1] = mma.y;
  dp->accel[2] = mma.z;

  return true;
}

// -----[ Initialization ]-----

void setup() {

  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower(); // needed for RFM69HCW
  radio.encrypt(ENCRYPTKEY);

  #ifdef TEST_MODE
    Serial.begin(115200);
    return;
  #endif

  if (!bmp.begin_I2C()) {
    while (1)
      ;
  };

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (!mma.begin()) {
    while (1)
      ;
  };
  mma.setRange(MMA8451_RANGE_2_G);

  xTaskCreatePinnedToCore(communication_rx, "communication_rx", 3000, NULL, 10,
                          &communication_rx_handle, 0);

  xTaskCreatePinnedToCore(communication_tx, "communication_tx", 3000, NULL, 5,
                          &communication_tx_handle, 0);
  // SD v

  cardfail = false;

  if(!SD.begin()) {
    Serial.println("SD Card Initializatoin Failed");
    cardFail = true;
  }else if(SD.cardType() == CARD_NONE);{
    Serial.println("Please insert SD Card");
    cardFail = true;
  }

  if(!cardFail){ // increments the name by 1 every flight
    flightNum = 0;
    do {
      sprintf(file_name, "/tsatlog%d.csv", flightNum);
      i++;
    } while (SD.exists(file_name));
    
    flightLog = SD.open(file_name, FILE_WRITE);

    if(flightLog){
      flightLog.println("Index,Time (ms),Temperature (C),Pressure (pa),Altitude (m),Accel x (m/s^2),Accel y (m/s^2),Accel z (m/s^2)"); //prints headers for the file
      //prints the headers to the file
    }else{
      Serial.println("File falied to open");
    }
  }

}

unsigned int counter = 0;
bool previous = false;
void loop() {
  #ifdef TEST_MODE
    if (radio.receiveDone()) {
      for (byte i = 0; i < radio.DATALEN; i++)
        Serial.print(radio.DATA[i]);
      Serial.println();

      if (radio.ACKRequested())
      {
        radio.sendACK();
        Serial.print(" - ACK sent");
      }
    }
    Serial.println("Creating fake datapoint,");
    DataPoint dp;
    dp.index = counter++;
    dp.time = millis();
    dp.pressure = sin(millis() / 1000.0);
    dp.altitude = dp.pressure * 5 + 3;
    dp.temperature = cos(millis() / 1000.0);
    for (int i=0; i<3; i++) {
      dp.accel[i] = random(-100, 100) / 100.0;
    }
    Serial.println("Creating telemetry packet.");
    Packet packet = datapoint_to_telemetry(&dp, previous ? &previous_datapoint : NULL);

    // Node 255 broadcasts to every node on network
    if (radio.canSend()) {
      Serial.println("Sending packet!");
      radio.send(255, &packet, sizeof(PacketMeta) + sizeof(PacketTelemetry));
    } else {
      Serial.println("Unable to send packet.");
    }

    previous_datapoint = dp;
    previous = true;

    Serial.println("Sleeping.");
    delay(1000);
  #endif
  // put your main code here, to run repeatedly:
}
