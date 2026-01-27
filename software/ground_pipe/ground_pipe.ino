#include <RFM69.h>
#include <RFM69_ATC.h>

// -----[ Network Config ]-----

#define NODEID 1
#define NETWORKID 100
#define FREQUENCY RF69_915MHZ
#define ENCRYPTKEY "TSAT-2B/Bubbles-"

#define RFM69_CS 5
#define RFM69_RST 14
#define RFM69_IRQ 13
#define LED_BUILTIN 2

// Auto Transmission Control
// Saves power
#define ENABLE_ATC

// -----[ Statics ]-----

#ifdef ENABLE_ATC
RFM69_ATC radio(RFM69_CS, RFM69_IRQ);
#else
RFM69 radio(RFM69_CS, RFM69_IRQ);
#endif

void setup() {
  Serial.println("Initializing!");
  // Reset the RFM69
  // Needed for it to initialize properly!
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  pinMode(LED_BUILTIN, OUTPUT); // Blinks on packet recieve
  Serial.begin(115200);

  if (radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
    Serial.println("RFM69 Initialized Successfully.");
  } else {
    Serial.println("RFM69 Failed to initialize.");
  }

  radio.setHighPower(); // needed for RFM69HCW
  radio.encrypt(ENCRYPTKEY);
}

void loop() {
  if (radio.receiveDone()) {
    // Print out the packet
    for (int i = 0; i < radio.DATALEN; i++) {
      Serial.printf("%d ", radio.DATA[i]);
    }
    Serial.println();
    if (radio.ACKRequested()) {
      radio.sendACK();
      delay(10);
      Serial.println("ACK Sent.");
    }
    digitalWrite(LED_BUILTIN, HIGH);
    delay(3);
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  delay(100);
}
