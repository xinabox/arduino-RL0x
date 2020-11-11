/*************************************************************
  This is an examples for the RL01-02-03 Radio Range

  You can buy one on our store!
  -----> https://xinabox.cc/products/RL01/
  -----> https://xinabox.cc/products/RL02/
  -----> https://xinabox.cc/products/RL03/

  This example requests the Alcohol sensor to measure
  the Breath Alcohol Level

  Currently Supported on the following ☒CHIPs:
  - CW01
  - CR01/02/03

  The sensor communicates over the I2C Bus.

*************************************************************/

#include <xCore.h>
#include <xRL0x.h>

#define RL03_FREQ 915.0

// Radio Address
#define ADDR_ID 0x11
#define FROM_ID 0x11
#define TO_ID   0x31

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);

  // Set the RGB Pin directions
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Start the I2C Comunication
  Wire.begin();

  if (!RL0X.begin()) { // <-- enter radio name here
    Serial.println("Check the connector to RL03");
    while (1) {
      // Flash RED to indicate failure
      digitalWrite(LED_RED, HIGH);
      delay(100);
      digitalWrite(LED_RED, LOW);
      delay(100);
    }
  } else {
    // CR01 Initialized correctly
    //RL01.setModemConfig(RL01.Bw31_25Cr48Sf512);
    RL0X.setFrequency(RL03_FREQ);
    RL0X.setTxPower(23, false);
    RL0X.setThisAddress(ADDR_ID);
    RL0X.setHeaderFrom(FROM_ID);
    RL0X.setHeaderTo(TO_ID);
  }
  Serial.println("setup done");
}

void loop() {
  digitalWrite(LED_BUILTIN,HIGH);
  if (RL0X.waitAvailableTimeout(3000)) {
    uint8_t buf[195];
    uint8_t len = sizeof(buf);
    if (RL0X.recv(buf, &len)) {
      digitalWrite(LED_RED, HIGH);
      Serial.println("got message: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(RL0X.lastRssi(), DEC);

      // Send a reply
      uint8_t data[] = "And hello back to you";
      delay(100);
      RL0X.send(data, sizeof(data));
      Serial.println("Sent a reply");
      digitalWrite(LED_RED, LOW);
    } else {
      Serial.println("recv failed");
    }
  }
  digitalWrite(LED_BUILTIN,LOW);
}
