#define _SS_MAX_RX_BUFF 128
#include <SoftwareSerial.h>

#define ftcUartRx 10
#define ftcUartTx 11

SoftwareSerial ftcUART(ftcUartRx, ftcUartTx); // RX, TX

void setup() {
  // USB baud for debug
  Serial.begin(9600);
  // Touchscreen UART baud
  ftcUART.begin(9600);
}

void loop() {
  // Packet: B0: Pressure?, B1: X coord, B2: Y coord, B3: 0, packet terminator
  static byte packet[4];
  static int idx = 0;
  
  if (ftcUART.available()) {
    byte c = ftcUART.read();
    packet[idx++] = c;
    
    if (idx == 4) {
      if (packet[3] == 0) {
        // Print all bytes
        Serial.print("B0: ");
        Serial.print(packet[0]);
        Serial.print("\tB1: ");
        Serial.print(packet[1]);
        Serial.print("\tB2: ");
        Serial.print(packet[2]);
        Serial.print("\tB3: ");
        Serial.println(packet[3]);
      }
      idx = 0;
    }
  }
}