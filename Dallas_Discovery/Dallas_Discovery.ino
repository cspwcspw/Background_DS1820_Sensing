
#include "SensorDiscovery.h"


void setup(void) {

  byte buf[8];
  Serial.begin(115200);
  Serial.println("Sensor Discovery Example"); 

  SensorDiscovery sd;

  byte response;
  sd.begin(buf);
  while (true) {
    response = sd.findNextDevice();
    if (response == 0) {
      printBytes(buf, 8);
    }
    else break;
  }

  Serial.print("No more devices on the bus, response code is "); Serial.println(response);
}

void loop(void) {
}

// function to print bytes in hex
void printBytes(byte *b, int n) {
  char buf[20];
  for (byte i = 0; i < n; i++) {
    sprintf(buf, "%02x ", b[i]);
    Serial.print(buf);
  }
  Serial.println();
}
