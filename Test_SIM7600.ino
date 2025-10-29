// Testsketch SIM7600
#include <HardwareSerial.h>

HardwareSerial LTE(1);

#define LTE_RX 26  
#define LTE_TX 27   
#define LTE_BAUD 115200

void setup() {
  Serial.begin(115200);
  LTE.begin(LTE_BAUD, SERIAL_8N1, LTE_RX, LTE_TX);
  delay(2000);
  Serial.println("Sende SMS...");
  LTE.println("AT");
  delay(500);
  LTE.println("AT+CMGF=1"); 
  delay(500);
  LTE.println("AT+CMGS=\"+41XXXXXXXXX\"");  
  delay(500);
  LTE.print("SIM7600 SMS Test via ESP32");    
  delay(500);
  LTE.write(26);  
  delay(500);
  Serial.println("SMS gesendet (wenn Netz ok).");
}

void loop() {
  while (LTE.available()) {
    Serial.write(LTE.read());  
  }
}
