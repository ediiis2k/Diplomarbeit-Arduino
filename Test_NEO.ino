// Testsketch NEO M8N
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

HardwareSerial GPSSerial(2);    
TinyGPSPlus gps;

static const int RX_PIN   = 16;  
static const int TX_PIN   = 17;   
static const long GPS_BAUD = 9600;

unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 1000; 

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[GPS] TinyGPS++ Start");
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.printf("[GPS] UART2 gestartet (%ld Baud, RX=%d, TX=%d)\n", GPS_BAUD, RX_PIN, TX_PIN);
  Serial.println("[GPS] Warte auf NMEA-Daten / Fix ...");
}

void loop() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }
  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();
    printGpsInfo();
  }
}

void printGpsInfo() {
  Serial.println("----------------------------------------------------");
  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    Serial.printf("Fix: JA   Lat: %.6f   Lon: %.6f\n", lat, lon);
    if (gps.date.isValid() && gps.time.isValid()) {
      Serial.printf("Zeit (UTC): %02d:%02d:%02d   Datum: %02d.%02d.%04d\n",
                    gps.time.hour(), gps.time.minute(), gps.time.second(),
                    gps.date.day(), gps.date.month(), gps.date.year());
    } else {
      Serial.println("Zeit/Datum: nicht gueltig");
    }
    if (gps.satellites.isValid()) Serial.printf("Satelliten: %d\n", gps.satellites.value());
    else                          Serial.println("Satelliten: n/v");
    if (gps.hdop.isValid())       Serial.printf("HDOP: %.1f\n", gps.hdop.hdop());
    else                          Serial.println("HDOP: n/v");
    if (gps.speed.isValid())      Serial.printf("Speed: %.2f km/h\n", gps.speed.kmph());
    else                          Serial.println("Speed: n/v");
    if (gps.altitude.isValid())   Serial.printf("Hoehe: %.1f m\n", gps.altitude.meters());
    else                          Serial.println("Hoehe: n/v");
    Serial.printf("Google Maps Link: https://maps.google.com/?q=%.6f,%.6f\n", lat, lon);
  } else {
    Serial.println("Fix: NEIN ");
    if (gps.satellites.isValid()) Serial.printf("Satelliten (sichtbar): %d\n", gps.satellites.value());
    if (gps.hdop.isValid())       Serial.printf("HDOP: %.1f\n", gps.hdop.hdop());
  }
}
