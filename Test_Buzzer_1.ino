// Testsketch Buzzer 1, Periodischer Ton
#define BUZZ 25

void setup() { pinMode(BUZZ, OUTPUT); }

void loop() {
  tone(BUZZ, 2000);  
  delay(1000);
  noTone(BUZZ);      
  delay(1000);
}