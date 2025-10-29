// Testsketch Buzzer 2, Sirene
#define BUZZ 25

void setup() { pinMode(BUZZ, OUTPUT); }

void loop() {
  for (int f=800; f<=2500; f+=20) { tone(BUZZ, f); delay(5); }
  for (int f=2500; f>=800; f-=20) { tone(BUZZ, f); delay(5); }
  noTone(BUZZ);
  delay(300);
}
