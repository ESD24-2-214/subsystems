#define A1 11
#define A2 10
#define EN12 8
#define MAX 255

void setup() {
  pinMode (A1, OUTPUT);
  pinMode (A2, OUTPUT);
  pinMode (EN12, OUTPUT);

  digitalWrite(EN12, HIGH);
}

void loop() {
  analogWrite(A1, MAX / 2);
  delay(1000);
  analogWrite(A1, 0);
  analogWrite(A2, MAX / 2);
  delay(1000);
  analogWrite(A2, 0);
}
