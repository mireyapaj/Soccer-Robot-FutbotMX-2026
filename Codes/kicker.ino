const int MOSFET_PIN = 6;   // Gate del IRLZ44N

void setup() {
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);

  Serial.begin(9600);
}

void loop() {

  if (Serial.available()) {

    char dato = Serial.read();

    if (dato == '2') {

      // Disparo del kicker
      digitalWrite(MOSFET_PIN, HIGH);
      delay(50);  // Pulso de 50 ms
      digitalWrite(MOSFET_PIN, LOW);

      Serial.println("KICK");
    }
  }
}