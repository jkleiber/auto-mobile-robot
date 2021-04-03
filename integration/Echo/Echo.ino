void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println("{\"roll\": 0}");
  if(Serial.available())
  {
    String data = Serial.readStringUntil('\n');
    Serial.println(data);
  }
  delay(100);
}
