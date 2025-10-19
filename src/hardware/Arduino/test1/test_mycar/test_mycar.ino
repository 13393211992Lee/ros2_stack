void setup() {
  // put your setup code here, to run once:
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);  // ENA
  
}

void loop() {
  digitalWrite(7,HIGH);
  digitalWrite(8,LOW);
  digitalWrite(9,255);
}
