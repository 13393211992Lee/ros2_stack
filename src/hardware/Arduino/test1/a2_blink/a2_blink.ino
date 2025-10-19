/*
 * 外置 led
 */
void setup() {
  // put your setup code here, to run once:
  pinMode(A0,OUTPUT);
  pinMode(A5,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(A0,HIGH);
//  delay(1000);
  digitalWrite(A5,LOW);
//  delay(1000);
}
