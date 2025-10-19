/*
 * 
 * millis()
 * 休眠1000ms 执行)
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600一次);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  unsigned long t_time = millis();
  Serial.print("time = ");
  Serial.println(t_time);
}
