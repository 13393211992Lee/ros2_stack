int on_time = 1000;
int off_time = 1000;

// the setup function runs once when you press reset or power the board
// setup 函数执行
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
// loop 函数会反复
void loop() {
  off_time = off_time + 1000; // 每次循环off_time 增加一秒一名
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(on_time);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(off_time);                       // wait for a second
}
