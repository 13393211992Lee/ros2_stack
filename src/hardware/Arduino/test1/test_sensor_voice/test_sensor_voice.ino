const int fire_sensor = 2;  

void setup() {
  // 设置信号引脚为输出模式
  pinMode(fire_sensor, INPUT);
  Serial.begin(9600);
}

void loop() {
  int val_buzzer = digitalRead(fire_sensor);
  if(val_buzzer == LOW){
    Serial.println("发现火源");  
  }else{
    Serial.println("没有发现火源");  
  }
}
