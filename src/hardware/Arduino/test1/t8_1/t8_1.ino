int ledpin = 9;
int brightness = 0;
int direction = 1; 
void setup() {
  pinMode(ledpin , OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  brightness+=direction;
  // 当亮度达到255时，切换方向为递减
  if (brightness == 255) {
    direction = -1;
  }
  // 当亮度回到0时，切换方向为递增
  else if (brightness == 0) {
    direction = 1;
  }
  else if(brightness )
  analogWrite(ledpin, brightness);
  Serial.println(brightness);
}
