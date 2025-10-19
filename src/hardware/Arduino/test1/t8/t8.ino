bool push_button_up;
bool push_button_down;
int ledpin = 9;
int brightness = 128;
void setup() {
  // put your setup code here, to run once:
  pinMode(2 , INPUT_PULLUP);
  pinMode(8 , INPUT_PULLUP);
  pinMode(ledpin , OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  push_button_up = digitalRead(2);
  push_button_down = digitalRead(8);
//  Serial.println(push_button_down);
  if(!push_button_down && brightness > 0){
    brightness--;
  }else if(!push_button_up && brightness <= 255){
    brightness++;
  }

  analogWrite(ledpin, brightness);
  Serial.println(brightness);
}
