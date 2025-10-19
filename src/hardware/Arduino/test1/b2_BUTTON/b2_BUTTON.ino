
const int pin_led = 13;
const int pin_button = 2;
int button_state;
void setup() {
  pinMode(pin_led,OUTPUT);
  pinMode(pin_button,INPUT);
}

void loop() {
  button_state = digitalRead(pin_button);
  if(button_state == HIGH){
    digitalWrite(pin_led,HIGH);
  }else{
    digitalWrite(pin_led,LOW); 
  }
}
