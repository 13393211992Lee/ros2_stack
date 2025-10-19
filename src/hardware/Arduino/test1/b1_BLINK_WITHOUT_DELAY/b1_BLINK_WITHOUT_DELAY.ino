
const int pin_led = LED_BUILTIN;
int led_statue = LOW;
unsigned long previous_millis = 0;
const long interval = 1000;

void setup() {
  // put your setup code here, to run once:
  pinMode(pin_led,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long current_millis = millis();
  if(current_millis - previous_millis >= interval){
    previous_millis = current_millis;
    if(led_statue == LOW){
      led_statue = HIGH;
    }else{
      led_statue = LOW;
    }
  }
  
  digitalWrite(pin_led,led_statue);
}
