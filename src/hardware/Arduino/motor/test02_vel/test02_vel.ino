
/*
  实现脉冲计数set
  1. 封装引脚 变量和 计数
  2. setup 引脚和操作模式input， 计数器结果输出到上位机（设置波特率）
  3.还需要为引脚添加中断事件（attachInterrupt）,跳变的高电压 和低电压
  4.计数逻辑实现
  5. 输出到上位机
 */
 


int encoder_A = 2; //0
int encoder_B = 3; //1
volatile int count = 0; //计数器 的准确性 用volatile修饰

void count_fun_a(){
  //单倍频
  //上升沿
  if(digitalRead(encoder_A) == HIGH){
    if(digitalRead(encoder_B) == HIGH){
      count++;
    }else{
      count--;
    }
  }

  //双倍频 下降沿+ 上升沿
  //下降沿
  else if(digitalRead(encoder_A) == LOW){
    if(digitalRead(encoder_B) == LOW){
      count++;
    }else{
      count--;
    }
  }
}

void count_fun_b(){
  //上升沿
  if(digitalRead(encoder_B) == HIGH){
    if(digitalRead(encoder_A) == LOW){
      count++;
    }else{
      count--;
    }
  }
  //下降沿
  else if(digitalRead(encoder_B) == LOW){
    if(digitalRead(encoder_A) == HIGH){
      count++;
    }else{
      count--;
    }
  }
}


void setup() {
  Serial.begin(57600);
  pinMode(encoder_A,INPUT);
  pinMode(encoder_B,INPUT);
  // 中断函数 
  //单倍频 或 双倍频 只需要为encoder_A 添加中断
  attachInterrupt(digitalPinToInterrupt(encoder_A),count_fun_a,CHANGE);
  //4倍频为encoder_B 添加中断
  attachInterrupt(digitalPinToInterrupt(encoder_B),count_fun_b,CHANGE);
  
}

// 1.封装变量
long start_time = millis();   
int interval_time=50;  
int motor_i = 30;
int motor_ppr = 11;
int freqMult = 4;
int motor_per_round = freqMult*motor_i*motor_ppr;

// 2.逻辑
//  获取当前时间
//  if（当前时间 - 开始时间 > 单位时间）{
//    取消中断
//    计算转速
//    COUNT 归0
//    开始时间 重置为 当前时间
//    重启中断
//  }
void get_current_vel(){
  long right_now = millis();
  long  pass_time = right_now - start_time;   //ms
  if(pass_time >= interval_time){
    noInterrupts();
    double vel = (double)count / motor_per_round / pass_time *1000*60;
    Serial.println(vel);
    count = 0;
    start_time = right_now;
    interrupts();
  }
}
void loop() {

  get_current_vel();
}
