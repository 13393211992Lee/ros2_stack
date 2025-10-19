
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

void loop() {
  delay(2000);
  Serial.println(count);
}

//如何计算脉冲数：
//脉冲数 = n r * 减速比 * 1r产生的脉冲数 * 频率
//n r --> 转了几圈
//减速比
//1r产生的脉冲数
//频率 --> 单倍频1 双倍频2 四倍频4
//
//本机 JGB37-520 12V 330转
//减速比 1：30， 物理结构决定，不会因为电压改变而产生减速比变化
//PPR（每转脉冲数）： 11



//如何理解中断引脚：
//eg:encoder_A（接到 D2）：
//  物理线接在 “数字引脚 2” 
//  芯片（ATmega328P）内部，这根脚对应的外部中断名字叫 INT0，它的编号是 0。
//  
//获取 数字引脚的 中断引脚：digitalPinToInterrupt(2)


//attachInterrupt 的使用
//attachInterrupt(interrupt, function, mode)
//interrupt 中断引脚
//function: 中断触发时调用的函数
//  中断触发时，delay() he millis（）不会发生变化，串口数据会丢失，
//  因此应该声明一个变量 为 未来发生中断时存储变量
//mode:
//  LOW
//  CHANGE
//  RISING
//  FALLING


//AB 霍尔编码器 连接方式  
//M1 --> outA 
//GND  --> UNO GND
//C1    --> UNO 引脚2
//C2   --> UNO 引脚4
//VCC  --> UNO 3.3V
//M1 --> outA 
