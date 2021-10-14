#include <Servo.h>  //舵机库
#include <TuyaWifi.h>   //涂鸦通讯库
#include <SoftwareSerial.h>

//函数定义
  void KEY_Dispose(void);   //按键按下/放开判断函数
  void KEY1_Function(void); //按键1功能处理函数
  void KEY2_Function(void); //按键2功能处理函数
  void Link_Dealt(void);  //配网函数
    
  void Server1_Work(void);  //舵机1工作处理函数
  void Server2_Work(void);  //舵机2工作处理函数

  
  boolean T_2ms_flag = false;

  #define IO_LED 11

  unsigned long nowtime_buffer = 0; //记录上一次时间值
  
//舵机相关变量/常量定义
  #define SET 1
  #define RESET 0

  #define pos_1_off 35 //舵机1 关灯角度值
  #define pos_1_on  115 //舵机1 开灯角度值
  #define pos_1_not 90 //舵机1 常态角度值
  
  #define pos_2_off 55 //舵机2 关灯角度值
  #define pos_2_on  165 //舵机2 开灯角度值
  #define pos_2_not 90 //舵机2 常态角度值
  
  #define Server1_Power_EN 8     // 舵机1供电使能引脚
  #define Server2_Power_EN 5     // 舵机2供电使能引脚

  Servo myserver_1;  //创建一个舵机控制对象
  Servo myserver_2;  //创建一个舵机控制对象

  boolean Lamp1_State = false;  //定义1号灯工作状态标志
  boolean Lamp2_State = false;  //定义2号灯工作状态标志

  boolean lamp1_on_off_EN = false;  //1号灯 开关动作 使能位
  boolean lamp2_on_off_EN = false;  //2号灯 开关动作 使能位

  unsigned char Server1_Work_State;  //定义舵机1的工作状态
  unsigned char Server2_Work_State;  //定义舵机2的工作状态

  unsigned char server1_delay_cnt = 0; //定义舵机1专用延时计数  
  unsigned char server2_delay_cnt = 0; //定义舵机2专用延时计数  

//按键相关变量/常量定义
  #define KEY1_COM 12     // 按键1引脚定义
  #define KEY2_COM 10     // 按键2引脚定义

  #define IO_KEY1 12     // 按键1引脚定义
  #define IO_KEY2 10     // 按键2引脚定义
  
  boolean KEY1_Press_Flag = false;  // 按键1按下标志位
  boolean KEY2_Press_Flag = false;  // 按键2按下标志位

  boolean KEY1_State = false;  // 按键1状态
  boolean KEY2_State = false;  // 按键2状态
  
  unsigned char key_fun_new;  //按键当前状态 寄存器
  unsigned char key_fun_buf; //键值缓存器-上一次按键状态

  unsigned char key_fun_cnt; //按键按下计时器(消抖计时)
  unsigned char key_press_time; //按键按下时长计数(长按计时)
  
//涂鸦模组相关变量/常量定义
  TuyaWifi my_device; //定义一个涂鸦模组名称
  /* 数据点定义  数据点别名  数据点*/
  #define DPID_SWITCH_1 1   //开关1(可下发可上报)
  #define DPID_SWITCH_2 2   //开关2(可下发可上报)
  /*  存储所有DP点及其类型*/
  unsigned char dp_array[][2] =
  {
    {DPID_SWITCH_1, DP_TYPE_BOOL},
    {DPID_SWITCH_2, DP_TYPE_BOOL},
  };
  
  unsigned char pid[] = {"x6tlu1upxn7ple0x"};
  unsigned char mcu_ver[] = {"1.0.2"};  //软件版本信息

  boolean tuya_KEY1_Press_Flag = false;  // 涂鸦按键1按下标志位
  boolean tuya_KEY2_Press_Flag = false;  // 涂鸦按键2按下标志位

  boolean dp_updata_flag = true;  //回传DP点数据标志位
  
  boolean tuya_Link_Flag = false; //涂鸦配网标志位
  unsigned long tuya_link_led_blink_time = 0; //涂鸦配网时LED闪烁专用时间标志
  
//杂项变量/常量定义
  unsigned char print_1, print_2; //打印辅助值

//================================================================================
// 系统初始化
void setup()
{
//串口监视器通讯频率
  Serial.begin(9600);

//按键硬件配置为 输入模式,上拉输入
  pinMode(KEY1_COM, INPUT_PULLUP);
  pinMode(KEY2_COM, INPUT_PULLUP);

//初始化 舵机
  myserver_1.attach(9);  // 该舵机1由arduino第6脚控制
  myserver_2.attach(6);  // 该舵机1由arduino第10脚控制
  //舵机1供电使能脚配置为 输出模式 拉高(关三极管)
  pinMode(Server1_Power_EN, OUTPUT);
  digitalWrite(Server1_Power_EN, HIGH);
  //舵机2供电使能脚配置为 输出模式 拉高(关三极管)
  pinMode(Server2_Power_EN, OUTPUT);
  digitalWrite(Server2_Power_EN, HIGH);

//涂鸦模组初始化
  //输入PID和MCU软件版本
  my_device.init(pid, mcu_ver);
  //输入所有的DP和他们的类型数组，DP数字,!需要手动修改下面的数字,与自己的DP点数相对应
  my_device.set_dp_cmd_total(dp_array, 2);
  //注册DP下载处理回调功能
  my_device.dp_process_func_register(dp_process);
  //注册上传所有DP回调功能
  my_device.dp_update_all_func_register(dp_update_all);
  
//LED(小夜灯)控制脚
  pinMode(IO_LED, OUTPUT);
  digitalWrite(IO_LED, LOW);  //拉低(关掉)
    
}

//================================================================================
void loop()
{
//产生2ms轮询标志位的程序
  unsigned long nowtime = millis(); //获取当前的系统运行时间长度
  if(nowtime == 0)  nowtime_buffer = nowtime; //运行时间溢出时,复位 运行时间缓存值
  if((nowtime - nowtime_buffer) >= 2)
  {
    nowtime_buffer = nowtime; //保存当前arduino的运行时间
    T_2ms_flag = true;
  }   
  
//主功能程序    
  if(T_2ms_flag == true)
  {
  //  Serial.print("2ms\n");
    T_2ms_flag = false;    
    my_device.uart_service(); //涂鸦模块与arduino通讯 数据处理函数

    KEY_Dispose();  //按键按下消抖处理函数
    
    if(lamp1_on_off_EN == SET)  //接收到1号灯的开关动作指令,即进入舵机1的工作函数
      { Server1_Work();}    
    if(lamp2_on_off_EN == SET)  //接收到2号灯的开关动作指令,即进入舵机2的工作函数
      { Server2_Work();}  
           
  }

  if(tuya_Link_Flag == true)  //配网时LED灯闪烁
  {
    Link_Dealt();
  }
}

//================================================================================================
//按键按下判断函数
void KEY_Dispose(void)
{ //按键1消抖/放开处理部分
	/******初始化变量*********/
	key_fun_new = 0;	//清零 按键状态寄存器
	/******读取按键状态*********/
	if(digitalRead(IO_KEY1)==LOW)	key_fun_new |= 0x01;
	if(digitalRead(IO_KEY2)==LOW)	key_fun_new |= 0x02;
	/******按键消抖判断*********/
	if((key_fun_new==key_fun_buf)&&(key_fun_new != 0))
	{	
		if(key_fun_cnt<100) key_fun_cnt++;
		if((key_fun_cnt>=40)&&(key_fun_cnt<100))		//40*2ms=80ms,200*2ms=400ms
		{	
			//按键判断
			switch(key_fun_new)
			{	
				case 0x01:  //按键1按下
                  key_fun_cnt = 100;	//限值处理 确保该功能在按键成立时只进入一次
                  KEY1_Function(); 	//转跳到按键1功能函数
                  break;
				case 0x02:  //按键2按下
                  key_fun_cnt = 100;	//限值处理
                  KEY2_Function();  //转跳到按键2功能函数
                  break;
				case 0x03:  //按键1和按键2同时按下
                  key_fun_cnt = 0;  //按键消抖计数清零
                  key_press_time++;
                  if(key_press_time>38)  //38*80ms=3040ms=3.04秒
                  {
                    key_fun_cnt = 100;	//限值处理
                    key_press_time = 0; //清零按键按下时间计数
                    my_device.mcu_set_wifi_mode(SMART_CONFIG); //启用涂鸦模块配网功能
                    tuya_Link_Flag = true; //使能 涂鸦配网标志位
                  }
                  break;
				default: 	//其他键值成立时,视为干扰
					break;	
			}						
		}
	}
	else
	{	
		key_fun_buf = key_fun_new;	//按键状态缓存器赋值
		key_fun_cnt = 0;	//清零按键按下计数
	}	
}

//================================================================================
//按键1功能函数
void KEY1_Function(void)
{
  Server1_Work_State = 0; //复位舵机1工作状态
  Lamp1_State = !Lamp1_State; //取反1号灯的亮灭状态
  lamp1_on_off_EN = SET; //1号灯 开关动作 开始(使能)
  my_device.mcu_dp_update(DPID_SWITCH_1, Lamp1_State, 1); //回传1号灯的亮灭状态到APP
}

//================================================================================
//按键2功能函数
void KEY2_Function(void)
{
  Server2_Work_State = 0; //复位舵机2工作状态
  Lamp2_State = !Lamp2_State; //取反2号灯的亮灭状态
  lamp2_on_off_EN = SET; //2号灯 开关动作 开始(使能)
  my_device.mcu_dp_update(DPID_SWITCH_2, Lamp2_State, 1); //回传2号灯的亮灭状态到APP
}

//================================================================================
//涂鸦配网函数
void Link_Dealt(void)
{
  unsigned long tuya_nowtime = millis(); //获取当前的系统运行时间长度
    /* 网络连接时LED会闪烁 */
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW)) 
  {
    if (tuya_nowtime- tuya_link_led_blink_time >= 500) //LED每500ms切换一次亮灭状态
    {
      tuya_link_led_blink_time = tuya_nowtime; 
      if (digitalRead(IO_LED) == LOW) 
        { digitalWrite(IO_LED, HIGH);} 
      else 
        { digitalWrite(IO_LED, LOW);}
    }
  }
  else
  {
    digitalWrite(IO_LED, LOW);  //配网结束,熄灭LED灯
    tuya_Link_Flag = false; //除能 涂鸦配网标志位
  }
}

//================================================================================
//舵机1工作处理函数
void Server1_Work(void)
{
  switch(Server1_Work_State)
  {
    case 0:
      myserver_1.attach(9);  // 该舵机1由arduino第6脚控制
      digitalWrite(Server1_Power_EN, HIGH);  //断开舵机1的供电
      
      if (Lamp1_State == true) //判断1号灯的工作状态
        { myserver_1.write(pos_1_on);} //输出PWM波,控制舵机1开灯 
      else
        { myserver_1.write(pos_1_off);} //输出PWM波,控制舵机1关灯

      server1_delay_cnt = 0; //清零舵机1专用延时计数  
      Server1_Work_State = 1; //转到下一个状态
      break;
    case 1:
      server1_delay_cnt++;
      digitalWrite(Server1_Power_EN, LOW);  //给舵机1供电

      if(server1_delay_cnt >= 125)  //125*2ms=250ms延时
      {
        server1_delay_cnt = 0;  //清零延时计数
        digitalWrite(Server1_Power_EN, HIGH);  //断开舵机1的供电
        Server1_Work_State = 2; //转到下一个状态
      } 
      break;
    case 2:
      digitalWrite(Server1_Power_EN, HIGH);  //再次确认 断开舵机1的供电
      myserver_1.write(pos_1_not); //输出PWM波,控制舵机回到平衡水平
      Server1_Work_State = 3; //转到下一个状态
      break;
    case 3:
      server1_delay_cnt++;
      digitalWrite(Server1_Power_EN, LOW);  //给舵机1供电

      if(server1_delay_cnt >= 125)  //125*2ms=250ms延时
      {
        server1_delay_cnt = 0;  //清零延时计数
        digitalWrite(Server1_Power_EN, HIGH);  //断开舵机1的供电
        Server1_Work_State = 4; //转到下一个状态
      } 
      break;      
    case 4:
      digitalWrite(Server1_Power_EN, HIGH);  //再次确认 断开舵机1的供电
      myserver_1.detach();
      lamp1_on_off_EN = RESET; //1号灯 开关动作 结束(除能)
      break;   
    default : Server1_Work_State = 0; break;     
  }

}

//================================================================================
//舵机2工作处理函数
void Server2_Work(void)
{
  switch(Server2_Work_State)
  {
    case 0:
      digitalWrite(Server2_Power_EN, HIGH);  //断开舵机2的供电
      
      if (Lamp2_State == true) //判断2号灯的工作状态
        { myserver_2.write(pos_2_on);} //输出PWM波,控制舵机2开灯 
      else
        { myserver_2.write(pos_2_off);} //输出PWM波,控制舵机2关灯

      server2_delay_cnt = 0; //清零舵机2专用延时计数  
      Server2_Work_State = 1; //转到下一个状态
      break;
    case 1:
      server2_delay_cnt++;
      digitalWrite(Server2_Power_EN, LOW);  //给舵机2供电

      if(server2_delay_cnt >= 125)  //125*2ms=250ms延时
      {
        server2_delay_cnt = 0;  //清零延时计数
        digitalWrite(Server2_Power_EN, HIGH);  //断开舵机2的供电
        Server2_Work_State = 2; //转到下一个状态
      } 
      break;
    case 2:
      digitalWrite(Server2_Power_EN, HIGH);  //再次确认 断开舵机2的供电
      myserver_2.write(pos_2_not); //输出PWM波,控制舵机2回到平衡水平
      Server2_Work_State = 3; //转到下一个状态
      break;
    case 3:
      server2_delay_cnt++;
      digitalWrite(Server2_Power_EN, LOW);  //给舵机2供电

      if(server2_delay_cnt >= 125)  //125*2ms=250ms延时
      {
        server2_delay_cnt = 0;  //清零延时计数
        digitalWrite(Server2_Power_EN, HIGH);  //断开舵机2的供电
        Server2_Work_State = 4; //转到下一个状态
      } 
      break;      
    case 4:
      digitalWrite(Server2_Power_EN, HIGH);  //再次确认 断开舵机2的供电
      lamp2_on_off_EN = RESET; //2号灯 开关动作 结束(除能)
      break;   
    default : Server2_Work_State = 0; break;     
  }
}

//================================================================================
/**
 * @description: DP下载回调功能。
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)
{
print_1 = my_device.mcu_get_dp_download_data(1, value, length);
 print_2 = my_device.mcu_get_dp_download_data(2, value, length); 
  if(dpid==1)
  {
    KEY1_Function();   //转跳到按键1功能函数   
  }
  if(dpid==2)
  {
    KEY2_Function();   //转跳到按键2功能函数    
  }

  return SUCCESS;
}

//================================================================================
/**
 * @description: 上传当前设备的所有DP状态。
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
  my_device.mcu_dp_update(DPID_SWITCH_1, Server1_Work_State, 1);
  my_device.mcu_dp_update(DPID_SWITCH_2, Server2_Work_State, 1);

}
